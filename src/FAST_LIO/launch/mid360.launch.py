"""Unified Humble entry point: mapping, localization, or driver-free replay."""
from datetime import datetime
from pathlib import Path
import json
import shlex
import re
import math
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


def _boolean(context, name):
    value = context.launch_configurations[name].lower()
    if value not in ('true', 'false', '1', '0'):
        raise ValueError(f'{name} must be true or false, got {value!r}')
    return value in ('true', '1')


def _launch(context):
    args = context.launch_configurations
    mode = args['mode']
    if mode not in ('mapping', 'localization', 'replay'):
        raise ValueError('mode must be mapping, localization, or replay')
    driver = mode != 'replay' if args['with_driver'] == 'auto' else _boolean(context, 'with_driver')
    fast_share = Path(get_package_share_directory('fast_lio'))
    driver_share = Path(get_package_share_directory('livox_ros_driver2'))
    config = args['config_file'] or ('mid360_localization.yaml' if mode == 'localization' else 'mid360.yaml')
    config = Path(config).expanduser()
    if not config.is_absolute():
        # Bare YAML names remain package-relative; explicit relative paths
        # (e.g. config/local/scene.local.yaml) follow the launch CWD.
        config = config.resolve() if len(config.parts) > 1 else fast_share / 'config' / config
    if not config.is_file():
        raise ValueError(f'FAST-LIO config does not exist: {config}')
    config_params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    localization = mode == 'localization' or config_params.get('localization', {}).get('mode', False)
    params = {'use_sim_time': _boolean(context, 'use_sim_time'),
              'localization.mode': localization}
    if localization:
        if not args['map_path'].strip():
            raise ValueError('Localization requires an explicit map_path:=/path/to/scene.pcd launch argument; no default map is selected.')
        reference = Path(args['map_path']).expanduser().resolve()
        if reference.suffix.lower() != '.pcd' or not reference.is_file():
            raise ValueError(f'Reference map must be an existing .pcd file: {reference}')
        try:
            with reference.open('rb') as stream:
                if not stream.read(1):
                    raise ValueError(f'Reference map is empty: {reference}')
        except OSError as error:
            raise ValueError(f'Reference map is not readable: {reference}') from error
        if args['map_output']:
            raise ValueError('map_output cannot be used in localization: reference maps are read-only.')
        params['localization.map_path'] = str(reference)
        if args['relocalize'] != 'auto':
            params['localization.relocalization.enabled'] = _boolean(context, 'relocalize')
        if args['search_radius']:
            try:
                radius = float(args['search_radius'])
            except ValueError as error:
                raise ValueError('search_radius must be a finite positive number in metres') from error
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError('search_radius must be a finite positive number in metres')
            params['localization.relocalization.radius'] = radius
    elif args['search_radius'] or args['relocalize'] != 'auto':
        raise ValueError('relocalize and search_radius are localization-only arguments')
    if not localization:
        if not re.fullmatch(r'[\w-]+', args['map_name']):
            raise ValueError('map_name must use letters, digits, underscores or hyphens (no .pcd extension or path separators).')
        params['map_name'] = args['map_name']
        # Reset old/custom YAML fixed filenames; the node generates one timestamp
        # per session, shared by /map_save and graceful Ctrl+C automatic saving.
        params['map_file_path'] = ''
        if args['map_dir']:
            params['map_dir'] = str(Path(args['map_dir']).expanduser().resolve())
        if args['map_output']:
            output = Path(args['map_output']).expanduser().resolve()
            if output.suffix != '.pcd' or output.exists():
                raise ValueError(f'map_output must be a new .pcd path, not an existing file: {output}')
            params['map_file_path'] = str(output)
    actions = []
    if driver:
        lidar_config = (Path(args['lidar_config']).expanduser().resolve()
                        if args['lidar_config'] else driver_share / 'config' / 'MID360.json')
        if not lidar_config.is_file():
            raise ValueError(f'Livox config does not exist: {lidar_config}')
        try:
            if not isinstance(json.loads(lidar_config.read_text(encoding='utf-8')), dict):
                raise ValueError('Expected a JSON object')
        except (OSError, ValueError) as error:
            raise ValueError(f'Livox config must be a readable JSON object: {lidar_config}: {error}') from error
        actions.append(Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen',
            parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                         'publish_freq': 10.0, 'output_data_type': 0,
                         'frame_id': 'livox_frame', 'lvx_file_path': '',
                         'user_config_path': str(lidar_config.resolve()),
                         'cmdline_input_bd_code': 'livox0000000001'}]))
    actions.append(Node(package='fast_lio', executable='fastlio_mapping',
                        name='laser_mapping', output='screen',
                        parameters=[str(config), params]))
    if _boolean(context, 'rviz'):
        rviz_config = args['rviz_cfg'] or str(fast_share / 'rviz' / 'fastlio.rviz')
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', rviz_config], output='screen'))
    if _boolean(context, 'record_bag'):
        # Argument list, not bash interpolation; only create directories when recording.
        bag_root = Path(args['bag_dir']).expanduser().resolve()
        bag_root.mkdir(parents=True, exist_ok=True)
        bag_path = bag_root / f'{mode}_{datetime.now():%Y%m%d_%H%M%S_%f}'
        topics = list(dict.fromkeys(['/livox/lidar', '/livox/imu', '/Odometry']
                                    + shlex.split(args['extra_bag_topics'])))
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', str(bag_path), *topics], output='screen'))
    return actions


def generate_launch_description():
    defaults = {
        'mode': ('mapping', 'mapping | localization | replay (replay starts FAST-LIO only)'),
        'with_driver': ('auto', 'auto follows mode; false starts FAST-LIO without hardware'),
        'config_file': ('', 'YAML basename in fast_lio/config, or absolute/CWD-relative path such as config/local/scene.local.yaml'),
        'lidar_config': ('', 'Absolute/CWD-relative Livox JSON path; local copies are explicit opt-in, factory MID360.json remains the default'),
        'map_path': ('', 'REQUIRED for localization, including replay of a localization YAML; existing reference .pcd path'),
        'relocalize': ('auto', 'auto uses YAML/default enabled; true performs bounded startup matching, false retains manual initial_pose'),
        'search_radius': ('', 'Override startup position search radius in metres around YAML relocalization.center (default map origin, 3 m)'),
        'map_name': ('map', 'Scene name for mapping output: timestamp_<name>.pcd; no extension or path separators'),
        'map_dir': ('', 'Optional mapping output directory; default is pcd_map/ beside workspace src/'),
        'map_output': ('', 'Advanced override: new full .pcd output path; shared by /map_save and Ctrl+C'),
        'rviz': ('false', 'Start RViz (requires a working display)'),
        'rviz_cfg': ('', 'Optional RViz configuration path'),
        'use_sim_time': ('false', 'Set true when replaying a bag with --clock'),
        'record_bag': ('false', 'Record raw LiDAR, IMU and odometry; opt-in to avoid unexpected disk use'),
        'bag_dir': ('./bags', 'Directory for rosbag sessions'),
        'extra_bag_topics': ('', 'Additional space-separated topic names to record'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()),
        OpaqueFunction(function=_launch),
    ])
