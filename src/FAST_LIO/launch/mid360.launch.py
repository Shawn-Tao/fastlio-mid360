"""Unified Humble entry point: mapping, localization, or driver-free replay."""
from datetime import datetime
from pathlib import Path
import json
import shlex
import re
import math
import yaml
import zlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

_MAP_REQUIRED = ('mapping.extrinsic_T', 'mapping.extrinsic_R', 'preprocess.lidar_type',
                 'preprocess.timestamp_unit', 'common.time_sync_en', 'common.time_offset_lidar_to_imu',
                 'preprocess.scan_line', 'preprocess.blind', 'preprocess.operator_filter_en',
                 'preprocess.operator_filter_rear_angle', 'preprocess.operator_filter_range_min',
                 'preprocess.operator_filter_range_max', 'mapping.extrinsic_est_en',
                 'preprocess.scan_rate', 'point_filter_num', 'feature_extract_enable',
                 'filter_size_surf', 'filter_size_map', 'mapping.acc_cov', 'mapping.gyr_cov',
                 'mapping.b_acc_cov', 'mapping.b_gyr_cov')


def _compatible(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isfinite(a) and math.isfinite(b) and math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-8)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_compatible(x, y) for x, y in zip(a, b))
    return a == b


def _map_metadata(reference, config, policy):
    if policy not in ('auto', 'strict', 'ignore'):
        raise ValueError('map_metadata must be auto, strict, or ignore')
    if policy == 'ignore':
        return None
    sidecar = Path(str(reference) + '.json')
    if not sidecar.is_file():
        if policy == 'strict':
            raise ValueError(f'Reference map metadata is required: {sidecar}')
        return None  # node warns for legacy maps; never claim they were verified
    try:
        meta = json.loads(sidecar.read_text(encoding='utf-8'))
        if type(meta['schema_version']) is not int or meta['schema_version'] != 1 or meta['frame_id'] != 'camera_init' or meta['pose_frame'] != 'IMU':
            raise ValueError('Unsupported map schema or coordinate frame')
        if meta.get('complete') is not True:
            raise ValueError('Reference map is marked incomplete (capacity limit reached)')
        if type(meta.get('points')) is not int or meta['points'] <= 0:
            raise ValueError('Invalid map point count')
        gravity = meta['gravity']
        if not isinstance(gravity, list) or len(gravity) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in gravity) or sum(v*v for v in gravity) < 1e-12:
            raise ValueError('Invalid reference-map gravity')
        # PCD and JSON are individually atomic, not a two-file transaction. Check
        # the header against the manifest to detect interrupted snapshot pairs.
        pcd_points = None
        with reference.open('rb') as stream:
            for _ in range(100):
                line = stream.readline(4096)
                if line.startswith(b'POINTS '):
                    pcd_points = int(line.split()[1])
                if line.startswith(b'DATA '):
                    break
        if pcd_points != meta['points']:
            raise ValueError('PCD point count does not match metadata; snapshot pair may be interrupted')
        if reference.stat().st_size != meta['pcd_bytes']:
            raise ValueError('PCD file size does not match metadata')
        checksum = 0
        with reference.open('rb') as stream:
            while chunk := stream.read(1024 * 1024):
                checksum = zlib.crc32(chunk, checksum)
        if f'{checksum:08x}' != meta['pcd_crc32']:
            raise ValueError('PCD checksum does not match metadata; corrupted/interrupted snapshot pair')
        saved = meta['parameters']
        for name in _MAP_REQUIRED:
            current = config
            for part in name.split('.'):
                current = current[part]
            if name not in saved or not _compatible(saved[name], current):
                raise ValueError(f'Map calibration/time configuration mismatch: {name}')
        return meta
    except (OSError, ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
        raise ValueError(f'Invalid/incompatible reference-map metadata {sidecar}: {error}') from error


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
    if not localization:
        side = config_params.get('cube_side_length', 400.0)
        detection = config_params.get('mapping', {}).get('det_range', 100.0)
        if not math.isfinite(side) or not math.isfinite(detection) or detection <= 0 or side <= 3 * detection:
            raise ValueError('cube_side_length must be > 3 * mapping.det_range; invalid stationary local-map window')
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
        metadata = _map_metadata(reference, config_params, args['map_metadata'])
        params['localization.metadata_verified'] = metadata is not None
        if metadata is not None:
            params['localization.map_gravity'] = [float(value) for value in metadata['gravity']]
        if args['record_dir']:
            params['record.dir'] = str(Path(args['record_dir']).expanduser().resolve())
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
        if args['publish_map'] != 'auto':
            params['publish.map_en'] = _boolean(context, 'publish_map')
        elif _boolean(context, 'rviz'):
            params['publish.map_en'] = True  # bounded, subscriber-aware display only
        if args['record_dir']:
            params['record.dir'] = str(Path(args['record_dir']).expanduser().resolve())
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
        'map_metadata': ('auto', 'auto validates an existing .pcd.json; strict requires it; ignore explicitly bypasses metadata validation'),
        'record_dir': ('', 'CSV directory; default workspace records/ (persistent with the Docker workspace bind mount)'),
        'relocalize': ('auto', 'auto uses YAML/default enabled; true performs bounded startup matching, false retains manual initial_pose'),
        'search_radius': ('', 'Override startup position search radius in metres around YAML relocalization.center (default map origin, 3 m)'),
        'map_name': ('map', 'Scene name for mapping output: timestamp_<name>.pcd; no extension or path separators'),
        'map_dir': ('', 'Optional mapping output directory; default is pcd_map/ beside workspace src/'),
        'map_output': ('', 'Advanced override: new full .pcd output path; shared by /map_save and Ctrl+C'),
        'rviz': ('false', 'Start RViz (requires a working display)'),
        'publish_map': ('auto', 'Mapping display: auto enables bounded map when RViz starts; true supports remote RViz; false disables'),
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
