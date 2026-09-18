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

# Empty launch values preserve YAML/declared defaults. Bounds and cross-checks
# run before creating actions, not after a real driver has already started.
_STATIC_NUMERIC = {
    'confirm_hits': ('static_map.min_observations', 'count', 3),
    'confirm_seconds': ('static_map.confirmation_seconds', 'span', 0.6),
    'confirm_interval': ('static_map.observation_interval', 'span', 0.1),
    'candidate_ttl': ('static_map.candidate_ttl', 'positive', 5.0),
    'clear_hits': ('static_map.clear_observations', 'count', 3),
    'clear_seconds': ('static_map.clear_seconds', 'span', 0.4),
    'clear_interval': ('static_map.clear_observation_interval', 'interval', -1.0),
    'clear_vote_ttl': ('static_map.clear_vote_ttl', 'positive', 3.0),
    'clear_range': ('static_map.clear_max_range', 'positive', 15.0),
    'clear_endpoint_margin': ('static_map.endpoint_margin', 'positive', 0.5),
    'clear_ray_clearance': ('static_map.ray_clearance', 'positive', 0.025),
    'clear_max_speed': ('static_map.max_clear_speed', 'positive', 1.0),
    'clear_max_angular_speed': ('static_map.max_clear_angular_speed', 'positive', 1.0),
    'clear_max_position_std': ('static_map.max_clear_position_std', 'positive', 0.10),
    'clear_max_frame_gap': ('static_map.max_clear_frame_gap', 'positive', 0.5),
    'clear_max_rays': ('static_map.max_rays', 'count', 256),
    'clear_max_ray_steps': ('static_map.max_ray_steps', 'count', 600),
    'map_max_candidates': ('static_map.max_candidates', 'count', 250000),
    'map_max_frame_points': ('static_map.max_frame_points', 'count', 100000),
}


def _config_value(config, name, default):
    if name in config:  # ROS also permits flattened/dotted parameter names
        return config[name]
    value = config
    for part in name.split('.'):
        if not isinstance(value, dict) or part not in value:
            return default
        value = value[part]
    return value


def _static_number(name, value, kind):
    if kind == 'count':
        valid = type(value) is int and 1 <= value <= 2147483647
    else:
        valid = type(value) in (int, float) and math.isfinite(value)
        if valid:
            valid = (value > 0 if kind == 'positive' else
                     (value == -1 or value >= 0) if kind == 'interval' else value >= 0)
    if not valid:
        bound = {'count': 'integer in [1, 2147483647]', 'positive': 'finite and > 0',
                 'span': 'finite and >= 0', 'interval': '-1 (inherit) or finite and >= 0'}[kind]
        raise ValueError(f'{name} must be {bound}, got {value!r}')
    return value if kind == 'count' else float(value)


def _static_parameters(context, config, localization):
    args = context.launch_configurations
    switches = {'static_filter': 'static_map.enabled',
                'map_confirm': 'static_map.confirmation_enabled',
                'ray_clear': 'static_map.clearing_enabled'}
    numeric = {name: args[name].strip() for name in _STATIC_NUMERIC if args[name].strip()}
    if localization:
        if numeric or any(args[name] != 'auto' for name in switches):
            raise ValueError('static_filter/map_confirm/ray_clear and static thresholds are mapping-only; localization reference maps are read-only')
        return {'static_map.enabled': False}
    params, active = {}, {}
    for name, key in switches.items():
        active[key] = _config_value(config, key, True)
        if args[name] != 'auto':
            active[key] = params[key] = _boolean(context, name)
        if type(active[key]) is not bool:
            raise ValueError(f'{key} must be a YAML boolean')
    for name, (key, kind, default) in _STATIC_NUMERIC.items():
        value = _config_value(config, key, default)
        if name in numeric:
            try:
                if kind == 'count' and not re.fullmatch(r'[+-]?\d+', numeric[name]):
                    raise ValueError('not an integer')
                value = int(numeric[name]) if kind == 'count' else float(numeric[name])
            except (ValueError, OverflowError) as error:
                raise ValueError(f'{name} is not a valid {kind} value: {numeric[name]!r}') from error
            params[key] = _static_number(name, value, kind)
        active[key] = _static_number(key, value, kind)
    if not active['static_map.enabled']:
        if numeric or any(args[name] != 'auto' and active[key] for name, key in switches.items() if name != 'static_filter'):
            raise ValueError('static_filter master switch is false; use static_filter:=true before enabling/tuning individual stages')
        return params
    voxel = _config_value(config, 'pcd_save.voxel_size', 0.1)
    cap = _config_value(config, 'pcd_save.max_points', 2000000)
    if type(voxel) not in (int, float) or not math.isfinite(voxel) or voxel < 0.0001 or type(cap) is not int or cap <= 0:
        raise ValueError('static_map requires positive bounded pcd_save.voxel_size/max_points; use static_filter:=false for raw/unbounded mode')
    if active['static_map.confirmation_enabled'] and active['static_map.candidate_ttl'] <= active['static_map.confirmation_seconds']:
        raise ValueError('candidate_ttl must exceed confirm_seconds when confirmation is enabled')
    if active['static_map.clearing_enabled']:
        if active['static_map.clear_vote_ttl'] <= active['static_map.clear_seconds']:
            raise ValueError('clear_vote_ttl must exceed clear_seconds')
        if active['static_map.endpoint_margin'] >= active['static_map.clear_max_range']:
            raise ValueError('clear_endpoint_margin must be less than clear_range')
        if active['static_map.ray_clearance'] > voxel / 2:
            raise ValueError('clear_ray_clearance must be <= pcd_save.voxel_size/2')
    return params


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
    params.update(_static_parameters(context, config_params, localization))
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
        rviz_config = args['rviz_cfg'] or str(fast_share / 'rviz' / ('localization.rviz' if localization else 'mapping.rviz'))
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
        'static_filter': ('auto', 'Mapping/archive-only temporal static filter; auto uses YAML (default enabled); false restores legacy voxel archive'),
        'map_confirm': ('auto', 'Mapping-only admission confirmation stage: auto uses YAML, true/false independently enables/disables'),
        'ray_clear': ('auto', 'Mapping-only free-ray cleanup stage: auto uses YAML, true/false independently enables/disables'),
        **{name: ('', f'Mapping-only override {key} ({kind}); empty preserves YAML/default')
           for name, (key, kind, _) in _STATIC_NUMERIC.items()},
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
