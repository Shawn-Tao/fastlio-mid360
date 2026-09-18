#!/usr/bin/env python3
"""Host-only launch/configuration contracts, using explicit ROS action stubs.

No ROS installation or processes are needed. This tests our launch parameter
dataflow, NOT ROS action execution, C++ node compilation or sensor performance.
Run test_workspace.py under Humble as well for real ROS launch action coverage.
"""
import importlib.util
import copy
import json
import tempfile
import zlib
import yaml
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class Declaration:
    def __init__(self, name, default_value, description):
        self.name, self.default_value = name, default_value

    def execute(self, context):
        context.launch_configurations.setdefault(self.name, self.default_value)


class RelocalizationConfigTests(unittest.TestCase):
    def actions(self, **arguments):
        def module(name, **fields):
            value = ModuleType(name)
            value.__dict__.update(fields)
            return value
        stubs = {
            'ament_index_python': module('ament_index_python'),
            'ament_index_python.packages': module('ament_index_python.packages',
                get_package_share_directory=lambda name: str(ROOT / 'src' / (
                    'FAST_LIO' if name == 'fast_lio' else 'livox_ros_driver2'))),
            'launch': module('launch', LaunchDescription=lambda entities: SimpleNamespace(entities=entities)),
            'launch.actions': module('launch.actions', DeclareLaunchArgument=Declaration,
                ExecuteProcess=lambda **kw: kw, OpaqueFunction=lambda **kw: kw),
            'launch_ros': module('launch_ros'),
            'launch_ros.actions': module('launch_ros.actions', Node=lambda **kw: kw),
        }
        with patch.dict(sys.modules, stubs):
            path = ROOT / 'src/FAST_LIO/launch/mid360.launch.py'
            spec = importlib.util.spec_from_file_location('host_relocalization_launch', path)
            launch = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(launch)
            context = SimpleNamespace(launch_configurations={})
            for action in launch.generate_launch_description().entities:
                if isinstance(action, Declaration):
                    action.execute(context)
            context.launch_configurations.update(arguments)
            return launch._launch(context)

    def test_default_map_only_localization(self):
        actions = self.actions(mode='localization', map_path=str(ROOT / 'pcd_map/test.pcd'))
        self.assertEqual(len(actions), 2)
        self.assertEqual(actions[1]['parameters'][1]['localization.map_path'], str(ROOT / 'pcd_map/test.pcd'))

    def test_static_filter_archive_only_override(self):
        for mode in ('mapping', 'replay'):
            for value, expected in (('true', True), ('false', False)):
                node = self.actions(mode=mode, static_filter=value)[-1]
                self.assertEqual(node['parameters'][1]['static_map.enabled'], expected)
            self.assertNotIn('static_map.enabled', self.actions(mode=mode)[-1]['parameters'][1])
        options = self.actions(mode='localization', map_path=str(ROOT / 'pcd_map/test.pcd'))[-1]['parameters'][1]
        self.assertFalse(options['static_map.enabled'])
        with self.assertRaisesRegex(ValueError, 'mapping-only'):
            self.actions(mode='localization', map_path=str(ROOT / 'pcd_map/test.pcd'), static_filter='true')
        with self.assertRaisesRegex(ValueError, 'mapping-only'):
            self.actions(mode='replay', config_file='mid360_localization.yaml',
                         map_path=str(ROOT / 'pcd_map/test.pcd'), static_filter='false')
        with self.assertRaisesRegex(ValueError, 'true or false'):
            self.actions(mode='mapping', static_filter='yes')

    def test_static_independent_switches_and_typed_thresholds(self):
        for mode in ('mapping', 'replay'):
            for confirm in ('true', 'false'):
                for clear in ('true', 'false'):
                    params = self.actions(mode=mode, map_confirm=confirm, ray_clear=clear)[-1]['parameters'][1]
                    self.assertEqual(params['static_map.confirmation_enabled'], confirm == 'true')
                    self.assertEqual(params['static_map.clearing_enabled'], clear == 'true')
            params = self.actions(mode=mode, confirm_hits='3', confirm_seconds='0.6', confirm_interval='0.1',
                clear_hits='3', clear_seconds='0.4', clear_interval='0.15', clear_max_speed='1',
                clear_max_angular_speed='1.2', clear_max_position_std='0.2', clear_max_frame_gap='1')[-1]['parameters'][1]
            self.assertEqual(params['static_map.min_observations'], 3)
            self.assertIs(type(params['static_map.min_observations']), int)
            for key, value in {'confirmation_seconds': .6, 'observation_interval': .1,
                'clear_observation_interval': .15, 'max_clear_speed': 1.0,
                'max_clear_frame_gap': 1.0, 'max_clear_angular_speed': 1.2}.items():
                self.assertEqual(params['static_map.' + key], value)
                self.assertIs(type(params['static_map.' + key]), float)

    def test_static_one_hit_and_zero_spans_are_explicitly_allowed(self):
        params = self.actions(confirm_hits='1', confirm_seconds='0', confirm_interval='0',
                              clear_hits='1', clear_seconds='0', clear_interval='0')[-1]['parameters'][1]
        self.assertEqual(params['static_map.min_observations'], 1)
        self.assertEqual(params['static_map.clear_observations'], 1)
        self.assertEqual(params['static_map.clear_seconds'], 0.0)
        params = self.actions(clear_interval='-1')[-1]['parameters'][1]
        self.assertEqual(params['static_map.clear_observation_interval'], -1.0)

    def test_static_invalid_thresholds_and_contradictions_fail_early(self):
        cases = ({'confirm_hits': '0'}, {'clear_hits': '1.5'}, {'confirm_hits': '2147483648'},
                 {'confirm_seconds': '-.1'}, {'clear_seconds': 'nan'}, {'clear_max_speed': '0'},
                 {'clear_max_angular_speed': 'inf'}, {'clear_interval': '-.5'},
                 {'confirm_seconds': '5'}, {'clear_seconds': '3'}, {'clear_ray_clearance': '.06'},
                 {'clear_range': '.4'}, {'static_filter': 'false', 'map_confirm': 'true'},
                 {'static_filter': 'false', 'clear_hits': '3'}, {'ray_clear': 'yes'})
        for arguments in cases:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.actions(**arguments)
        # Inactive-stage pair relationships don't prevent testing the other stage.
        self.actions(map_confirm='false', confirm_seconds='10')
        self.actions(ray_clear='false', clear_seconds='10', clear_ray_clearance='.06')

    def test_static_overrides_rejected_in_localization_and_localization_replay(self):
        for mode, config in (('localization', 'mid360_localization.yaml'), ('replay', 'mid360_localization.yaml')):
            for arguments in ({'map_confirm': 'false'}, {'ray_clear': 'false'}, {'clear_hits': '3'}, {'confirm_seconds': '.6'}):
                with self.subTest(mode=mode, arguments=arguments), self.assertRaisesRegex(ValueError, 'mapping-only'):
                    self.actions(mode=mode, config_file=config, map_path=str(ROOT/'pcd_map/test.pcd'), **arguments)

    def test_static_local_yaml_values_preserved_and_cli_overrides_only_selected(self):
        config = yaml.safe_load((ROOT/'src/FAST_LIO/config/mid360.yaml').read_text())
        policy = config['/**']['ros__parameters']['static_map']
        policy.update(confirmation_enabled=False, clearing_enabled=True, min_observations=2,
                      confirmation_seconds=.3, clear_observation_interval=.1, max_clear_speed=1.1)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'mapping.local.yaml'; path.write_text(yaml.safe_dump(config))
            params=self.actions(config_file=str(path))[-1]['parameters'][1]
            self.assertFalse(any(key.startswith('static_map.') for key in params))
            params=self.actions(config_file=str(path), map_confirm='true', clear_hits='4')[-1]['parameters'][1]
            self.assertTrue(params['static_map.confirmation_enabled'])
            self.assertEqual(params['static_map.clear_observations'], 4)
            self.assertNotIn('static_map.max_clear_speed', params)
            self.assertEqual(yaml.safe_load(path.read_text()), config)

    def test_static_legacy_strict_local_yaml_is_not_silently_relaxed(self):
        config = yaml.safe_load((ROOT/'src/FAST_LIO/config/mid360.yaml').read_text())
        policy = config['/**']['ros__parameters']['static_map']
        policy.update(min_observations=4, confirmation_seconds=1.2, observation_interval=.2,
                      clear_observations=6, clear_seconds=1.0, max_clear_speed=.5, max_clear_angular_speed=.3)
        for key in ('confirmation_enabled', 'clearing_enabled', 'clear_observation_interval', 'max_clear_frame_gap'):
            policy.pop(key)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'strict.local.yaml'; path.write_text(yaml.safe_dump(config))
            for mode in ('mapping', 'replay'):
                node=self.actions(mode=mode, config_file=str(path))[-1]
                self.assertEqual(node['parameters'][0], str(path))
                self.assertFalse(any(key.startswith('static_map.') for key in node['parameters'][1]))
            params=self.actions(config_file=str(path), confirm_hits='3', clear_interval='0.1')[-1]['parameters'][1]
            self.assertEqual(params['static_map.min_observations'], 3)
            self.assertEqual(params['static_map.clear_observation_interval'], .1)
            self.assertEqual(yaml.safe_load(path.read_text()), config)

    def test_radius_and_manual_override(self):
        actions = self.actions(mode='localization', map_path=str(ROOT / 'pcd_map/test.pcd'),
                               search_radius='2.5', relocalize='true')
        options = actions[1]['parameters'][1]
        self.assertEqual(options['localization.relocalization.radius'], 2.5)
        self.assertTrue(options['localization.relocalization.enabled'])
        actions = self.actions(mode='localization', map_path=str(ROOT / 'pcd_map/test.pcd'), relocalize='false')
        self.assertFalse(actions[1]['parameters'][1]['localization.relocalization.enabled'])

    def test_invalid_radius_and_mode_rejected(self):
        for radius in ('0', '-3', 'nan', 'inf', 'abc'):
            with self.assertRaisesRegex(ValueError, 'finite positive'):
                self.actions(mode='localization', map_path=str(ROOT / 'pcd_map/test.pcd'), search_radius=radius)
        with self.assertRaisesRegex(ValueError, 'localization-only'):
            self.actions(mode='mapping', search_radius='3')
        with self.assertRaisesRegex(ValueError, 'true or false'):
            self.actions(mode='localization', map_path=str(ROOT / 'pcd_map/test.pcd'), relocalize='yes')

    def test_localization_replay_uses_same_startup_gate(self):
        actions = self.actions(mode='replay', config_file='mid360_localization.yaml',
                               map_path=str(ROOT / 'pcd_map/test.pcd'), search_radius='3')
        self.assertEqual(len(actions), 1)
        self.assertTrue(actions[0]['parameters'][1]['localization.mode'])
        self.assertEqual(actions[0]['parameters'][1]['localization.relocalization.radius'], 3.0)

    def test_rviz2_is_opt_in(self):
        base = {'mode': 'localization', 'map_path': str(ROOT / 'pcd_map/test.pcd')}
        for selection in ({}, {'rviz': 'false'}):
            actions = self.actions(**base, **selection)
            self.assertFalse(any(action.get('package') == 'rviz2' for action in actions))
        actions = self.actions(**base, rviz='true')
        rviz = [action for action in actions if action.get('package') == 'rviz2']
        self.assertEqual(len(rviz), 1)
        self.assertEqual(rviz[0]['executable'], 'rviz2')
        self.assertTrue(rviz[0]['arguments'][1].endswith('/rviz/localization.rviz'))
        actions = self.actions(mode='mapping', rviz='true')
        mapping = next(action for action in actions if action.get('package') == 'fast_lio')
        self.assertTrue(mapping['parameters'][1]['publish.map_en'])
        rviz = next(action for action in actions if action.get('package') == 'rviz2')
        self.assertTrue(rviz['arguments'][1].endswith('/rviz/mapping.rviz'))
        mapping = self.actions(mode='mapping', publish_map='true')[1]
        self.assertTrue(mapping['parameters'][1]['publish.map_en'])
        mapping = self.actions(mode='mapping', rviz='true', publish_map='false')[1]
        self.assertFalse(mapping['parameters'][1]['publish.map_en'])

    def metadata_fixture(self, directory):
        reference = Path(directory) / 'scene.pcd'
        reference.write_bytes(b'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3 4\n')
        config = yaml.safe_load((ROOT / 'src/FAST_LIO/config/mid360_localization.yaml').read_text())['/**']['ros__parameters']
        parameters = {}
        def flatten(prefix, value):
            for key, item in value.items():
                name = f'{prefix}.{key}' if prefix else key
                if isinstance(item, dict): flatten(name, item)
                else: parameters[name] = item
        flatten('', config)
        metadata = {'schema_version': 1, 'frame_id': 'camera_init', 'pose_frame': 'IMU',
                    'complete': True, 'points': 1, 'gravity': [0.2, 0.1, -9.8], 'parameters': parameters,
                    'pcd_bytes': reference.stat().st_size, 'pcd_crc32': f'{zlib.crc32(reference.read_bytes()):08x}'}
        return reference, metadata

    def test_map_manifest_and_record_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            reference, meta = self.metadata_fixture(directory)
            Path(str(reference) + '.json').write_text(json.dumps(meta))
            options = self.actions(mode='localization', map_path=str(reference), map_metadata='strict', record_dir=directory)[1]['parameters'][1]
            self.assertTrue(options['localization.metadata_verified'])
            self.assertEqual(options['localization.map_gravity'], meta['gravity'])
            self.assertEqual(options['record.dir'], directory)

    def test_manifest_rejects_incomplete_calibration_and_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            reference, valid = self.metadata_fixture(directory)
            for mutate in ('incomplete', 'extrinsic', 'points', 'crc', 'gravity'):
                meta = copy.deepcopy(valid)
                if mutate == 'incomplete': meta['complete'] = False
                elif mutate == 'extrinsic': meta['parameters']['mapping.extrinsic_T'] = [1, 2, 3]
                elif mutate == 'points': meta['points'] = 2
                elif mutate == 'crc': meta['pcd_crc32'] = '00000000'
                elif mutate == 'gravity': meta['gravity'] = [0, 0, 0]
                Path(str(reference) + '.json').write_text(json.dumps(meta))
                with self.assertRaises(ValueError): self.actions(mode='localization', map_path=str(reference))
            actions = self.actions(mode='localization', map_path=str(reference), map_metadata='ignore')
            self.assertFalse(actions[1]['parameters'][1]['localization.metadata_verified'])

    def test_legacy_map_policy_and_invalid_window(self):
        with tempfile.TemporaryDirectory() as directory:
            reference, _ = self.metadata_fixture(directory)
            self.actions(mode='localization', map_path=str(reference))
            with self.assertRaisesRegex(ValueError, 'metadata is required'):
                self.actions(mode='localization', map_path=str(reference), map_metadata='strict')
            config = yaml.safe_load((ROOT / 'src/FAST_LIO/config/mid360.yaml').read_text())
            config['/**']['ros__parameters']['cube_side_length'] = 100
            path = Path(directory) / 'bad.yaml'; path.write_text(yaml.safe_dump(config))
            with self.assertRaisesRegex(ValueError, 'stationary local-map window'):
                self.actions(mode='mapping', config_file=str(path))


if __name__ == '__main__':
    unittest.main()
