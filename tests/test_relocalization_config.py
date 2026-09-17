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
