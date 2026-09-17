#!/usr/bin/env python3
"""Host-only launch/configuration contracts, using explicit ROS action stubs.

No ROS installation or processes are needed. This tests our launch parameter
dataflow, NOT ROS action execution, C++ node compilation or sensor performance.
Run test_workspace.py under Humble as well for real ROS launch action coverage.
"""
import importlib.util
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


if __name__ == '__main__':
    unittest.main()
