#!/usr/bin/env python3
"""Configuration contracts, tested by colcon without a LiDAR."""
import ast
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
FAST = ROOT / 'src' / 'FAST_LIO'
DRIVER = ROOT / 'src' / 'livox_ros_driver2'


class WorkspaceTests(unittest.TestCase):
    def config(self, name):
        return yaml.safe_load((FAST / 'config' / name).read_text())['/**']['ros__parameters']

    def test_colcon_dependency_order(self):
        package = ET.parse(FAST / 'package.xml').getroot()
        self.assertIn('livox_ros_driver2', [el.text for el in package.findall('depend')])
        self.assertEqual(ET.parse(DRIVER / 'package.xml').findtext('export/build_type'), 'ament_cmake')

    def test_physics_identical(self):
        mapping = self.config('mid360.yaml')
        localization = self.config('mid360_localization.yaml')
        for key in ('preprocess', 'mapping', 'common', 'feature_extract_enable',
                    'point_filter_num', 'filter_size_surf'):
            self.assertEqual(mapping[key], localization[key], key)
        self.assertFalse(mapping['localization']['mode'])
        self.assertTrue(localization['localization']['mode'])
        self.assertFalse(localization['pcd_save']['pcd_save_en'])

    def test_runtime_safety_defaults(self):
        mapping = self.config('mid360.yaml')
        self.assertGreater(mapping['cube_side_length'], 3 * mapping['mapping']['det_range'])
        self.assertFalse(mapping['publish']['map_en'])
        self.assertEqual(mapping['pcd_save']['voxel_size'], 0.1)
        self.assertGreater(mapping['pcd_save']['max_points'], 0)
        self.assertGreater(mapping['record']['flush_interval_sec'], 0)
        self.assertTrue(self.config('mid360_localization.yaml')['localization']['relocalization']['gravity_alignment'])

    def test_topics_and_custom_message(self):
        mapping = self.config('mid360.yaml')
        self.assertEqual(mapping['preprocess']['lidar_type'], 1)
        self.assertEqual(mapping['common']['lid_topic'], '/livox/lidar')
        self.assertEqual(mapping['common']['imu_topic'], '/livox/imu')
        launch_source = (FAST / 'launch' / 'mid360.launch.py').read_text()
        self.assertIn("'xfer_format': 1", launch_source)

    def test_bundled_files(self):
        config = json.loads((DRIVER / 'config' / 'MID360.json').read_text())
        lidar_ip = config['lidar_configs'][0]['ip']
        self.assertIn(lidar_ip, config['MID360']['host_net_info'][0]['lidar_ip'])
        for arch in ('x86', 'arm'):
            sdk = ROOT / f'livox-sdk-{arch}'
            self.assertTrue((sdk / 'include' / 'livox_lidar_api.h').is_file())
            self.assertTrue((sdk / 'lib' / 'liblivox_lidar_sdk_shared.so').is_file())
        self.assertEqual(self.config('mid360_localization.yaml')['localization']['map_path'], '')
        self.assertTrue((ROOT / 'pcd_map' / 'test.pcd').is_file())
        self.assertEqual(self.config('mid360.yaml')['map_file_path'], '')
        self.assertTrue((FAST / 'include' / 'ikd-Tree' / 'ikd_Tree.cpp').is_file())

    def test_launch_syntax(self):
        for path in [*(FAST / 'launch').glob('*.py'), *(DRIVER / 'launch_ROS2').glob('*.py')]:
            ast.parse(path.read_text(), filename=str(path))

    def launch_actions(self, inspect_nodes=False, **arguments):
        # Construct actions only: no processes, network traffic or bag writes.
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        path = FAST / 'launch' / 'mid360.launch.py'
        spec = importlib.util.spec_from_file_location('mid360_test_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        context.launch_configurations.update(arguments)
        with patch.object(module, 'get_package_share_directory',
                          side_effect=lambda name: str(FAST if name == 'fast_lio' else DRIVER)):
            if inspect_nodes:
                with patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
                    return module._launch(context)
            return module._launch(context)

    def test_launch_modes(self):
        self.assertEqual(len(self.launch_actions(mode='mapping')), 2)
        self.assertEqual(len(self.launch_actions(mode='localization', map_path=str(ROOT / 'pcd_map' / 'test.pcd'))), 2)
        self.assertEqual(len(self.launch_actions(mode='replay')), 1)
        self.assertEqual(len(self.launch_actions(mode='mapping', with_driver='false')), 1)
        self.assertEqual(len(self.launch_actions(mode='replay', rviz='true')), 2)

    def test_launch_invalid_arguments(self):
        with self.assertRaisesRegex(ValueError, 'mode must be'):
            self.launch_actions(mode='invalid')
        with self.assertRaisesRegex(ValueError, 'true or false'):
            self.launch_actions(mode='replay', rviz='invalid')
        with self.assertRaisesRegex(ValueError, 'does not exist'):
            self.launch_actions(config_file=str(ROOT / 'missing_test_config.yaml'))

    def test_rviz2_default_off_and_explicit_enable(self):
        for mode in ('mapping', 'localization', 'replay'):
            arguments = {'mode': mode}
            if mode == 'localization':
                arguments['map_path'] = str(ROOT / 'pcd_map/test.pcd')
            actions = self.launch_actions(inspect_nodes=True, **arguments)
            self.assertFalse(any(action['package'] == 'rviz2' for action in actions))
            actions = self.launch_actions(inspect_nodes=True, **arguments, rviz='true')
            self.assertEqual(sum(action['package'] == 'rviz2' for action in actions), 1)

    def test_localization_requires_explicit_map(self):
        for arguments in ({'mode': 'localization'},
                          {'mode': 'replay', 'config_file': 'mid360_localization.yaml'},
                          {'mode': 'mapping', 'config_file': 'mid360_localization.yaml'}):
            with self.assertRaisesRegex(ValueError, 'requires an explicit map_path'):
                self.launch_actions(**arguments)
        with self.assertRaisesRegex(ValueError, 'existing .pcd'):
            self.launch_actions(mode='localization', map_path=str(ROOT / 'pcd_map' / 'missing.pcd'))
        with tempfile.TemporaryDirectory(prefix='fastlio-launch-tests-') as directory:
            empty = Path(directory) / 'empty.pcd'
            empty.touch()
            with self.assertRaisesRegex(ValueError, 'empty'):
                self.launch_actions(mode='localization', map_path=str(empty))

    def test_bounded_relocalization_defaults_and_overrides(self):
        options = self.config('mid360_localization.yaml')['localization']['relocalization']
        self.assertTrue(options['enabled'])
        self.assertEqual(options['center'], [0.0, 0.0, 0.0])
        self.assertEqual(options['radius'], 3.0)
        actions = self.launch_actions(inspect_nodes=True, mode='localization',
                                      map_path=str(ROOT / 'pcd_map/test.pcd'),
                                      relocalize='true', search_radius='2.5')
        overrides = actions[1]['parameters'][1]
        self.assertTrue(overrides['localization.relocalization.enabled'])
        self.assertEqual(overrides['localization.relocalization.radius'], 2.5)
        actions = self.launch_actions(inspect_nodes=True, mode='localization',
                                      map_path=str(ROOT / 'pcd_map/test.pcd'), relocalize='false')
        self.assertFalse(actions[1]['parameters'][1]['localization.relocalization.enabled'])

    def test_invalid_relocalization_arguments(self):
        base = {'mode': 'localization', 'map_path': str(ROOT / 'pcd_map/test.pcd')}
        for radius in ('-1', '0', 'nan', 'inf', 'abc'):
            with self.assertRaisesRegex(ValueError, 'finite positive'):
                self.launch_actions(**base, search_radius=radius)
        with self.assertRaisesRegex(ValueError, 'true or false'):
            self.launch_actions(**base, relocalize='yes')
        with self.assertRaisesRegex(ValueError, 'localization-only'):
            self.launch_actions(mode='mapping', search_radius='3')

    def test_map_name_and_output_validation(self):
        self.assertEqual(len(self.launch_actions(mode='replay', map_name='场景A')), 1)
        with self.assertRaisesRegex(ValueError, 'map_name'):
            self.launch_actions(map_name='../lab')
        with self.assertRaisesRegex(ValueError, 'not an existing file'):
            self.launch_actions(map_output=str(ROOT / 'pcd_map' / 'test.pcd'))
        with self.assertRaisesRegex(ValueError, 'read-only'):
            self.launch_actions(mode='localization', map_path=str(ROOT / 'pcd_map' / 'test.pcd'), map_output='/tmp/not-written.pcd')

    def test_local_lidar_config_is_explicit_and_relative_paths_work(self):
        actions = self.launch_actions(inspect_nodes=True)
        self.assertEqual(actions[0]['parameters'][0]['user_config_path'], str(DRIVER / 'config' / 'MID360.json'))
        with tempfile.TemporaryDirectory(prefix='fastlio-local-launch-tests-') as directory:
            config = Path(directory) / 'config/local/MID360.local.json'
            config.parent.mkdir(parents=True)
            config.write_text((DRIVER / 'config/MID360.json').read_text())
            with patch('os.getcwd', return_value=directory):
                actions = self.launch_actions(inspect_nodes=True, lidar_config='config/local/MID360.local.json')
            self.assertEqual(actions[0]['parameters'][0]['user_config_path'], str(config))
            # Existing local config never changes the factory default selection.
            actions = self.launch_actions(inspect_nodes=True)
            self.assertEqual(actions[0]['parameters'][0]['user_config_path'], str(DRIVER / 'config/MID360.json'))

    def test_relative_local_yaml_and_map_argument_still_required(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-local-yaml-tests-') as directory:
            config = Path(directory) / 'config/local/scene.local.yaml'
            config.parent.mkdir(parents=True)
            config.write_text((FAST / 'config/mid360_localization.yaml').read_text())
            with patch('os.getcwd', return_value=directory):
                with self.assertRaisesRegex(ValueError, 'requires an explicit map_path'):
                    self.launch_actions(mode='localization', config_file='config/local/scene.local.yaml')
                actions = self.launch_actions(inspect_nodes=True, mode='localization',
                                              config_file='config/local/scene.local.yaml',
                                              map_path=str(ROOT / 'pcd_map/test.pcd'))
            self.assertEqual(actions[1]['parameters'][0], str(config))

    def test_invalid_lidar_json_is_rejected_before_nodes_start(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-bad-json-tests-') as directory:
            config = Path(directory) / 'bad.local.json'
            for content in ('', '{broken', '[]'):
                config.write_text(content)
                with self.assertRaisesRegex(ValueError, 'readable JSON object'):
                    self.launch_actions(lidar_config=str(config))
            with self.assertRaisesRegex(ValueError, 'Livox config does not exist'):
                self.launch_actions(lidar_config=str(Path(directory) / 'missing.local.json'))


if __name__ == '__main__':
    unittest.main()
