#!/usr/bin/env python3
"""Git metadata exists only in a temporary directory, never in the project."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'src/livox_ros_driver2/config/MID360.json'
HELPER = ROOT / 'scripts/init_local_config.py'
spec = importlib.util.spec_from_file_location('local_config_test_helper', HELPER)
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class GitRulesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='fastlio-git-rule-tests-')
        metadata = Path(cls.directory.name) / 'metadata.git'
        cls.env = {key: value for key, value in os.environ.items() if not key.startswith('GIT_')}
        cls.env.update(GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL=os.devnull)
        subprocess.run(['git', 'init', '--bare', '--quiet', str(metadata)],
                       env=cls.env, check=True, capture_output=True)
        cls.command = ['git', '--git-dir=' + str(metadata), '--work-tree=' + str(ROOT),
                       '-c', 'core.excludesFile=' + os.devnull]

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def ignored(self, path):
        result = subprocess.run(self.command + ['check-ignore', '--no-index', '--quiet', path],
                                cwd=ROOT, env=self.env, capture_output=True, text=True)
        self.assertIn(result.returncode, (0, 1), result.stderr)
        return result.returncode == 0

    def test_required_sources_and_sdk_bytes_are_not_ignored(self):
        paths = (
            '.gitignore', '.gitattributes', 'README.md', 'colcon.meta',
            'src/livox_ros_driver2/package.xml', 'src/FAST_LIO/package.xml',
            'src/livox_ros_driver2/config/MID360.json', 'src/FAST_LIO/config/mid360.yaml',
            'src/livox_ros_driver2/LICENSE.txt', 'src/FAST_LIO/LICENSE',
            'src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp',
            'src/FAST_LIO/include/IKFoM_toolkit/esekfom/esekfom.hpp',
            'livox-sdk-arm/lib/liblivox_lidar_sdk_shared.so',
            'livox-sdk-arm/lib/liblivox_lidar_sdk_static.a',
            'livox-sdk-x86/lib/liblivox_lidar_sdk_shared.so',
            'livox-sdk-x86/lib/liblivox_lidar_sdk_static.a',
            'src/FAST_LIO/doc/Fast_LIO_2.pdf', 'src/FAST_LIO/Log/guide.md',
            'pcd_map/test.pcd', 'pcd_map/README.md', 'pcd_map/COLCON_IGNORE',
            'pcd_map/maps.yaml', 'pcd_map/maps.sha256', 'config/README.md',
            'config/COLCON_IGNORE', 'tests/MID360_loopback.json', '.env.example',
        )
        for path in paths:
            with self.subTest(path=path):
                self.assertFalse(self.ignored(path), path)

    def test_outputs_and_local_machine_settings_are_ignored(self):
        paths = (
            'build/a.o', 'install/lib/a.so', 'log/smoke/a.log', 'bags/session/a.db3',
            'Record_Path/episode.csv', 'maps/test.pcd', 'reference/README.md',
            'pcd_map/lab.pcd', 'pcd_map/lab/map.pcd', 'pcd_map/map.tmpABC',
            'config/local/device.json', 'config/local/nested/device.yaml',
            'src/livox_ros_driver2/config/MID360.jetson.local.json',
            'src/FAST_LIO/config/mid360.local.yaml', 'dds_config/cyclonedds.local.xml',
            'src/FAST_LIO/.gitmodules', 'src/FAST_LIO/Log/mat_pre.txt',
            'src/FAST_LIO/PCD/scans.pcd', 'src/FAST_LIO/PCD/1',
            '.vscode/settings.json', 'src/FAST_LIO/.vscode/settings.json',
            'tests/__pycache__/test.pyc', '.pytest_cache/v/cache',
            'release.zip', 'release.zip.sha256', '.env', '.env.local', 'secrets/key',
        )
        for path in paths:
            with self.subTest(path=path):
                self.assertTrue(self.ignored(path), path)

    def test_source_lf_and_binary_attributes(self):
        for path in ('scripts/run.sh', 'scripts/setenv.zsh', 'scripts/init_local_config.py',
                     'src/FAST_LIO/CMakeLists.txt', 'src/FAST_LIO/src/laserMapping.cpp',
                     'src/livox_ros_driver2/config/MID360.json', '.gitignore'):
            result = subprocess.run(self.command + ['check-attr', 'text', 'eol', '--', path],
                                    cwd=ROOT, env=self.env, check=True, capture_output=True, text=True)
            self.assertIn(': text: set', result.stdout)
            self.assertIn(': eol: lf', result.stdout)
        for path in ('livox-sdk-arm/lib/liblivox_lidar_sdk_shared.so',
                     'livox-sdk-x86/lib/liblivox_lidar_sdk_static.a', 'pcd_map/test.pcd',
                     'src/FAST_LIO/doc/Fast_LIO_2.pdf'):
            result = subprocess.run(self.command + ['check-attr', 'text', 'diff', 'merge', 'filter', '--', path],
                                    cwd=ROOT, env=self.env, check=True, capture_output=True, text=True)
            for attribute in ('text', 'diff', 'merge'):
                self.assertIn(f': {attribute}: unset', result.stdout)
            self.assertIn(': filter: unspecified', result.stdout)


class LocalConfigTests(unittest.TestCase):
    def template(self):
        return json.loads(DEFAULT.read_text())

    def test_copy_does_not_mutate_template(self):
        original = self.template()
        copied = helper.prepare_configuration(original)
        self.assertEqual(copied, original)
        self.assertIsNot(copied, original)

    def test_ips_are_linked_and_other_settings_preserved(self):
        original = self.template()
        snapshot = copy.deepcopy(original)
        config = helper.prepare_configuration(original, '10.23.0.18', '10.23.0.114')
        host = config['MID360']['host_net_info'][0]
        self.assertEqual(host['host_ip'], '10.23.0.18')
        self.assertEqual(host['lidar_ip'], ['10.23.0.114'])
        self.assertEqual(config['lidar_configs'][0]['ip'], '10.23.0.114')
        self.assertEqual(config['MID360']['lidar_net_info'], original['MID360']['lidar_net_info'])
        self.assertEqual(config['lidar_configs'][0]['extrinsic_parameter'], original['lidar_configs'][0]['extrinsic_parameter'])
        for key, value in original['MID360']['host_net_info'][0].items():
            if key.endswith('_port'):
                self.assertEqual(host[key], value)
        self.assertEqual(original, snapshot)

    def test_object_style_host_ip_fields(self):
        template = self.template()
        template['MID360']['host_net_info'] = {key: '10.0.0.18' for key in helper.HOST_KEYS}
        config = helper.prepare_configuration(template, '10.23.0.18', '10.23.0.114')
        self.assertEqual(set(config['MID360']['host_net_info'].values()), {'10.23.0.18'})

    def test_rejects_invalid_ip_schema_and_multi_lidar_override(self):
        for address in ('not-an-ip', '::1', '300.2.3.4', '0.0.0.0', '224.0.0.1', '255.255.255.255'):
            with self.subTest(address=address), self.assertRaises(ValueError):
                helper.prepare_configuration(self.template(), address)
        with self.assertRaisesRegex(ValueError, 'must not equal'):
            helper.prepare_configuration(self.template(), '192.168.123.114')
        with self.assertRaises(ValueError):
            helper.prepare_configuration({})
        template = self.template()
        template['lidar_configs'].append(copy.deepcopy(template['lidar_configs'][0]))
        with self.assertRaisesRegex(ValueError, 'one LiDAR only'):
            helper.prepare_configuration(template, lidar_ip='10.0.0.114')

    def test_exclusive_file_creation_and_factory_unchanged(self):
        original = DEFAULT.read_bytes()
        with tempfile.TemporaryDirectory(prefix='fastlio-local-config-tests-') as directory:
            output = Path(directory) / 'MID360.local.json'
            helper.create_local_config(DEFAULT, output, '10.23.0.18', '10.23.0.114')
            contents = output.read_bytes()
            self.assertTrue(contents.endswith(b'\n'))
            self.assertEqual(json.loads(contents)['lidar_configs'][0]['ip'], '10.23.0.114')
            with self.assertRaises(FileExistsError):
                helper.create_local_config(DEFAULT, output)
            self.assertEqual(output.read_bytes(), contents)
            with self.assertRaisesRegex(ValueError, '.local.json'):
                helper.create_local_config(DEFAULT, Path(directory) / 'tracked.json')
            with self.assertRaises(ValueError):
                helper.create_local_config(DEFAULT, Path(directory) / 'bad.local.json', 'wrong')
            self.assertFalse((Path(directory) / 'bad.local.json').exists())
        self.assertEqual(DEFAULT.read_bytes(), original)

    def test_cli_has_clear_errors_without_writes(self):
        for arguments, expected in ((['--name', '../escape'], 'path separators'),
                                    (['--name', 'pc', '--output', 'pc.local.json'], 'either'),
                                    (['--host-ip', 'bad'], 'IPv4')):
            result = subprocess.run([sys.executable, str(HELPER), *arguments],
                                    text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            self.assertIn(expected, result.stderr)


if __name__ == '__main__':
    unittest.main()
