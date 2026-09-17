#!/usr/bin/env python3
"""No daemon, sudo, apt, NIC writes or ROS processes needed for these tests."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class ToolingTests(unittest.TestCase):
    def environment(self, **changes):
        env = dict(os.environ)
        for name in ('FASTLIO_CONTAINER', 'FASTLIO_CONTAINER_WORKSPACE', 'FASTLIO_NATIVE',
                     'FASTLIO_DDS', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI'):
            env.pop(name, None)
        env.update(PATH=str(ROOT / 'tests' / 'fake_bin') + os.pathsep + env['PATH'],
                   FAKE_DOCKER_WORKSPACE=str(ROOT))
        env.update(changes)
        return env

    def run_script(self, name, *args, **env):
        return subprocess.run(['bash', str(ROOT / name), *args], env=self.environment(**env),
                              text=True, capture_output=True, timeout=15)

    def test_shell_syntax(self):
        for directory in ('scripts', 'docker'):
            for path in (ROOT / directory).iterdir():
                if path.suffix in ('.sh', '.bash'):
                    result = subprocess.run(['bash', '-n', str(path)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, f'{path}: {result.stderr}')
                elif path.suffix == '.zsh' and shutil.which('zsh'):
                    result = subprocess.run(['zsh', '-n', str(path)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, f'{path}: {result.stderr}')

    def test_auto_container_and_argument_boundaries(self):
        argument = 'map_path:=pcd_map/场景 A.pcd'
        result = self.run_script('scripts/in_container.sh', 'run.sh', 'localization', argument,
                                 ROS_DOMAIN_ID='23', FASTLIO_DDS='cyclone')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(args[args.index('-w') + 1], f'/portable space/{ROOT.name}')
        self.assertIn('ROS_DOMAIN_ID=23', args)
        self.assertIn('FASTLIO_DDS=cyclone', args)
        self.assertEqual(args[-1], argument)
        self.assertIn('test-container', args)

    def test_longest_mount(self):
        result = self.run_script('scripts/in_container.sh', 'build.sh', FAKE_DOCKER_MOUNT='nested')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(args[args.index('-w') + 1], f'/nested space/{ROOT.name}')

    def test_run_help_does_not_require_ros_docker_or_pcd(self):
        for script in ('scripts/run.sh', 'scripts/run_mapping_local.sh',
                       'scripts/run_localization_local.sh'):
            result = self.run_script(script, '--help', FASTLIO_NATIVE='0', FAKE_DOCKER_IDS='')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--rviz', result.stdout)
            self.assertIn('--no-rviz', result.stdout)
            self.assertIn('no PCD required', result.stdout)
            self.assertNotIn('[fastlio] container=', result.stdout)

    def test_run_rviz_flags_and_parameter_boundaries(self):
        argument = 'map_path:=pcd_map/场景 A.pcd'
        cases = [
            ('scripts/run_localization_local.sh', (argument,), 'localization', []),
            ('scripts/run_localization_local.sh', ('--rviz', argument), 'localization', ['rviz:=true']),
            ('scripts/run.sh', ('localization', argument, '--no-rviz'), 'localization', ['rviz:=false']),
            ('scripts/run_mapping_local.sh', ('--rviz',), 'mapping', ['rviz:=true']),
            ('scripts/run.sh', ('--rviz',), 'mapping', ['rviz:=true']),
            ('scripts/run.sh', ('replay', '--rviz'), 'replay', ['rviz:=true']),
            ('scripts/run_localization_local.sh', (argument, 'rviz:=true'), 'localization', ['rviz:=true']),
            ('scripts/run_localization_local.sh', (argument, '--rviz', '--no-rviz'), 'localization', ['rviz:=false']),
        ]
        for script, parameters, mode, expected in cases:
            result = self.run_script(script, *parameters, FASTLIO_NATIVE='0')
            self.assertEqual(result.returncode, 0, result.stderr)
            args = json.loads(result.stdout.splitlines()[-1])
            marker = next(i for i, value in enumerate(args) if value.endswith('/scripts/run.sh'))
            self.assertEqual(args[marker + 1], mode)
            tail = args[marker + 2:]
            self.assertEqual([value for value in tail if value.startswith('rviz:=')], expected)
            if argument in parameters:
                self.assertIn(argument, tail)
            self.assertNotIn('--rviz', tail)
            self.assertNotIn('--no-rviz', tail)

    def test_ambiguous_missing_and_explicit_container(self):
        for changes in ({'FAKE_DOCKER_IDS': ''},
                        {'FAKE_DOCKER_IDS': 'first\nsecond'},
                        {'FAKE_DOCKER_MOUNT': 'none'}):
            result = self.run_script('scripts/in_container.sh', 'build.sh', **changes)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        result = self.run_script('scripts/in_container.sh', 'build.sh',
                                 FASTLIO_CONTAINER='selected', FASTLIO_CONTAINER_WORKSPACE='/custom path')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        self.assertIn('selected', args)
        self.assertEqual(args[args.index('-w') + 1], '/custom path')

    def test_dds_fragment_in_bash_and_zsh(self):
        shells = ['bash'] + (['zsh'] if shutil.which('zsh') else [])
        command = ('fastlio_workspace_dir="$1"; source "$1/scripts/dds_env.sh" || exit $?; '
                   'printf "%s|%s|%s" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID" "${CYCLONEDDS_URI:-}"')
        for shell in shells:
            for selection, rmw in (('fastdds', 'rmw_fastrtps_cpp'), ('cyclone', 'rmw_cyclonedds_cpp')):
                result = subprocess.run([shell, '-c', command, 'dds-test', str(ROOT)],
                                        env=self.environment(FASTLIO_DDS=selection), text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                expected_uri = f'file://{ROOT}/dds_config/cyclonedds.xml' if selection == 'cyclone' else ''
                self.assertEqual(result.stdout, f'{rmw}|18|{expected_uri}')

    def test_dds_xml_has_no_fixed_domain_nic_or_peer(self):
        ns = {'c': 'https://cdds.io/config'}
        for path in (ROOT / 'dds_config').glob('*.xml'):
            xml = ET.parse(path)
            self.assertEqual(xml.find('c:Domain', ns).attrib, {'Id': 'any'})
            self.assertEqual(xml.find('.//c:NetworkInterface', ns).attrib, {'autodetermine': 'true'})
            self.assertIsNone(xml.find('.//c:Peer', ns))

    def test_docker_dependencies_and_aliases(self):
        dockerfile = (ROOT / 'docker' / 'Dockerfile').read_text()
        self.assertIn('FROM ros:humble-ros-base-jammy', dockerfile)
        for name in ('Dockerfile_pc', 'Dockerfile_jetson'):
            self.assertEqual((ROOT / 'docker' / name).read_text(), dockerfile)
        result = self.run_script('scripts/install_deps.sh', '--print')
        self.assertEqual(result.returncode, 0, result.stderr)
        for token in dockerfile.split():
            if token.startswith('ros-humble-') and token not in {'ros-humble-rviz2'}:
                self.assertIn(token, result.stdout)

    def test_docker_builder_and_no_container_replacement(self):
        result = self.run_script('docker/build.sh', '--no-cache', FASTLIO_INSTALL_RVIZ='1')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        self.assertIn('INSTALL_RVIZ=1', args)
        self.assertIn('--no-cache', args)
        self.assertEqual(args[-1], str(ROOT))
        result = self.run_script('docker/run.sh', '--detach', FAKE_DOCKER_EXISTING_SOURCE='/somewhere-else')
        self.assertEqual(result.returncode, 2)
        self.assertIn('nothing was replaced', result.stderr)

    def test_packaging_refuses_workspace_or_existing_target(self):
        result = self.run_script('scripts/package.sh', str(ROOT / 'recursive.zip'))
        self.assertEqual(result.returncode, 2)
        self.assertIn('outside the workspace', result.stderr)
        result = self.run_script('scripts/package.sh', str(ROOT / 'README.md'))
        self.assertEqual(result.returncode, 2)
        self.assertIn('NEW .zip path', result.stderr)


if __name__ == '__main__':
    unittest.main()
