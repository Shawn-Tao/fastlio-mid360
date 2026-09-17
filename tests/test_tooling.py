#!/usr/bin/env python3
"""No daemon, sudo, apt, NIC writes or ROS processes needed for these tests."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


class ToolingTests(unittest.TestCase):
    def environment(self, **changes):
        env = dict(os.environ)
        for name in ('FASTLIO_CONTAINER', 'FASTLIO_CONTAINER_WORKSPACE', 'FASTLIO_NATIVE',
                     'FASTLIO_DDS', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI',
                     'ROS_LOCALHOST_ONLY', 'DISPLAY', 'WAYLAND_DISPLAY', 'QT_QPA_PLATFORM'):
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
            for path in (ROOT / directory).rglob('*'):
                if path.suffix in ('.sh', '.bash'):
                    result = subprocess.run(['bash', '-n', str(path)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, f'{path}: {result.stderr}')
                elif path.suffix == '.zsh' and shutil.which('zsh'):
                    result = subprocess.run(['zsh', '-n', str(path)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, f'{path}: {result.stderr}')

    def test_auto_container_and_argument_boundaries(self):
        argument = 'map_path:=pcd_map/场景 A.pcd'
        result = self.run_script('scripts/lib/in_container.sh', 'run.sh', 'localization', argument,
                                 ROS_DOMAIN_ID='23', FASTLIO_DDS='cyclone')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(args[args.index('-w') + 1], f'/portable space/{ROOT.name}')
        self.assertIn('ROS_DOMAIN_ID=23', args)
        self.assertIn('FASTLIO_DDS=cyclone', args)
        self.assertEqual(args[-1], argument)
        self.assertIn('test-container', args)

    def test_longest_mount(self):
        result = self.run_script('scripts/lib/in_container.sh', 'build.sh', FAKE_DOCKER_MOUNT='nested')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(args[args.index('-w') + 1], f'/nested space/{ROOT.name}')

    def test_run_help_does_not_require_ros_docker_or_pcd(self):
        for script in ('scripts/run.sh',):
            result = self.run_script(script, '--help', FASTLIO_NATIVE='0', FAKE_DOCKER_IDS='')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--rviz', result.stdout)
            self.assertIn('--no-rviz', result.stdout)
            self.assertIn('no PCD required', result.stdout)
            self.assertNotIn('[fastlio] container=', result.stdout)

    def test_run_rviz_flags_and_parameter_boundaries(self):
        argument = 'map_path:=pcd_map/场景 A.pcd'
        cases = [
            ('scripts/run.sh', ('localization', argument), 'localization', []),
            ('scripts/run.sh', ('localization', '--rviz', argument), 'localization', ['rviz:=true']),
            ('scripts/run.sh', ('localization', argument, '--no-rviz'), 'localization', ['rviz:=false']),
            ('scripts/run.sh', ('mapping', '--rviz'), 'mapping', ['rviz:=true']),
            ('scripts/run.sh', ('--rviz',), 'mapping', ['rviz:=true']),
            ('scripts/run.sh', ('replay', '--rviz'), 'replay', ['rviz:=true']),
            ('scripts/run.sh', ('localization', argument, 'rviz:=true'), 'localization', ['rviz:=true']),
            ('scripts/run.sh', ('localization', argument, '--rviz', '--no-rviz'), 'localization', ['rviz:=false']),
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
            result = self.run_script('scripts/lib/in_container.sh', 'build.sh', **changes)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        result = self.run_script('scripts/lib/in_container.sh', 'build.sh',
                                 FASTLIO_CONTAINER='selected', FASTLIO_CONTAINER_WORKSPACE='/custom path')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        self.assertIn('selected', args)
        self.assertEqual(args[args.index('-w') + 1], '/custom path')

    def test_dds_fragment_in_bash_and_zsh(self):
        shells = ['bash'] + (['zsh'] if shutil.which('zsh') else [])
        command = ('fastlio_workspace_dir="$1"; source "$1/scripts/lib/dds_env.sh" || exit $?; '
                   'printf "%s|%s|%s" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID" "${CYCLONEDDS_URI:-}"')
        for shell in shells:
            for selection, rmw in (('fastdds', 'rmw_fastrtps_cpp'), ('cyclone', 'rmw_cyclonedds_cpp')):
                result = subprocess.run([shell, '-c', command, 'dds-test', str(ROOT)],
                                        env=self.environment(FASTLIO_DDS=selection), text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                expected_uri = f'file://{ROOT}/dds_config/cyclonedds.xml' if selection == 'cyclone' else ''
                self.assertEqual(result.stdout, f'{rmw}|18|{expected_uri}')

    def environment_fixture(self, directory, overlay=True, include_ros=True, ros_failure=False):
        # Explicit fake setup files: tests sourcing/DDS, not a ROS installation.
        workspace = Path(directory) / 'portable workspace'
        ros_root = Path(directory) / 'fake_humble'
        (workspace / 'scripts/lib').mkdir(parents=True)
        for name in ('env.sh', 'setenv.bash', 'setenv.zsh'):
            content = (ROOT / 'scripts' / name).read_text()
            (workspace / 'scripts' / name).write_text(content.replace('/opt/ros/humble', str(ros_root)))
        shutil.copy(ROOT / 'scripts/lib/dds_env.sh', workspace / 'scripts/lib/dds_env.sh')
        if include_ros:
            ros_root.mkdir()
            for shell in ('bash', 'zsh'):
                (ros_root / ('setup.' + shell)).write_text(
                    'return 7\n' if ros_failure else
                    f'export ROS_DISTRO=humble FAKE_ENV_ROS_SHELL={shell}\n')
        if overlay:
            (workspace / 'install').mkdir()
            for shell in ('bash', 'zsh'):
                (workspace / 'install' / ('local_setup.' + shell)).write_text(
                    f'export FAKE_ENV_OVERLAY_SHELL={shell}\n')
        return workspace

    def source_environment(self, shell, workspace, entry='env.sh', **changes):
        command = (
            'fastlio_test_pwd="$PWD"; fastlio_test_opts="$-"; fastlio_test_arg="$1"; '
            'if source "$1/scripts/$2"; then fastlio_test_code=0; else fastlio_test_code=$?; fi; '
            '[ "$PWD" = "$fastlio_test_pwd" ] && [ "$-" = "$fastlio_test_opts" ] '
            '&& [ "$1" = "$fastlio_test_arg" ] || exit 88; '
            'if typeset -f _fastlio_env_load >/dev/null; then exit 89; fi; '
            'printf "__ENV__%s|%s|%s|%s|%s|%s\\n" '
            '"${FAKE_ENV_ROS_SHELL:-}" "${FAKE_ENV_OVERLAY_SHELL:-}" '
            '"${ROS_DOMAIN_ID:-}" "${RMW_IMPLEMENTATION:-}" "${CYCLONEDDS_URI:-}" '
            '"${ROS_LOCALHOST_ONLY:-}"; exit "$fastlio_test_code"')
        env = self.environment(**changes)
        for name in ('FAKE_ENV_ROS_SHELL', 'FAKE_ENV_OVERLAY_SHELL'):
            env.pop(name, None)
        return subprocess.run([shell, '-c', command, 'source-env-test', str(workspace), entry],
                              env=env, text=True, capture_output=True, timeout=15)

    def environment_shells(self):
        return ['bash'] + (['zsh'] if shutil.which('zsh') else [])

    def test_environment_entry_rejects_child_shell_execution(self):
        for shell in self.environment_shells():
            result = subprocess.run([shell, str(ROOT / 'scripts/env.sh')],
                                    env=self.environment(), text=True, capture_output=True, timeout=15)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            self.assertIn('source scripts/env.sh', result.stderr)
            self.assertNotIn('Environment loaded', result.stdout)

    def test_environment_entry_loads_matching_shell_and_defaults(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-env-stubs-') as directory:
            workspace = self.environment_fixture(directory)
            for shell in self.environment_shells():
                result = self.source_environment(shell, workspace, CYCLONEDDS_URI='file:///stale.xml')
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(f'__ENV__{shell}|{shell}|18|rmw_fastrtps_cpp||', result.stdout)
                self.assertIn(f'workspace={workspace}', result.stdout)
                self.assertIn('ros2 topic list', result.stdout)

    def test_environment_entry_preserves_explicit_dds_overrides(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-env-stubs-') as directory:
            workspace = self.environment_fixture(directory)
            for shell in self.environment_shells():
                for changes in ({'FASTLIO_DDS': 'cyclone'}, {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}):
                    result = self.source_environment(shell, workspace, ROS_DOMAIN_ID='23',
                                                     CYCLONEDDS_URI='file:///custom.xml', **changes)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn(f'__ENV__{shell}|{shell}|23|rmw_cyclonedds_cpp|file:///custom.xml|',
                                  result.stdout)

    def test_environment_entry_supports_ros_only_without_overlay(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-env-stubs-') as directory:
            workspace = self.environment_fixture(directory, overlay=False)
            for shell in self.environment_shells():
                result = self.source_environment(shell, workspace)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(f'__ENV__{shell}||18|rmw_fastrtps_cpp||', result.stdout)
                self.assertIn('without workspace overlay', result.stderr)

    def test_environment_entry_missing_ros_gives_docker_guidance(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-env-stubs-') as directory:
            workspace = self.environment_fixture(directory, include_ros=False)
            for shell in self.environment_shells():
                result = self.source_environment(shell, workspace)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn('bash docker/run.sh', result.stderr)
                self.assertIn('inside the container', result.stderr)
                self.assertNotIn('Environment loaded', result.stdout)

    def test_environment_entry_propagates_setup_failure(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-env-stubs-') as directory:
            workspace = self.environment_fixture(directory, ros_failure=True)
            for shell in self.environment_shells():
                result = self.source_environment(shell, workspace)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn('Failed to load', result.stderr)
                self.assertNotIn('Environment loaded', result.stdout)

    def test_environment_entry_rejects_invalid_dds(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-env-stubs-') as directory:
            workspace = self.environment_fixture(directory)
            for shell in self.environment_shells():
                result = self.source_environment(shell, workspace, FASTLIO_DDS='unsupported')
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn('FASTLIO_DDS must be', result.stderr)
                self.assertNotIn('Environment loaded', result.stdout)

    def test_environment_entry_warns_without_overwriting_localhost(self):
        with tempfile.TemporaryDirectory(prefix='fastlio-env-stubs-') as directory:
            workspace = self.environment_fixture(directory)
            for shell in self.environment_shells():
                for value in ('0', '1'):
                    result = self.source_environment(shell, workspace, ROS_LOCALHOST_ONLY=value)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn(f'__ENV__{shell}|{shell}|18|rmw_fastrtps_cpp||{value}', result.stdout)
                    self.assertEqual('prevents NX/AGX' in result.stderr, value == '1')

    def test_legacy_environment_entries_reject_wrong_shell(self):
        for shell, entry, hint in [('bash', 'setenv.zsh', 'setenv.bash')] + (
                [('zsh', 'setenv.bash', 'setenv.zsh')] if shutil.which('zsh') else []):
            result = self.source_environment(shell, ROOT, entry=entry)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            self.assertIn(hint, result.stderr)
            self.assertNotIn('bad substitution', result.stderr)

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

    def test_scripts_layout_and_active_references(self):
        expected = {'build.sh', 'test.sh', 'run.sh', 'rviz.sh', 'env.sh', 'setenv.bash', 'setenv.zsh',
                    'init_local_config.sh', 'check_network.sh', 'install_deps.sh', 'package.sh'}
        actual = {p.name for p in (ROOT / 'scripts').iterdir()
                  if p.is_file() and p.suffix in ('.sh', '.bash', '.zsh', '.py') and '.local.' not in p.name}
        self.assertEqual(actual, expected)
        for name in ('common.sh', 'dds_env.sh', 'in_container.sh', 'init_local_config.py', 'check_network.py'):
            self.assertTrue((ROOT / 'scripts/lib' / name).is_file(), name)
        for name in expected:
            text = (ROOT / 'scripts' / name).read_text()
            self.assertNotIn('source "$workspace_dir/scripts/backup/', text)
            self.assertNotIn('exec bash "$workspace_dir/scripts/backup/', text)

    def test_archived_aliases_still_delegate(self):
        if not (ROOT / 'scripts/backup').is_dir():
            self.skipTest('Historical aliases intentionally omitted from deployment ZIP')
        argument = 'map_path:=pcd_map/场景 A.pcd'
        for name, mode in (('run_mapping_local.sh', 'mapping'), ('run_localization_local.sh', 'localization')):
            help_result = self.run_script('scripts/backup/' + name, '--help', FASTLIO_NATIVE='0', FAKE_DOCKER_IDS='')
            self.assertEqual(help_result.returncode, 0, help_result.stderr)
            result = self.run_script('scripts/backup/' + name, '--rviz', argument, FASTLIO_NATIVE='0')
            self.assertEqual(result.returncode, 0, result.stderr)
            args = json.loads(result.stdout.splitlines()[-1])
            self.assertEqual(args[-3:], [mode, argument, 'rviz:=true'])

    def test_viewer_help_and_invalid_options_without_ros(self):
        result = self.run_script('scripts/rviz.sh', '--help', FASTLIO_NATIVE='0', FAKE_DOCKER_IDS='')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('no workspace build or PCD', result.stdout)
        self.assertNotIn('[fastlio] container=', result.stdout)
        for args in (('replay',), ('localization', '--config'), ('mapping', '--config', '--help'), ('mapping', '--unknown')):
            result = self.run_script('scripts/rviz.sh', *args, FASTLIO_NATIVE='0', FAKE_DOCKER_IDS='')
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            self.assertNotIn('[fastlio] container=', result.stdout)

    def test_viewer_container_dispatch_and_argument_boundaries(self):
        custom = 'src/FAST_LIO/rviz/场景 A.rviz'
        result = self.run_script('scripts/rviz.sh', 'localization', '--config', custom, '--',
                                 '--ros-args', '-r', '__node:=remote_viewer', FASTLIO_NATIVE='0',
                                 ROS_DOMAIN_ID='23', ROS_LOCALHOST_ONLY='0', DISPLAY=':99')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout.splitlines()[-1])
        marker = next(i for i, value in enumerate(args) if value.endswith('/scripts/rviz.sh'))
        self.assertEqual(args[marker + 1:], ['localization', '--config', custom, '--', '--ros-args', '-r', '__node:=remote_viewer'])
        for value in ('ROS_DOMAIN_ID=23', 'ROS_LOCALHOST_ONLY=0', 'DISPLAY=:99'):
            self.assertIn(value, args)
        self.assertFalse(any(value.endswith('/scripts/run.sh') for value in args))

    def test_viewer_native_exec_only_with_fake_ros_and_rviz(self):
        # Explicit stubs: validates command construction and DDS, not real GUI/ROS.
        with tempfile.TemporaryDirectory(prefix='fastlio-viewer-stubs-') as directory:
            root = Path(directory)
            (root / 'scripts/lib').mkdir(parents=True)
            for path in ('scripts/rviz.sh', 'scripts/lib/common.sh', 'scripts/lib/dds_env.sh'):
                shutil.copy(ROOT / path, root / path)
            (root / 'scripts/setenv.bash').write_text('fastlio_workspace_dir="$workspace_dir"\nsource "$workspace_dir/scripts/lib/dds_env.sh"\n')
            configs = root / 'src/FAST_LIO/rviz'
            configs.mkdir(parents=True)
            for mode in ('mapping', 'localization'):
                shutil.copy(ROOT / 'src/FAST_LIO/rviz' / (mode + '.rviz'), configs)
            binary = root / 'bin/rviz2'
            binary.parent.mkdir()
            binary.write_text('#!/usr/bin/env python3\nimport json, os, sys\nprint(json.dumps({"args": sys.argv[1:], "domain": os.getenv("ROS_DOMAIN_ID"), "rmw": os.getenv("RMW_IMPLEMENTATION")}))\n')
            binary.chmod(0o755)
            def run(*args, **changes):
                env = self.environment(FASTLIO_NATIVE='1', DISPLAY=':99')
                env.update(PATH=str(binary.parent) + os.pathsep + env['PATH'], **changes)
                return subprocess.run(['bash', str(root / 'scripts/rviz.sh'), *args], env=env, text=True, capture_output=True, timeout=15)
            for mode in ('mapping', 'localization'):
                result = run(mode)
                self.assertEqual(result.returncode, 0, result.stderr)
                data = json.loads(result.stdout)
                self.assertEqual(data['args'], ['-d', str(configs / (mode + '.rviz'))])
                self.assertEqual(data['domain'], '18')
                self.assertEqual(data['rmw'], 'rmw_fastrtps_cpp')
            custom = configs / 'custom scene.rviz'
            custom.write_text('custom config stub')
            result = run('localization', '--config', str(custom.relative_to(root)), '--', '--ros-args', '-r', '__node:=viewer', ROS_DOMAIN_ID='23', FASTLIO_DDS='cyclone')
            self.assertEqual(result.returncode, 0, result.stderr)
            data = json.loads(result.stdout)
            self.assertEqual(data['args'], ['-d', str(custom.relative_to(root)), '--ros-args', '-r', '__node:=viewer'])
            self.assertEqual(data['domain'], '23')
            self.assertEqual(data['rmw'], 'rmw_cyclonedds_cpp')
            for args, changes, message in ((('--config', 'missing.rviz'), {}, 'configuration not found'),
                                            ((), {'DISPLAY': '', 'WAYLAND_DISPLAY': ''}, 'graphical desktop')):
                result = run(*args, **changes)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn(message, result.stderr)

    def test_viewer_presets_and_launch_selection(self):
        for mode, map_topic, durability in (('mapping', '/Laser_map', 'Volatile'), ('localization', '/reference_map', 'Transient Local')):
            config = yaml.safe_load((ROOT / 'src/FAST_LIO/rviz' / (mode + '.rviz')).read_text())['Visualization Manager']
            self.assertEqual(config['Global Options']['Fixed Frame'], 'camera_init')
            topics = {display['Topic']['Value']: display for display in config['Displays'] if 'Topic' in display}
            self.assertEqual(topics[map_topic]['Topic']['Durability Policy'], durability)
            self.assertEqual(topics[map_topic]['Decay Time'], 0)
            self.assertEqual(topics[map_topic]['Topic']['Depth'], 1)
            self.assertIn('/Odometry', topics)
            self.assertIn('/path', topics)
            if mode == 'localization':
                self.assertIn('/cloud_registered', topics)
                self.assertNotIn('/Laser_map', topics)


if __name__ == '__main__':
    unittest.main()
