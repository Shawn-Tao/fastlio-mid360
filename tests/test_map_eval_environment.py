"""Host-only local-environment wrapper and archive guards; no downloads."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

try:
    import tomllib
except ModuleNotFoundError:
    tomllib = None  # Humble uses Python 3.10; wrapper/archive tests still run.

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT/'tools/map_eval'


class MapEvalEnvironmentTests(unittest.TestCase):
    @unittest.skipIf(tomllib is None, 'TOML metadata check requires Python 3.11+')
    def test_manifest_is_cpu_only_and_colcon_ignored(self):
        project = tomllib.loads((TOOL/'pyproject.toml').read_text())
        self.assertEqual(project['project']['requires-python'], '>=3.12,<3.13')
        self.assertIn('open3d-cpu==0.19.0', project['project']['dependencies'])
        self.assertFalse(project['tool']['uv']['package'])
        self.assertTrue((TOOL/'COLCON_IGNORE').is_file())
        self.assertEqual((TOOL/'.python-version').read_text().strip(), '3.12')

    def test_help_needs_no_environment(self):
        result = subprocess.run(['bash', str(TOOL/'run.sh'), '--help'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('check_env.py', result.stdout)

    def test_wrapper_bounds_paths_and_does_not_inherit_ros_python(self):
        with tempfile.TemporaryDirectory(prefix='fastlio map env tests ') as directory:
            workspace = Path(directory)
            tool = workspace/'tools/map_eval'; tool.mkdir(parents=True)
            script = tool/'run.sh'; script.write_bytes((TOOL/'run.sh').read_bytes())
            (tool/'uv.lock').write_text('# fake lock for wrapper-only test\n')
            executable = workspace/'.local_tools/uv/uv'; executable.parent.mkdir(parents=True)
            executable.write_text(f'#!{sys.executable}\nimport json,os,sys\n'
                'keys=("UV_CACHE_DIR","UV_PYTHON_INSTALL_DIR","UV_PROJECT_ENVIRONMENT",'
                '"UV_PYTHON_INSTALL_BIN","PYTHONNOUSERSITE","MPLCONFIGDIR",'
                '"PYTHONHOME","PYTHONPATH","VIRTUAL_ENV","ROS_DOMAIN_ID")\n'
                'print(json.dumps({"argv":sys.argv[1:],"env":{key:os.environ.get(key) for key in keys}}))\n')
            executable.chmod(0o755)
            env = dict(os.environ, PYTHONHOME='/bad/ROS', PYTHONPATH='/bad/ROS/packages',
                       VIRTUAL_ENV='/bad/ROS/environment', UV_PROJECT_ENVIRONMENT='/bad/system',
                       UV_CACHE_DIR='/bad/global-cache', ROS_DOMAIN_ID='42')
            result = subprocess.run(['bash', str(script), 'python', 'check_env.py', '--maps', 'map file.pcd'],
                                    env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            captured = json.loads(result.stdout)
            self.assertEqual(captured['argv'], ['--directory', str(tool), 'run', '--managed-python',
                                               '--locked', 'python', 'check_env.py', '--maps', 'map file.pcd'])
            values = captured['env']
            self.assertEqual(values['UV_CACHE_DIR'], str(workspace/'.local_tools/cache'))
            self.assertEqual(values['UV_PYTHON_INSTALL_DIR'], str(workspace/'.local_tools/python'))
            self.assertEqual(values['UV_PROJECT_ENVIRONMENT'], str(tool/'.venv'))
            self.assertEqual(values['MPLCONFIGDIR'], str(workspace/'.local_tools/matplotlib'))
            self.assertEqual(values['UV_PYTHON_INSTALL_BIN'], '0')
            self.assertEqual(values['PYTHONNOUSERSITE'], '1')
            self.assertEqual(values['ROS_DOMAIN_ID'], '42')
            for key in ('PYTHONHOME', 'PYTHONPATH', 'VIRTUAL_ENV'):
                self.assertIsNone(values[key])
            self.assertEqual(env['PYTHONPATH'], '/bad/ROS/packages')

    def test_archive_checker_rejects_python_environments_and_reports(self):
        spec = importlib.util.spec_from_file_location('map_env_archive_checker', ROOT/'tests/check_archive.py')
        checker = importlib.util.module_from_spec(spec); spec.loader.exec_module(checker)
        paths = ('.local_tools/python/bin/python3.12', 'tools/map_eval/.venv/bin/python',
                 'tools/map_eval/venv/bin/python', 'tools/map_eval/.uv-cache/wheel.whl',
                 'tools/map_eval/results/report.json')
        with tempfile.TemporaryDirectory(prefix='fastlio-env-archive-tests-') as directory:
            for index, path in enumerate(paths):
                archive = Path(directory)/f'bad-{index}.zip'
                with zipfile.ZipFile(archive, 'w') as output:
                    output.writestr(f'workspace/{path}', b'fake environment artifact')
                with self.subTest(path=path), self.assertRaisesRegex(ValueError, 'artifact|cache|reports'):
                    checker.check_archive(archive)


if __name__ == '__main__':
    unittest.main()
