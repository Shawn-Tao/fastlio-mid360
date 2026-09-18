#!/usr/bin/env python3
"""Host contracts and CSV recovery; not a ROS node execution test."""
import ast
import importlib.util
from pathlib import Path
import re
import tempfile
import unittest
import warnings
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/FAST_LIO/src/laserMapping.cpp'


class RuntimeContracts(unittest.TestCase):
    def test_gating_before_outputs_and_map_changes(self):
        source = SOURCE.read_text()
        timer = source.split('    void timer_callback()', 1)[1].split('    void map_publish_callback()', 1)[0]
        gate = timer.index('if(!trusted)')
        self.assertLess(gate, timer.index('publish_odometry('))
        self.assertLess(gate, timer.index('lasermap_fov_segment();'))
        self.assertLess(gate, timer.index('accumulate_map_frame();'))
        self.assertLess(gate, timer.index('append_record_sample('))
        self.assertNotIn('*pcl_wait_pub +=', source)
        self.assertIn('create_wall_timer', source)
        self.assertIn('get_subscription_count()>0', source)
        self.assertIn('save_worker_=std::async', source)
        odometry = source.split('void publish_odometry(', 1)[1].split('void publish_path(', 1)[0]
        self.assertLess(odometry.index('pose.covariance['), odometry.index('pubOdomAftMapped->publish('))
        self.assertIn('filter_covariance.topLeftCorner<6,6>()', odometry)

    def test_defaults_and_physics_groups(self):
        configs = [yaml.safe_load((ROOT / 'src/FAST_LIO/config' / name).read_text())['/**']['ros__parameters']
                   for name in ('mid360.yaml', 'mid360_localization.yaml')]
        a, b = configs
        for key in ('mapping', 'preprocess', 'common', 'point_filter_num', 'filter_size_surf'):
            self.assertEqual(a[key], b[key], key)
        self.assertGreater(a['cube_side_length'], 3 * a['mapping']['det_range'])
        self.assertFalse(a['publish']['map_en'])
        self.assertFalse(a['record']['republish_saved'])
        self.assertGreater(a['pcd_save']['max_points'], 0)
        self.assertEqual(a['tracking'], b['tracking'])

    def test_static_filter_is_archive_only_and_audited(self):
        source = SOURCE.read_text()
        archive = source.split('    void accumulate_map_frame()', 1)[1].split('    std::string parameter_json', 1)[0]
        for expression in ('archive_->insert_frame', 'pos_lid.x()', 'last_position_std_', 'state_point.vel.norm()', 'angular_speed'):
            self.assertIn(expression, archive)
        self.assertIn('std::min(source->size(),archive_->frame_limit())', archive)
        self.assertIn('source->size()-count', archive)
        self.assertIn('Measures.imu', archive)
        self.assertIn('clear_motion_limits_.reason(', archive)
        self.assertIn('archive_->clearing_enabled()', archive)
        self.assertIn(r'\"archive_clear_gate\"', source)
        live = source.split('void map_incremental()', 1)[1].split('void publish_frame_world', 1)[0]
        self.assertNotIn('archive_->', live)
        self.assertNotIn('static_map.', live)
        self.assertGreaterEqual(source.count('archive_->status_json()'), 2)
        config = yaml.safe_load((ROOT / 'src/FAST_LIO/config/mid360.yaml').read_text())['/**']['ros__parameters']
        policy = config['static_map']
        self.assertTrue(policy['enabled'])
        self.assertTrue(policy['confirmation_enabled'])
        self.assertTrue(policy['clearing_enabled'])
        expected = {'min_observations': 3, 'observation_interval': .1, 'confirmation_seconds': .6,
                    'clear_observations': 3, 'clear_seconds': .4, 'clear_observation_interval': .1,
                    'max_clear_speed': 1.0, 'max_clear_angular_speed': 1.0}
        for name, value in expected.items():
            self.assertEqual(policy[name], value, name)
        self.assertEqual(policy['max_clear_position_std'], .1)
        self.assertEqual(policy['max_clear_frame_gap'], 0.5)
        self.assertEqual(policy['max_rays'], 256)
        self.assertEqual(policy['max_ray_steps'], 600)
        self.assertEqual(policy['endpoint_margin'], .5)
        self.assertEqual(policy['ray_clearance'], .025)
        self.assertGreater(policy['candidate_ttl'], policy['confirmation_seconds'])
        self.assertLessEqual(policy['ray_clearance'], config['pcd_save']['voxel_size']/2)
        for name in policy:
            self.assertIn('"static_map.' + name + '"', source)
        localization = yaml.safe_load((ROOT / 'src/FAST_LIO/config/mid360_localization.yaml').read_text())['/**']['ros__parameters']
        self.assertFalse(localization['static_map']['enabled'])

    def test_static_yaml_node_and_launch_fallback_defaults_agree(self):
        # Read source contracts only; C++ policy defaults are tested natively.
        tree = ast.parse((ROOT/'src/FAST_LIO/launch/mid360.launch.py').read_text())
        registry = next(ast.literal_eval(item.value) for item in tree.body
                        if isinstance(item, ast.Assign) and any(
                            isinstance(target, ast.Name) and target.id == '_STATIC_NUMERIC'
                            for target in item.targets))
        policy = yaml.safe_load((ROOT/'src/FAST_LIO/config/mid360.yaml').read_text())['/**']['ros__parameters']['static_map']
        source = SOURCE.read_text()
        for alias, (key, _, default) in registry.items():
            with self.subTest(alias=alias):
                declaration = re.search(r'\{"' + re.escape(key) + r'",([^}]+)\}', source)
                self.assertIsNotNone(declaration, key)
                self.assertEqual(float(declaration.group(1)), default)
                value = policy[key.split('.', 1)[1]]
                if alias == 'clear_interval':
                    self.assertEqual(default, -1.0)  # old YAML inherits its admission interval
                    self.assertEqual(value, policy['observation_interval'])
                else:
                    self.assertEqual(value, default)

    def postprocess(self):
        path = ROOT / 'src/FAST_LIO/scripts/trajectory_csv.py'
        spec = importlib.util.spec_from_file_location('runtime_postprocess', path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        return module

    def test_streaming_csv_compatibility_and_crash_recovery(self):
        helper = self.postprocess()
        header = '# fastlio_gt_recording_version: 3\n# control_source: "manual"\n# note: "line1\\nline2"\nx,y,z,qx,qy,qz,qw,stamp_sec,stamp_nanosec,effective_points,match_ratio,mean_residual\n'
        row = '1,2,3,0,0,0,1,10,0,50,0.5,0.1\n'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'episode.csv'; path.write_text(header + row + '4,5,')
            with warnings.catch_warnings(record=True) as caught:
                meta, columns, rows = helper.load_csv(path)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0][:3], [1, 2, 3])
            self.assertEqual(meta['interrupted'], 'true')
            self.assertEqual(meta['control_source'], 'manual')
            self.assertEqual(meta['note'], 'line1\nline2')
            self.assertTrue(caught)
            path.write_text(header + row + '# interrupted: false\n')
            self.assertEqual(helper.load_csv(path)[0]['interrupted'], 'false')
            path.write_text(header + 'broken\n' + row)
            with self.assertRaisesRegex(ValueError, 'before end of file'):
                helper.load_csv(path)


if __name__ == '__main__':
    unittest.main()
