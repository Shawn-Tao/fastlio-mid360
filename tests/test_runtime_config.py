#!/usr/bin/env python3
"""Host contracts and CSV recovery; not a ROS node execution test."""
import importlib.util
from pathlib import Path
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
