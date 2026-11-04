"""Offline generation preserves measured Grasp and global-Z/orientation semantics."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from waypoint_offsets import derive_from_grasp, offset_pose
import derive_grasp


class OffsetTests(unittest.TestCase):
    def source(self):
        pose = np.eye(4)
        pose[:3,:3] = [[0,0,1],[0,1,0],[-1,0,0]]
        pose[:3,3] = [1.,2.,3.]
        return dict(schema=1, identity={'unchanged': True}, waypoints=dict(
            Grasp=dict(tcp_base=pose.tolist(), joints_rad=[.1]*6),
            Pregrasp={'old': 'measured'}, Lift={'old': 'measured'}, Home={'untouched': True}))

    def test_same_orientation_and_150mm_in_base_z(self):
        source = self.source(); result = derive_from_grasp(source)
        for name in ('Pregrasp', 'Lift'):
            rec = result['waypoints'][name]
            expected = np.asarray(source['waypoints']['Grasp']['tcp_base']); expected[2,3] += .15
            np.testing.assert_array_equal(rec['tcp_base'], expected)
            self.assertEqual(rec['derived_from'], 'Grasp')
            self.assertNotIn('joints_rad', rec)
        self.assertEqual(result['waypoints']['Grasp'], source['waypoints']['Grasp'])
        self.assertEqual(result['waypoints']['Home'], source['waypoints']['Home'])
        self.assertEqual(result['identity'], source['identity'])
        self.assertEqual(source['waypoints']['Pregrasp'], {'old': 'measured'})

    def test_invalid_offsets_rejected(self):
        for offset in ([0,0,0], [0,0,-.15], [.01,0,.15], [0,0,float('nan')], [0,0,float('inf')]):
            with self.assertRaises(ValueError): offset_pose(np.eye(4), offset)

    def test_missing_or_unmeasured_grasp_rejected(self):
        source = self.source(); del source['waypoints']['Grasp']
        with self.assertRaises(ValueError): derive_from_grasp(source)
        source = self.source(); source['waypoints']['Grasp']['derived_from'] = 'Home'
        with self.assertRaises(ValueError): derive_from_grasp(source)

    def test_cli_backs_up_original_before_replacing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.yaml'
            original = yaml.safe_dump(self.source()); path.write_text(original)
            with patch.object(sys, 'argv', ['derive_grasp.py', '--waypoints', str(path)]), patch('builtins.print'):
                derive_grasp.main()
            backups = list(path.parent.glob('*.bak'))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_text(), original)
            self.assertEqual(yaml.safe_load(path.read_text())['waypoints']['Lift']['offset_base_m'], [0.,0.,.15])


if __name__ == '__main__': unittest.main()
