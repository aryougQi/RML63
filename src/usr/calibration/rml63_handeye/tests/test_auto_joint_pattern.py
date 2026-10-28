#!/usr/bin/env python3
"""Check expanded automatic targets without planning or connecting to a robot."""
from pathlib import Path
import sys
import unittest
from copy import deepcopy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from auto_collect_joints import targets_around


class Pattern(unittest.TestCase):
    def test_count_diversity_and_bounded_displacement(self):
        start = PoseStamped()
        start.pose.position.x = .2
        start.pose.position.z = .5
        start.pose.orientation.w = 1
        original = deepcopy(start)
        targets = targets_around(start, np.radians(20), .04)
        self.assertEqual(len(targets), 33)
        self.assertEqual(start, original)
        unique = set()
        for t in targets:
            q = t.pose.orientation
            r = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            angle = np.degrees(np.arccos(np.clip((np.trace(r)-1)/2, -1, 1)))
            self.assertLessEqual(angle, 20.1)
            p = np.array([t.pose.position.x-.2, t.pose.position.y, t.pose.position.z-.5])
            self.assertLessEqual(np.linalg.norm(p), .040001)
            unique.add(tuple(np.round(np.r_[p, r.ravel()], 8)))
        self.assertEqual(len(unique), 33)


if __name__ == '__main__':
    unittest.main()
