#!/usr/bin/env python3
"""Synthetic stationary-target checks independent of any robot or ROS master."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
from tf.transformations import euler_matrix

spec = importlib.util.spec_from_file_location('validation', str(Path(__file__).resolve().parents[1] / 'scripts/validate_calibration.py'))
validation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validation)


class ValidationMath(unittest.TestCase):
    def setUp(self):
        self.x = euler_matrix(.2, -.4, .6)
        self.x[:3, 3] = [.08, .05, .04]
        self.y = euler_matrix(-.1, .3, 1.2)
        self.y[:3, 3] = [-.35, .02, .01]
        self.gs = [euler_matrix(a, b, .2) for a, b in [(-.3, -.2), (.3, -.2), (0, .4), (.3, .3)]]
        for i, g in enumerate(self.gs):
            g[:3, 3] = [.1 * i, .03 * i, .5]
        self.cs = [np.linalg.inv(self.x) @ np.linalg.inv(g) @ self.y for g in self.gs]

    def test_correct_handeye_keeps_target_fixed(self):
        values = [g @ self.x @ c for g, c in zip(self.gs, self.cs)]
        report = validation.statistics(values)
        self.assertLess(report['position_max_mm'], 1e-9)
        self.assertLess(report['rotation_max_deg'], 1e-5)
        np.testing.assert_allclose(validation.mean_transform(values), self.y, atol=1e-12)

    def test_wrong_extrinsic_is_visible_across_poses(self):
        bad = self.x.copy()
        bad[0, 3] += .02
        report = validation.statistics([g @ bad @ c for g, c in zip(self.gs, self.cs)])
        self.assertGreater(report['position_max_pair_mm'], 5)

    def test_report_distinguishes_radius_and_pairwise_drift(self):
        a, b = np.eye(4), np.eye(4)
        b[0, 3] = .01
        report = validation.statistics([a, b])
        self.assertAlmostEqual(report['position_rms_mm'], 5)
        self.assertAlmostEqual(report['position_max_pair_mm'], 10)


if __name__ == '__main__':
    unittest.main()
