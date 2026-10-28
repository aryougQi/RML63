#!/usr/bin/env python3
"""Synthetic recovery, gauge ambiguity and held-out accuracy; no hardware required."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from joint_calibration_model import RobotChain, rotation, fit, evaluate


def robot_xml():
    origins = [('0 0 .2', '0 0 0'), ('0 0 0', '1.5708 0 0'),
               ('.38 0 0', '0 0 0'), ('.07 -.4 0', '1.5708 0 0'),
               ('0 0 0', '1.5708 0 0'), ('0 .12 0', '1.5708 0 0')]
    joints = []
    for i, (xyz, rpy) in enumerate(origins):
        joints.append('<joint name="joint%d" type="revolute"><parent link="%s"/>'
                      '<child link="link%d"/><origin xyz="%s" rpy="%s"/><axis xyz="0 0 1"/></joint>' %
                      (i+1, 'base_link' if i == 0 else 'link%d' % i, i+1, xyz, rpy))
    return '<robot name="synthetic">'+''.join(joints)+'</robot>'


class JointCalibration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.chain = RobotChain(robot_xml())
        cls.x = np.eye(4)
        cls.x[:3, :3] = rotation([.2, -.3, .4])
        cls.x[:3, 3] = [.08, .05, .04]
        cls.y = np.eye(4)
        cls.y[:3, :3] = rotation([.3, -.1, -.2])
        cls.y[:3, 3] = [-.35, .02, 0]
        cls.delta = np.radians([0, .4, -.3, .25, .2, 0])
        cls.samples = []
        for q in np.random.RandomState(12).uniform(-1.3, 1.3, (32, 6)):
            c = np.linalg.inv(cls.x) @ np.linalg.inv(cls.chain.fk(q, cls.delta)) @ cls.y
            cls.samples.append({'joint_positions_rad': q.tolist(), 'camera_marker_matrix': c.tolist()})
        cls.result = fit(cls.chain, cls.samples[:24], cls.x)

    def test_offset_recovery_and_sign(self):
        self.assertTrue(self.result['success'])
        np.testing.assert_allclose(self.result['offsets_deg'], np.degrees(self.delta), atol=.02)
        self.assertEqual(self.result['data_jacobian_rank'], self.result['parameter_count'])
        self.assertFalse(self.result['offset_bound_reached'])

    def test_improves_unseen_poses_against_same_baseline(self):
        baseline = fit(self.chain, self.samples[:24], self.x, with_offsets=False)
        corrected = evaluate(self.chain, self.samples[24:], self.result)
        uncorrected = evaluate(self.chain, self.samples[24:], baseline)
        self.assertLess(corrected['position_rms_mm'], .1)
        self.assertGreater(uncorrected['position_rms_mm'], 1)
        self.assertLess(corrected['position_rms_mm'], uncorrected['position_rms_mm']*.1)

    def test_end_joint_offsets_can_be_absorbed_by_external_frames(self):
        d = .05
        first_origin, first_axis, _ = self.chain.chain[0]
        rz = np.eye(4)
        rz[:3, :3] = rotation(first_axis*d)
        base_change = first_origin @ rz @ np.linalg.inv(first_origin)
        last_change = np.eye(4)
        last_change[:3, :3] = rotation(self.chain.chain[-1][1]*d)
        for sample in self.samples[:3]:
            q = sample['joint_positions_rad']
            np.testing.assert_allclose(self.chain.fk(q, [d, 0, 0, 0, 0, 0]), base_change@self.chain.fk(q), atol=1e-12)
            np.testing.assert_allclose(self.chain.fk(q, [0, 0, 0, 0, 0, d])@self.x,
                                       self.chain.fk(q)@last_change@self.x, atol=1e-12)

    def test_priors_do_not_make_repeated_pose_observable(self):
        result = fit(self.chain, [self.samples[0]]*8, self.x)
        self.assertLess(result['data_jacobian_rank'], result['parameter_count'])


if __name__ == '__main__':
    unittest.main()
