#!/usr/bin/env python3
"""Validate paired calibration configuration and camera-frame composition without hardware."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch
import numpy as np
import yaml
import rospy
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from publish_joint_calibration import load_config, chain_signature, CalibratedCameraPublisher
from joint_calibration_model import RobotChain, JOINT_NAMES, rotation
from test_joint_calibration import robot_xml


class PublisherTests(unittest.TestCase):
    def setUp(self):
        self.chain = RobotChain(robot_xml())
        self.x = np.eye(4)
        self.x[:3, :3] = rotation([.2, -.3, .1])
        self.x[:3, 3] = [.08, .05, .04]
        self.config = {'schema': 1, 'joint_names': JOINT_NAMES, 'base_frame': 'base_link',
                       'nominal_effector_frame': 'link6', 'calibrated_effector_frame': 'calibrated',
                       'camera_link_frame': 'camera_link', 'camera_optical_frame': 'optical',
                       'robot_chain_sha256': chain_signature(self.chain),
                       'joint_offsets_deg': dict(zip(JOINT_NAMES, [0, .1, .3, -.2, 1., 0])),
                       'effector_to_camera_optical': {'translation_m': self.x[:3, 3].tolist(),
                                                     'quaternion_xyzw': quaternion_from_matrix(self.x).tolist()}}

    def load(self, config):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'config.yaml'
            path.write_text(yaml.safe_dump(config))
            return load_config(path, robot_xml())

    def test_degrees_and_optical_conversion_compose_to_fitted_prediction(self):
        _, chain, delta, x = self.load(self.config)
        np.testing.assert_allclose(delta, np.radians([0, .1, .3, -.2, 1., 0]))
        native = np.eye(4)
        native[:3, :3] = rotation([1., -.2, .5])
        native[:3, 3] = [.02, .03, .01]
        mount = x @ np.linalg.inv(native)
        q = [.1, -.5, .2, .8, .4, .6]
        np.testing.assert_allclose(chain.fk(q, delta) @ mount @ native, chain.fk(q, delta) @ self.x, atol=1e-12)

    def test_wrong_model_or_invalid_quaternion_refused(self):
        bad = deepcopy(self.config)
        bad['robot_chain_sha256'] = 'wrong'
        with self.assertRaisesRegex(ValueError, 'kinematic chain differs'):
            self.load(bad)
        bad = deepcopy(self.config)
        bad['effector_to_camera_optical']['quaternion_xyzw'] = [0, 0, 0, 0]
        with self.assertRaisesRegex(ValueError, 'unit length'):
            self.load(bad)

    def test_nonzero_end_joint_gauge_refused(self):
        bad = deepcopy(self.config)
        bad['joint_offsets_deg']['joint6'] = .5
        with self.assertRaisesRegex(ValueError, 'J1/J6'):
            self.load(bad)

    def test_stale_feedback_is_not_restamped_as_fresh(self):
        node = CalibratedCameraPublisher.__new__(CalibratedCameraPublisher)
        node.config, node.chain, node.delta, _ = self.load(self.config)
        node.last_stamp = rospy.Time(0)
        node.max_age = .5
        node.dynamic = MagicMock()
        msg = JointState()
        msg.name, msg.position = JOINT_NAMES, [0]*6
        msg.header.stamp = rospy.Time.from_sec(9.)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10.)), patch.object(rospy, 'logwarn_throttle'):
            node.callback(msg)
        node.dynamic.sendTransform.assert_not_called()
        msg.header.stamp = rospy.Time.from_sec(9.9)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10.)):
            node.callback(msg)
        self.assertEqual(node.dynamic.sendTransform.call_args[0][0].header.stamp, msg.header.stamp)


if __name__ == '__main__':
    unittest.main()
