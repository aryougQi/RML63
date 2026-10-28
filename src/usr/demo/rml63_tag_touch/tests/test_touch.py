"""Offline geometry and execution-gating tests. Never initialize MoveIt or send ROS goals."""
from copy import deepcopy
from pathlib import Path
import hashlib
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
from tf.transformations import euler_matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from touch_geometry import calibration_matrix, joint_calibration_matrix, tcp_transform, touch_targets
from touch_tag import TouchTag, check_trajectory, state_after


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.tag = np.eye(4)
        self.tag[:3, 3] = [.4, .1, .0]
        self.camera = np.eye(4)
        self.camera[:3, 3] = [.4, .1, .4]
        self.hand = euler_matrix(np.pi, 0, .3)
        self.hand[:3, 3] = [.3, .1, .4]
        self.tcp = tcp_transform([0, 0, .15], [0, 0, 0])

    def test_contact_is_center_with_rotated_tcp_offset(self):
        self.tcp = tcp_transform([.012, -.004, .150], [.1, .2, -.4])
        tcp, tools, normal = touch_targets(self.tag, self.camera, self.hand, self.tcp, .05, 0, True)
        np.testing.assert_allclose(tcp[1][:3, 3], self.tag[:3, 3])
        np.testing.assert_allclose(tcp[0][:3, 3], self.tag[:3, 3] + .05 * normal)
        for target, tool in zip(tcp, tools):
            np.testing.assert_allclose(tool @ self.tcp, target, atol=1e-12)
        np.testing.assert_allclose(tcp[1][:3, 2], -normal)
        self.assertAlmostEqual(np.linalg.det(tcp[1][:3, :3]), 1)

    def test_flipped_tag_normal_still_approaches_from_camera_side(self):
        self.tag[:3, :3] = euler_matrix(np.pi, 0, 0)[:3, :3]
        tcp, _, normal = touch_targets(self.tag, self.camera, self.hand, self.tcp, .05, .02, True)
        np.testing.assert_allclose(normal, [0, 0, 1], atol=1e-12)
        self.assertAlmostEqual(tcp[1][2, 3], .02)

    def test_tilted_board_and_preserved_orientation(self):
        self.tag[:3, :3] = euler_matrix(.3, -.4, .1)[:3, :3]
        tcp, tools, normal = touch_targets(self.tag, self.camera, self.hand, self.tcp, .05, 0, False)
        np.testing.assert_allclose(tcp[1][:3, :3], (self.hand @ self.tcp)[:3, :3])
        np.testing.assert_allclose(tcp[0][:3, 3] - tcp[1][:3, 3], .05 * normal)
        np.testing.assert_allclose(tools[1] @ self.tcp, tcp[1], atol=1e-12)

    def test_reject_tcp_on_other_side_of_board(self):
        self.hand[2, 3] = -.2
        with self.assertRaises(ValueError):
            touch_targets(self.tag, self.camera, self.hand, self.tcp, .05, 0, True)

    def test_reject_negative_clearance(self):
        with self.assertRaises(ValueError):
            touch_targets(self.tag, self.camera, self.hand, self.tcp, .05, -.001, True)

    def test_stop_five_cm_above_center(self):
        tcp, tools, normal = touch_targets(self.tag, self.camera, self.hand, self.tcp, .10, .05, True)
        np.testing.assert_allclose(tcp[1][:3, 3], self.tag[:3, 3] + .05 * normal)
        np.testing.assert_allclose(tools[1] @ self.tcp, tcp[1], atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(tcp[1][:3, 3] - self.tag[:3, 3]), .05)

    def test_calibration_frame_mismatch_and_zero_quaternion(self):
        params = dict(eye_on_hand=True, robot_base_frame='base_link', robot_effector_frame='link6',
                      tracking_base_frame='camera_color_optical_frame', tracking_marker_frame='tag')
        data = dict(parameters=params, transformation=dict(x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1))
        np.testing.assert_allclose(calibration_matrix(data, 'base_link', 'link6', 'camera_color_optical_frame', 'tag'), np.eye(4))
        with self.assertRaises(ValueError):
            calibration_matrix(data, 'base_link', 'link6', 'camera_link', 'tag')
        data['transformation']['qw'] = 0
        with self.assertRaises(ValueError):
            calibration_matrix(data, 'base_link', 'link6', 'camera_color_optical_frame', 'tag')


def state_and_plan():
    state, plan = RobotState(), RobotTrajectory()
    state.joint_state.name = ['joint1', 'joint2']
    state.joint_state.position = [.1, .2]
    plan.joint_trajectory.joint_names = ['joint2', 'joint1']
    for positions, stamp in [([.2, .1], 0), ([.21, .12], 1)]:
        point = JointTrajectoryPoint(positions=positions, time_from_start=rospy.Duration(stamp))
        plan.joint_trajectory.points.append(point)
    return state, plan


class PlanningTests(unittest.TestCase):
    def test_state_after_respects_joint_names(self):
        state, plan = state_and_plan()
        result = state_after(state, plan)
        self.assertEqual(result.joint_state.position, [.12, .21])
        self.assertEqual(state.joint_state.position, [.1, .2])
        check_trajectory(plan, state, ['joint1', 'joint2'], 90, 10)

    def test_reject_start_mismatch_joint_jump_and_bad_timing(self):
        state, plan = state_and_plan()
        for mutate in [lambda p: setattr(p.joint_trajectory.points[0], 'positions', [1., .1]),
                       lambda p: setattr(p.joint_trajectory.points[1], 'positions', [.9, .1]),
                       lambda p: setattr(p.joint_trajectory.points[1], 'time_from_start', rospy.Duration(0))]:
            broken = deepcopy(plan)
            mutate(broken)
            with self.assertRaises(RuntimeError):
                check_trajectory(broken, state, ['joint1', 'joint2'], 90, 10)

    @patch('touch_tag.rospy.Time.now', return_value=rospy.Time(10))
    def test_partial_cartesian_path_never_executes(self, now):
        node = TouchTag.__new__(TouchTag)
        node.group = MagicMock()
        node.base, node.eef_step = 'base_link', .002
        node.group.compute_cartesian_path.return_value = (RobotTrajectory(), .99)
        with self.assertRaises(RuntimeError):
            node.plan_line(np.eye(4), RobotState())
        node.group.execute.assert_not_called()
        node.group.retime_trajectory.assert_not_called()

    @patch('touch_tag.rospy.get_param')
    def test_preview_and_failed_preflight_never_execute_or_stop(self, param):
        param.side_effect = lambda name, default=None: {
            '~keep_alive': False, '~retract': True}.get(name, default)
        node = TouchTag.__new__(TouchTag)
        node.use_joint_offsets = False
        node.execute, node.base, node.tool = False, 'base_link', 'link6'
        node.tcp = tcp_transform([0, 0, .15], [0, 0, 0])
        node.approach, node.clearance, node.max_distance = .05, 0, .4
        node.group, node.robot = MagicMock(), MagicMock()
        tag, camera = np.eye(4), np.eye(4)
        camera[2, 3] = .3
        hand = euler_matrix(np.pi, 0, 0)
        hand[2, 3] = .3
        node.capture = MagicMock(return_value=(tag, camera, hand))
        node.fresh_tool = MagicMock(return_value=hand)
        state, plan = state_and_plan()
        node.robot.get_current_state.return_value = state
        node.plan_pose = MagicMock(return_value=plan)
        node.plan_line = MagicMock(return_value=plan)
        node.preview = MagicMock()
        node.execute_one = MagicMock()
        node.run()
        self.assertEqual(node.plan_line.call_count, 2)
        node.execute_one.assert_not_called()
        node.group.execute.assert_not_called()
        node.group.stop.assert_not_called()
        node.plan_line.side_effect = RuntimeError('Incomplete path')
        with self.assertRaises(RuntimeError):
            node.run()
        node.execute_one.assert_not_called()
        node.group.execute.assert_not_called()


class JointCalibrationTests(unittest.TestCase):
    def config(self):
        return dict(schema=1, joint_names=['joint%d' % i for i in range(1, 7)],
                    joint_offsets_deg={'joint%d' % i: 0.1 for i in range(1, 7)},
                    base_frame='base_link', nominal_effector_frame='link6',
                    calibrated_effector_frame='rml63_link6_calibrated',
                    camera_optical_frame='camera_color_optical_frame',
                    effector_to_camera_optical=dict(translation_m=[.08, .06, .05],
                                                   quaternion_xyzw=[0, 0, 0, 1]))

    def test_paired_schema_does_not_accept_old_handeye_or_wrong_frames(self):
        config = self.config()
        x, frame = joint_calibration_matrix(config, 'base_link', 'link6', 'camera_color_optical_frame')
        self.assertEqual(frame, 'rml63_link6_calibrated')
        np.testing.assert_allclose(x[:3, 3], [.08, .06, .05])
        config['nominal_effector_frame'] = 'other_tool'
        with self.assertRaises(ValueError):
            joint_calibration_matrix(config, 'base_link', 'link6', 'camera_color_optical_frame')
        with self.assertRaises(ValueError):
            joint_calibration_matrix({'parameters': {}}, 'base_link', 'link6', 'camera_color_optical_frame')

    def test_camera_uses_corrected_fk_at_image_time_not_nominal_fk(self):
        node = TouchTag.__new__(TouchTag)
        node.use_joint_offsets = True
        node.base, node.corrected_tool = 'base_link', 'rml63_link6_calibrated'
        node.handeye, _ = joint_calibration_matrix(self.config(), 'base_link', 'link6', 'camera_color_optical_frame')
        corrected = TransformStamped()
        corrected.transform.rotation.w = 1
        corrected.transform.translation.x = .015
        node.buffer = MagicMock()
        node.buffer.lookup_transform.return_value = corrected
        stamp = rospy.Time(123, 456)
        camera = node.camera_at(stamp, np.eye(4))
        self.assertAlmostEqual(camera[0, 3], .095)
        node.buffer.lookup_transform.assert_called_once_with(
            'base_link', 'rml63_link6_calibrated', stamp, rospy.Duration(.1))

    @patch('touch_tag.rospy.get_param')
    def test_edited_file_or_publisher_mismatch_is_rejected(self, param):
        with tempfile.TemporaryDirectory() as directory:
            node = TouchTag.__new__(TouchTag)
            node.use_joint_offsets = True
            node.calibration_path = Path(directory) / 'active.yaml'
            node.calibration_path.write_bytes(b'original paired parameters')
            node.calibration_hash = hashlib.sha256(node.calibration_path.read_bytes()).hexdigest()
            node.calibration_publisher = '/rml63_handeye_tf'
            param.return_value = node.calibration_hash
            node.verify_calibration_source()
            param.return_value = 'older parameters'
            with self.assertRaises(RuntimeError):
                node.verify_calibration_source()
            param.return_value = node.calibration_hash
            node.calibration_path.write_bytes(b'new paired parameters')
            with self.assertRaises(RuntimeError):
                node.verify_calibration_source()


if __name__ == '__main__':
    unittest.main()
