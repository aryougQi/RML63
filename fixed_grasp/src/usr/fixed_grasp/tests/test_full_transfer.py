"""Full transfer release checks with simulated feedback; no ROS node or hardware."""
import hashlib
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import backend
from core import Runner


class FullTransferTests(unittest.TestCase):
    def model(self):
        root = Path(__file__).resolve().parents[1]
        b = backend.Backend.__new__(backend.Backend)
        b.config = backend.load_settings(root / 'config/fixed.yaml')
        b.waypoints_sha256 = hashlib.sha256((root / 'config/waypoints.yaml').read_bytes()).hexdigest()
        b.check = Mock()
        b.s = Mock()
        b.s.offsets = np.zeros(6)
        b.s.chain.fk.return_value = np.eye(4)
        b.s.measured.side_effect = [(np.zeros(6), rospy.Time(i)) for i in (1, 2, 3)]
        b.tcp = np.eye(4)
        b.poses = {name: np.eye(4) for name in
                   ('Home', 'Pregrasp', 'Grasp', 'Lift', 'Preplace', 'Place', 'Retreat')}
        b.poses['Lift'][2, 3] = .15
        b.gripper = Mock()
        b.gripper.query.return_value = dict(status=1, enable=1, error=0, mode=6, current_force=500)
        b.allow = True
        b.scene_ready = False
        b.require_open = Mock()
        b.build_scene = Mock()
        return b

    def runner(self, b):
        b.plan = Mock(return_value=dict.fromkeys(list(b.poses) + ['ReturnHome']))
        b.move = Mock()
        b.prepare_gripper = Mock()
        b.close = Mock()
        b.attach = Mock()
        b.detach = Mock()
        return Runner(b)

    def test_confirmed_waypoints_enable_full_preflight_without_service(self):
        b = self.model()
        with patch.object(backend.rospy, 'logwarn'):
            b.preflight(True, True)
        b.s.verify_hardware.assert_called_once()
        b.s.require_environment_verified.assert_called_once()
        b.require_open.assert_called_once()

    def test_changed_or_unconfirmed_teaching_is_rejected_before_hardware(self):
        for update in ({'verified': False}, {'source': ''}, {'waypoints_sha256': 'changed'}):
            with self.subTest(update=update):
                b = self.model()
                b.config['taught_place_verification'].update(update)
                with self.assertRaisesRegex(RuntimeError, 'release not verified'):
                    b.preflight(True, True)
                b.s.verify_hardware.assert_not_called()
                b.build_scene.assert_not_called()

    def test_stationary_taught_place_releases_retreats_and_returns_home(self):
        b = self.model()
        r = self.runner(b)
        with patch.object(backend.rospy, 'sleep'), patch.object(backend.time, 'sleep'), \
                patch.object(backend.rospy, 'logwarn'), patch.object(backend.rospy, 'ServiceProxy') as service:
            r.run(1, True)
        service.assert_not_called()
        b.gripper.open.assert_called_once_with(.050)
        self.assertEqual(b.s.measured.call_count, 3)
        self.assertEqual([call.args[0] for call in b.move.call_args_list],
                         ['Home', 'Pregrasp', 'Grasp', 'Lift', 'Preplace', 'Place', 'Retreat', 'ReturnHome'])
        b.detach.assert_called_once()
        self.assertTrue(r.executed)
        self.assertTrue(r.grasp_verified)
        self.assertFalse(r.holding)

    def test_position_orientation_or_motion_failure_never_opens_or_retreats(self):
        translated = np.eye(4); translated[0, 3] = .005
        rotated = np.eye(4); rotated[:2, :2] = [[0, -1], [1, 0]]
        nonfinite = np.eye(4); nonfinite[0, 3] = float('nan')
        for pose in (translated, rotated, nonfinite):
            with self.subTest(pose=pose):
                b = self.model()
                b.s.chain.fk.return_value = pose
                self.assert_no_release(b)
        b = self.model()
        b.s.measured.side_effect = [(np.full(6, value), rospy.Time(i))
                                    for i, value in enumerate((0., .003, .003), 1)]
        self.assert_no_release(b)

    def test_stale_feedback_hardware_error_and_force_loss_prevent_release(self):
        b = self.model()
        b.s.measured.side_effect = [(np.zeros(6), rospy.Time(1))] * 3
        self.assert_no_release(b)
        b = self.model()
        b.s.measured.side_effect = RuntimeError('joint feedback stale')
        self.assert_no_release(b)
        b = self.model()
        b.s.verify_hardware.side_effect = RuntimeError('joint fault')
        self.assert_no_release(b)
        b = self.model()
        b.gripper.query.return_value['current_force'] = 0
        self.assert_no_release(b)

    def assert_no_release(self, b):
        r = self.runner(b)
        with patch.object(backend.rospy, 'sleep'), patch.object(backend.time, 'sleep'), \
                patch.object(backend.rospy, 'logwarn'):
            with self.assertRaises((RuntimeError, ValueError)):
                r.run(1, True)
        b.gripper.open.assert_not_called()
        b.detach.assert_not_called()
        self.assertNotIn('Retreat', [call.args[0] for call in b.move.call_args_list])
        self.assertFalse(r.executed)

    def test_plan_only_never_checks_release_or_uses_gripper(self):
        b = self.model()
        b.config['taught_place_verification']['verified'] = False
        r = self.runner(b)
        r.run(0, True)
        b.s.verify_hardware.assert_not_called()
        b.s.measured.assert_not_called()
        b.move.assert_not_called()
        b.gripper.open.assert_not_called()
        b.gripper.query.assert_not_called()

    def test_initial_home_and_return_home_keep_distinct_plans_and_start_states(self):
        b = self.model()
        b.s.active = ['j%d' % i for i in range(6)]
        b.data = dict(waypoints=dict(Home=dict(joints_rad=[.1]*6)))
        b.s.solve.return_value = np.full(6, .2)
        b.set_basket = Mock()
        def state_for(q, reference=None):
            state = RobotState(is_diff=True)
            state.joint_state.name = b.s.active
            state.joint_state.position = list(q)
            return state
        b.s.state_for.side_effect = state_for
        def trajectory(start, end):
            result = RobotTrajectory()
            result.joint_trajectory.joint_names = b.s.active
            result.joint_trajectory.points = [JointTrajectoryPoint(positions=[value]*6)
                                              for value in (start, end)]
            return result
        outward = trajectory(0., .1)
        transfer = trajectory(.1, .2)
        homeward = trajectory(.2, .1)
        b.s.plan_joint.side_effect = [outward, transfer, homeward]
        plans = b.plan(['Home', 'Preplace', 'ReturnHome'], b.check)
        self.assertIs(plans['Home'], outward)
        self.assertIs(plans['ReturnHome'], homeward)
        starts = [list(call.args[1].joint_state.position) for call in b.s.plan_joint.call_args_list]
        self.assertEqual(starts, [[0.]*6, [.1]*6, [.2]*6])
        self.assertEqual(list(plans), ['Home', 'Preplace', 'ReturnHome'])

    def test_return_home_requires_open_feedback_and_checks_home_target(self):
        b = self.model()
        b.attached = False
        b.poses['Home'][0, 3] = .2
        path = Mock()
        b.move('ReturnHome', path)
        b.require_open.assert_called_once()
        np.testing.assert_array_equal(b.s.execute_one.call_args.args[2], b.poses['Home'])
        b.s.execute_one.reset_mock()
        b.s.check_trajectory.reset_mock()
        b.require_open.side_effect = RuntimeError('gripper not open')
        with self.assertRaisesRegex(RuntimeError, 'gripper not open'):
            b.move('ReturnHome', path)
        b.s.execute_one.assert_not_called()
        b.s.check_trajectory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
