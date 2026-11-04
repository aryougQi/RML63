"""Short transitions still wait for measured arrival and stable gripper feedback."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    import grasp_ros
    import rospy
    from core import Cancelled
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class TransitionTimingTests(unittest.TestCase):
    def setUp(self):
        clock = patch.object(grasp_ros.rospy.Time, 'now', return_value=rospy.Time(100))
        clock.start()
        self.addCleanup(clock.stop)

    def session(self, legacy=False):
        s = backend.Session.__new__(backend.Session)
        s.config = backend.load_settings(Path(__file__).resolve().parents[1]/'config/fixed.yaml')
        if legacy:
            for key in ('trajectory_start_delay_sec', 'trajectory_settle_timeout_sec',
                        'trajectory_settle_poll_sec', 'motion_result_poll_sec'):
                s.config.pop(key)
        s.active = ['joint%d' % i for i in range(1, 7)]
        s.offsets = np.zeros(6); s.moving = False; s.fault_message = None
        s.require_environment_verified = Mock(); s.verify_hardware = Mock()
        s.measured = Mock(return_value=(np.zeros(6), rospy.Time(100)))
        s.chain = Mock(); s.chain.fk.return_value = np.eye(4)
        s.motion_client = Mock()
        s.motion_client.wait_for_server.return_value = True
        s.motion_client.wait_for_result.return_value = True
        s.motion_client.get_state.return_value = 3
        s.motion_client.get_result.return_value = Mock(error_code=0)
        s.stop_pub = Mock()
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = s.active
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.]*6, time_from_start=rospy.Duration(t))
                                             for t in (0., 1.)]
        return s, trajectory

    def test_ready_tcp_continues_without_fixed_post_motion_sleep(self):
        s, trajectory = self.session()
        with patch.object(grasp_ros.rospy.Time, 'now', return_value=rospy.Time(100)), \
                patch.object(grasp_ros.rospy, 'sleep') as sleep:
            result = s.execute_one(trajectory, np.eye(4), np.eye(4))
        sleep.assert_not_called()
        self.assertEqual(result['position_model_error_m'], 0.)
        goal = s.motion_client.send_goal.call_args.args[0]
        self.assertAlmostEqual(goal.trajectory.header.stamp.to_sec(), 100.05)
        s.motion_client.wait_for_result.assert_called_once_with(rospy.Duration(.05))
        s.verify_hardware.assert_called_once()

    def test_controller_success_waits_for_actual_tcp_before_continuing(self):
        s, trajectory = self.session()
        delayed = np.eye(4); delayed[0, 3] = .02
        s.chain.fk.side_effect = [delayed, np.eye(4)]
        with patch.object(grasp_ros.rospy.Time, 'now', return_value=rospy.Time(100)), \
                patch.object(grasp_ros.rospy, 'sleep') as sleep:
            s.execute_one(trajectory, np.eye(4), np.eye(4))
        sleep.assert_called_once_with(.02)
        self.assertEqual(s.chain.fk.call_count, 2)

    def test_timeout_cancel_and_hardware_failure_still_abort(self):
        s, trajectory = self.session()
        delayed = np.eye(4); delayed[0, 3] = .02; s.chain.fk.return_value = delayed
        clock = [100.]
        def advance(seconds): clock[0] += seconds
        with patch.object(grasp_ros.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(grasp_ros.rospy.Time, 'now', return_value=rospy.Time(100)), \
                patch.object(grasp_ros.rospy, 'sleep', side_effect=advance):
            with self.assertRaisesRegex(RuntimeError, 'TCP did not reach target'):
                s.execute_one(trajectory, np.eye(4), np.eye(4))
        self.assertGreaterEqual(clock[0]-100., s.config['trajectory_settle_timeout_sec'])
        s, trajectory = self.session()
        s.measured.side_effect = [(np.zeros(6), rospy.Time(100)), Cancelled('cancelled')]
        with self.assertRaises(Cancelled): s.execute_one(trajectory, np.eye(4), np.eye(4))
        s, trajectory = self.session(); s.verify_hardware.side_effect = RuntimeError('disabled joint')
        with self.assertRaises(RuntimeError): s.execute_one(trajectory, np.eye(4), np.eye(4))
        s.motion_client.send_goal.assert_not_called()

    def test_fault_while_moving_still_sends_stop(self):
        s, trajectory = self.session()
        s.motion_client.wait_for_result.return_value = False
        s.fault_message = Mock(joint=[0, 1, 0, 0, 0, 0])
        with self.assertRaisesRegex(RuntimeError, 'joint fault during execution'):
            s.execute_one(trajectory, np.eye(4), np.eye(4))
        s.motion_client.cancel_goal.assert_called_once()
        s.stop_pub.publish.assert_called_once()

    def test_callers_without_timing_options_keep_legacy_delays(self):
        s, trajectory = self.session(legacy=True)
        with patch.object(grasp_ros.rospy.Time, 'now', return_value=rospy.Time(100)), \
                patch.object(grasp_ros.rospy, 'sleep') as sleep:
            s.execute_one(trajectory, np.eye(4), np.eye(4))
        sleep.assert_called_once_with(.3)
        goal = s.motion_client.send_goal.call_args.args[0]
        self.assertAlmostEqual(goal.trajectory.header.stamp.to_sec(), 100.2)
        s.motion_client.wait_for_result.assert_called_once_with(rospy.Duration(.1))

    def test_short_gripper_checks_still_require_three_samples_without_final_extra_sleep(self):
        b = backend.Backend.__new__(backend.Backend)
        b.config = backend.load_settings(Path(__file__).resolve().parents[1]/'config/fixed.yaml')
        b.gripper = Mock()
        b.gripper.query.return_value = dict(status=1, enable=1, error=0, mode=3, current_force=0)
        with patch.object(backend.time, 'sleep') as sleep:
            b.require_open()
        self.assertEqual(b.gripper.query.call_count, 3)
        self.assertEqual([c.args[0] for c in sleep.call_args_list], [.05, .05])
        b.gripper.query.reset_mock()
        b.gripper.query.return_value = dict(status=1, enable=1, error=0, mode=6, current_force=500)
        with patch.object(backend.time, 'sleep') as sleep:
            b.verify_hold()
        self.assertEqual(b.gripper.query.call_count, 3)
        self.assertEqual([c.args[0] for c in sleep.call_args_list], [.05, .05])

    def test_open_and_close_keep_three_stable_feedback_samples_with_short_polling(self):
        config = backend.load_settings(Path(__file__).resolve().parents[1]/'config/fixed.yaml')
        gripper = backend.Gripper.__new__(backend.Gripper)
        gripper.config = config; gripper.check = Mock()
        gripper._connected = Mock(); gripper.position_pub = Mock(); gripper.pick_pub = Mock()
        state = dict(status=1, enable=1, error=0, mode=3, current_force=0, actpos=714)
        gripper.query = Mock(side_effect=[dict(voltage_type=3), state, state, state])
        with patch.object(backend.rospy, 'sleep') as sleep:
            result = gripper.open(.050)
        self.assertEqual(len(result['stable_feedback_samples']), 3)
        self.assertEqual([c.args[0] for c in sleep.call_args_list], [.05, .05])
        gripper.query.side_effect = None
        gripper.query.return_value = dict(state, mode=6, current_force=500)
        b = backend.Backend.__new__(backend.Backend)
        b.config = config; b.gripper = gripper; b.check = Mock(); b.verify_hold = Mock()
        gripper.query.reset_mock()
        with patch.object(backend.rospy, 'sleep') as sleep:
            b.close()
        self.assertEqual(gripper.query.call_count, 3)
        b.verify_hold.assert_not_called()
        self.assertEqual([c.args[0] for c in sleep.call_args_list], [.05, .05])

    def test_invalid_timing_values_are_rejected_before_execution(self):
        for key, value in (('trajectory_start_delay_sec', -1), ('trajectory_settle_timeout_sec', -1),
                           ('trajectory_settle_poll_sec', 0), ('motion_result_poll_sec', float('nan')),
                           ('gripper_poll_interval_sec', 0)):
            with self.subTest(key=key), tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as file:
                yaml.safe_dump({key: value}, file); file.flush()
                with self.assertRaisesRegex(ValueError, key): backend.load_settings(file.name)


if __name__ == '__main__': unittest.main()
