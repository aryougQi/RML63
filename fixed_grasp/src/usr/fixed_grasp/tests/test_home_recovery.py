"""Explicit empty Home recovery without any ROS nodes or physical commands."""
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    import server
    import client
    from core import Runner, Cancelled
    from fixed_grasp.msg import FixedGraspGoal, FixedGraspResult
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class HomeRecoveryTests(unittest.TestCase):
    def backend(self):
        b = backend.Backend.__new__(backend.Backend)
        b.config = backend.load_settings(Path(__file__).resolve().parents[1]/'config/fixed.yaml')
        b.check = Mock(); b.gripper = Mock()
        b.gripper.query.return_value = dict(status=0, enable=0, error=0, mode=0, actpos=0, current_force=0)
        return b

    def test_all_zero_startup_feedback_requires_explicit_empty_confirmation(self):
        b = self.backend()
        with self.assertRaisesRegex(RuntimeError, 'gripper preflight failed'):
            b.require_home_empty()
        with patch.object(backend.time, 'sleep'), patch.object(backend.rospy, 'logwarn'):
            b.require_home_empty(True)
        b.gripper.open.assert_not_called(); b.gripper.close.assert_not_called()

    def test_confirmation_never_overrides_load_movement_fault_or_invalid_feedback(self):
        for update in ({'mode': 4}, {'mode': 5}, {'mode': 6}, {'current_force': 50},
                       {'current_force': -50}, {'current_force': float('nan')}, {'error': 1},
                       {'current_force': None}, {'status': None}):
            with self.subTest(update=update):
                b = self.backend(); b.gripper.query.return_value.update(update)
                with self.assertRaisesRegex(RuntimeError, 'confirmed-empty Home blocked'):
                    b.require_home_empty(True)
        b = self.backend(); b.gripper.query.side_effect = OSError('disconnected')
        with self.assertRaises(OSError): b.require_home_empty(True)
        for mode in ('robot_only', 'measured'):
            b = self.backend(); b.config['scene_mode'] = mode
            with self.assertRaises(RuntimeError): b.require_home_empty(True)

    def test_cancellation_during_empty_feedback_prevents_home_dispatch(self):
        b = Mock(); b.plan.return_value = {'Home': 'trajectory'}
        runner = Runner(b)
        b.require_home_empty.side_effect = lambda *args: runner.cancel.set()
        with self.assertRaises(Cancelled): runner.run_home(1, confirm_empty=True)
        b.move.assert_not_called()
        self.assertTrue(all(call.kwargs == {'home_only': True, 'confirm_empty': True}
                            for call in b.preflight.call_args_list))

    def test_recovery_admission_still_rejects_busy_tasks_and_grasp_requests(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.needs_recovery = True; s.runner = None; s.active = Mock()
        handle = Mock(); s.goal(handle, 'home', confirm_empty=True)
        handle.set_rejected.assert_called_once(); handle.set_accepted.assert_not_called()
        s.active = None
        handle = Mock(); s.goal(handle, 1, confirm_empty=True)
        handle.set_rejected.assert_called_once()
        handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=1)
        with patch.object(server.threading, 'Thread'):
            s.goal(handle, 'home', confirm_empty=True)
        handle.set_accepted.assert_called_once()
        self.assertTrue(s.needs_recovery)

    def test_only_successful_executed_empty_home_clears_latch_and_stop_failure_finishes_goal(self):
        for mode, failure in ((0, None), (1, None), (1, 'plan'), (1, 'stop')):
            with self.subTest(mode=mode, failure=failure):
                s = server.Server.__new__(server.Server)
                s.lock = threading.RLock(); s.active = Mock(); s.backend = None
                s.needs_recovery = True; s.recovery_stage = 'CLOSE'; s.recovery_reason = 'old failure'
                handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=mode)
                runner = Runner(None)
                b = Mock(); b.config = {'scene_mode': 'existing'}
                b.plan.return_value = {'Home': 'trajectory'}
                b.move.side_effect = lambda *args: b.s.motion_client.send_goal('Home')
                if failure: getattr(b, failure).side_effect = RuntimeError('test '+failure)
                with patch.object(server, 'load_settings', return_value=b.config), \
                        patch.object(server, 'Backend', return_value=b), patch.object(server.rospy, 'get_param'), \
                        patch.object(server.rospy, 'logwarn'):
                    s.work(handle, runner, 'home', confirm_empty=True)
                self.assertIsNone(s.active)
                self.assertEqual(s.needs_recovery, mode == 0 or failure is not None)
                if failure:
                    handle.set_aborted.assert_called_once()
                else:
                    handle.set_succeeded.assert_called_once()
                b.gripper.open.assert_not_called(); b.gripper.close.assert_not_called()

    def test_client_selects_recovery_endpoint_and_reports_failed_action(self):
        with patch.object(client.rospy, 'myargv', return_value=['client.py', '--home', '--confirm-empty', '--execute']), \
                patch.object(client.rospy, 'init_node'), patch.object(client.actionlib, 'SimpleActionClient') as factory, \
                patch('builtins.print'):
            factory.return_value.wait_for_result.return_value = True
            factory.return_value.get_result.return_value = FixedGraspResult(success=False)
            self.assertEqual(client.main(), 1)
        self.assertEqual(factory.call_args.args[0], '/fixed_grasp/home_empty')

    def test_shutdown_requests_cancel_instead_of_spinning(self):
        with patch.object(client.actionlib, 'SimpleActionClient') as factory, \
                patch.object(client.rospy, 'is_shutdown', return_value=True):
            factory.return_value.wait_for_result.return_value = False
            with self.assertRaisesRegex(RuntimeError, 'ROS shut down'):
                client.send_task(home=True)
            factory.return_value.cancel_goal.assert_called_once()


if __name__ == '__main__': unittest.main()
