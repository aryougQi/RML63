"""Home return sequencing without ROS nodes, controllers or physical motion."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch, mock_open
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import return_home
    from core import Runner, Cancelled
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class ReturnHomeTests(unittest.TestCase):
    def backend(self):
        b = Mock()
        b.plan.return_value = {'Home': 'trajectory'}
        return b

    def test_default_only_plans_home(self):
        b = self.backend()
        with patch('builtins.print'): return_home.return_home(b, False)
        b.preflight.assert_called_once_with(False, False, home_only=True)
        b.plan.assert_called_once_with(['Home'], b.check)
        b.move.assert_not_called(); b.gripper.open.assert_not_called(); b.gripper.close.assert_not_called()

    def test_execution_only_moves_home(self):
        b = self.backend()
        with patch('builtins.print'): return_home.return_home(b, True)
        self.assertEqual(b.preflight.call_count, 2)
        b.move.assert_called_once_with('Home', 'trajectory')
        b.gripper.open.assert_not_called(); b.gripper.close.assert_not_called()

    def test_failed_preflight_never_plans_or_moves(self):
        b = self.backend(); b.preflight.side_effect = RuntimeError('unsafe')
        with self.assertRaises(RuntimeError): return_home.return_home(b, True)
        b.plan.assert_not_called(); b.move.assert_not_called()

    def test_failed_plan_never_moves(self):
        b = self.backend(); b.plan.side_effect = RuntimeError('collision')
        with patch('builtins.print'):
            with self.assertRaises(RuntimeError): return_home.return_home(b, True)
        b.move.assert_not_called()

    def test_cancel_before_execution_never_moves(self):
        b = self.backend(); b.check.side_effect = RuntimeError('cancelled')
        with patch('builtins.print'):
            with self.assertRaises(RuntimeError): return_home.return_home(b, True)
        b.move.assert_not_called()

    def test_service_home_preview_never_moves_or_uses_gripper(self):
        b = self.backend(); runner = Runner(b)
        runner.run_home(0)
        b.preflight.assert_called_once_with(False, False, home_only=True)
        self.assertEqual(b.plan.call_args.args[0], ['Home'])
        b.move.assert_not_called()
        b.require_open.assert_not_called()
        b.gripper.open.assert_not_called(); b.gripper.close.assert_not_called()
        self.assertTrue(runner.planned); self.assertFalse(runner.executed)
        self.assertFalse(runner.grasp_verified)

    def test_service_home_only_moves_home_after_open_and_hardware_checks(self):
        b = self.backend(); runner = Runner(b)
        runner.run_home(1)
        self.assertEqual([call.args for call in b.preflight.call_args_list], [(True, False)]*2)
        self.assertTrue(all(call.kwargs == {'home_only': True} for call in b.preflight.call_args_list))
        b.require_home_empty.assert_called_once_with(False)
        b.move.assert_called_once_with('Home', 'trajectory')
        b.prepare_gripper.assert_not_called()
        b.close.assert_not_called(); b.open.assert_not_called()
        b.gripper.open.assert_not_called(); b.gripper.close.assert_not_called()
        self.assertTrue(runner.executed)
        self.assertFalse(runner.grasp_verified); self.assertFalse(runner.holding)

    def test_service_home_failure_or_cancel_prevents_motion(self):
        for method in ('preflight', 'plan', 'require_home_empty'):
            with self.subTest(method=method):
                b = self.backend(); runner = Runner(b)
                getattr(b, method).side_effect = RuntimeError('blocked')
                with self.assertRaisesRegex(RuntimeError, 'blocked'): runner.run_home(1)
                b.move.assert_not_called()
                self.assertFalse(runner.executed)
        b = self.backend(); runner = Runner(b)
        b.require_home_empty.side_effect = lambda *args: runner.cancel.set()
        with self.assertRaises(Cancelled): runner.run_home(1)
        b.move.assert_not_called()

    def test_legacy_command_forwards_to_running_service_without_taking_control(self):
        with patch.object(return_home.rospy, 'myargv', return_value=['return_home.py', '--execute']), \
                patch.object(return_home.rospkg.RosPack, 'get_path', return_value=str(Path(__file__).resolve().parents[1])), \
                patch('builtins.open', mock_open()), patch('builtins.print'), \
                patch.object(return_home.fcntl, 'flock', side_effect=BlockingIOError()), \
                patch.object(return_home.rospy, 'init_node'), patch.object(return_home, 'Backend') as backend, \
                patch.object(return_home.moveit_commander, 'roscpp_initialize') as moveit, \
                patch.object(return_home, 'send_task', return_value=SimpleNamespace(success=True)) as request:
            return_home.main()
        request.assert_called_once_with(execute=True, home=True)
        backend.assert_not_called(); moveit.assert_not_called()

    def test_service_forwarding_reports_rejected_home(self):
        with patch.object(return_home.rospy, 'myargv', return_value=['return_home.py', '--execute']), \
                patch.object(return_home.rospkg.RosPack, 'get_path', return_value=str(Path(__file__).resolve().parents[1])), \
                patch('builtins.open', mock_open()), patch('builtins.print'), \
                patch.object(return_home.fcntl, 'flock', side_effect=BlockingIOError()), \
                patch.object(return_home.rospy, 'init_node'), \
                patch.object(return_home, 'send_task', return_value=SimpleNamespace(success=False, reason='BUSY')):
            with self.assertRaisesRegex(RuntimeError, 'BUSY'): return_home.main()


if __name__ == '__main__': unittest.main()
