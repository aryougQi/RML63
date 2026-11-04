"""Startup ordering and gripper commands with mocked ROS; no nodes or hardware."""
from contextlib import ExitStack
from pathlib import Path
import sys
import subprocess
import unittest
from unittest.mock import Mock, mock_open, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import start_system
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class StartupTests(unittest.TestCase):
    def test_shell_entry_preserves_its_arguments_while_sourcing_ros(self):
        entry = Path(__file__).resolve().parents[2]/'start_fixed_grasp.sh'
        result = subprocess.run(['bash', str(entry), '--help'], capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--confirm-empty', result.stdout)

    def gripper(self):
        gripper = Mock()
        gripper.query.side_effect = [dict(status=0, enable=0, error=0, mode=0, current_force=0),
                                     dict(voltage_type=3)]
        return gripper

    def test_startup_powers_then_opens_without_requiring_initialized_state_first(self):
        gripper = self.gripper(); publisher = Mock()
        events = []
        publisher.publish.side_effect = lambda message: events.append(('power', message.data))
        gripper.open.side_effect = lambda width: events.append(('open', width))
        with patch.object(start_system, 'Gripper', return_value=gripper), \
                patch.object(start_system.rospy, 'Publisher', return_value=publisher), \
                patch.object(start_system.rospy, 'loginfo'):
            start_system.initialize_gripper({'open_width_m': .050}, Mock())
        self.assertEqual(events, [('power', 3), ('open', .050)])
        publisher.unregister.assert_called_once()
        gripper.require_open.assert_not_called()
        gripper.close.assert_not_called()

    def test_loaded_moving_or_faulted_gripper_never_receives_power_or_open_command(self):
        for update in ({'mode': 6}, {'mode': 4}, {'mode': 5}, {'current_force': 50},
                       {'current_force': float('nan')}, {'error': 1}):
            with self.subTest(update=update):
                gripper = self.gripper()
                state = dict(status=1, enable=1, error=0, mode=3, current_force=0)
                state.update(update); gripper.query.side_effect = None; gripper.query.return_value = state
                with patch.object(start_system, 'Gripper', return_value=gripper), \
                        patch.object(start_system.rospy, 'Publisher') as publisher:
                    with self.assertRaises(RuntimeError):
                        start_system.initialize_gripper({'open_width_m': .050}, Mock())
                publisher.assert_not_called(); gripper.open.assert_not_called()

    def test_unconfirmed_voltage_and_failed_open_never_report_success(self):
        for failure in ('power', 'open'):
            with self.subTest(failure=failure):
                gripper = self.gripper()
                if failure == 'power':
                    gripper.query.side_effect = [dict(error=0, mode=0, current_force=0), dict(voltage_type=0)]
                else:
                    gripper.open.side_effect = RuntimeError('gripper still offline')
                with patch.object(start_system, 'Gripper', return_value=gripper), \
                        patch.object(start_system.rospy, 'Publisher'), patch.object(start_system.rospy, 'loginfo'), \
                        patch.object(start_system.time, 'monotonic', side_effect=[0., 11.]):
                    with self.assertRaises(RuntimeError):
                        start_system.initialize_gripper({'open_width_m': .050}, Mock())
                if failure == 'power': gripper.open.assert_not_called()

    def test_confirmation_required_before_launching(self):
        with patch('sys.stderr'), patch.object(start_system.roslaunch.parent, 'ROSLaunchParent') as launch:
            with self.assertRaises(SystemExit): start_system.main([])
        launch.assert_not_called()

    def test_startup_failure_and_ctrl_c_clean_up_only_started_launches(self):
        for failure in ('initialize', 'ctrl_c'):
            with self.subTest(failure=failure), ExitStack() as stack:
                robot = Mock(); robot.pm.is_shutdown = False
                task = Mock(); task.pm.is_shutdown = False
                stack.enter_context(patch('builtins.open', mock_open()))
                flock = stack.enter_context(patch.object(start_system.fcntl, 'flock'))
                stack.enter_context(patch.object(start_system.rospkg, 'RosPack'))
                stack.enter_context(patch.object(start_system, 'load_settings', return_value={'robot_ip': '192.168.1.18'}))
                stack.enter_context(patch.object(start_system.rosgraph, 'is_master_online', return_value=False))
                stack.enter_context(patch.object(start_system.roslaunch.rlutil, 'get_or_generate_uuid', return_value='test'))
                stack.enter_context(patch.object(start_system.roslaunch, 'configure_logging'))
                launch = stack.enter_context(patch.object(start_system.roslaunch.parent, 'ROSLaunchParent', side_effect=[robot, task]))
                stack.enter_context(patch.object(start_system.rospy, 'init_node'))
                stack.enter_context(patch.object(start_system.rospy, 'loginfo'))
                stack.enter_context(patch.object(start_system.rospy, 'is_shutdown', return_value=False))
                stack.enter_context(patch.object(start_system.rospy, 'sleep', side_effect=KeyboardInterrupt))
                stack.enter_context(patch.object(start_system, 'wait_for_stack'))
                initialize = stack.enter_context(patch.object(start_system, 'initialize_gripper',
                    side_effect=RuntimeError('offline') if failure == 'initialize' else None))
                client = stack.enter_context(patch.object(start_system.actionlib, 'SimpleActionClient'))
                client.return_value.wait_for_server.return_value = True
                with patch('builtins.print'):
                    with self.assertRaises(RuntimeError if failure == 'initialize' else KeyboardInterrupt):
                        start_system.main(['--confirm-empty', '--no-rviz'])
                robot.shutdown.assert_called_once(); initialize.assert_called_once()
                self.assertEqual(launch.call_count, 1 if failure == 'initialize' else 2)
                if failure == 'ctrl_c':
                    task.shutdown.assert_called_once()
                    self.assertEqual(flock.call_args.args[1], start_system.fcntl.LOCK_UN)
                else:
                    task.start.assert_not_called()


if __name__ == '__main__': unittest.main()
