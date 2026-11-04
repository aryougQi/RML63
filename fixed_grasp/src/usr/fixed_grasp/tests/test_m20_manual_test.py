"""One user-entered place, isolated test heartbeat and correlated real-arm results."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import m20_manual_test as manual
    import server
    import rospy
    from fixed_grasp.msg import FixedGraspGoal
    from unittest.mock import Mock
    import threading
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class ManualM20Tests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.test = manual.ManualPlace(lambda: self.now)
        self.status = dict(schema_version=1, manual_test=True, boot_id='arm-test-boot', seq=1,
            sample_stamp=self.now, healthy=True, ready=True, arm_safe_to_move=True, next_grasp=1,
            active_request_id=None, transaction_id=None,
            manual_robot_state_ref=dict(boot_id=self.test.boot, seq=10),
            manual_authorized_transaction_id=self.test.tx)
        self.test.observe_status(self.status)

    def test_single_place_uses_acknowledged_reference_and_cannot_be_built_twice(self):
        self.test.arm_test()
        request = self.test.build_request()
        self.assertEqual(request['task_type'], 'place'); self.assertEqual(request['operation'], 'execute')
        self.assertEqual(request['payload'], {})
        self.assertEqual(request['robot_state_ref'], self.status['manual_robot_state_ref'])
        self.assertEqual(request['transaction_id'], self.test.tx)
        with self.assertRaises(RuntimeError): self.test.build_request()

    def test_normal_mode_stale_status_busy_arm_and_sequence_complete_are_not_authorized(self):
        self.test.observe_status(dict(self.status, manual_test=False))
        with self.assertRaisesRegex(RuntimeError, '手动测试模式'): self.test.ready()
        self.setUp(); self.now += .6
        self.assertFalse(self.test.ready())
        self.setUp(); self.test.observe_status(dict(self.status, seq=2, ready=False, active_request_id='other'))
        self.assertFalse(self.test.ready())
        self.setUp(); self.test.observe_status(dict(self.status, seq=2, next_grasp=None, ready=False))
        with self.assertRaisesRegex(RuntimeError, '两块'): self.test.ready()

    def test_server_must_acknowledge_this_heartbeat_boot_and_transaction(self):
        self.test.arm_test()
        self.test.observe_status(dict(self.status, seq=2, manual_authorized_transaction_id='another-tx'))
        self.assertFalse(self.test.authorized())
        with self.assertRaises(RuntimeError): self.test.build_request()
        self.test.observe_status(dict(self.status, seq=3, manual_robot_state_ref=dict(boot_id='other-peer', seq=10)))
        self.assertFalse(self.test.authorized())

    def test_cancel_ack_or_other_request_never_counts_as_original_place_success(self):
        self.test.arm_test(); request = self.test.build_request()
        feedback = dict(request, status='succeeded', task_success=True)
        self.assertFalse(self.test.feedback(dict(feedback, request_id='other')))
        self.assertFalse(self.test.feedback(dict(feedback, operation='cancel')))
        self.assertIsNone(self.test.result)
        self.assertTrue(self.test.feedback(feedback))
        self.assertTrue(self.test.result['task_success'])
        self.assertFalse(self.test.stowed())
        self.now += .1
        self.test.observe_status(dict(self.status, seq=2, sample_stamp=self.now,
                                     transaction_id=self.test.tx, next_grasp=2))
        self.assertTrue(self.test.stowed())

    def test_restart_during_command_invalidates_further_sends(self):
        self.test.arm_test()
        self.test.observe_status(dict(self.status, boot_id='new-arm-boot', seq=1))
        with self.assertRaisesRegex(RuntimeError, '重启'): self.test.build_request()

    def test_cancellation_targets_only_this_transaction_and_uses_new_request_id(self):
        self.test.arm_test(); original = self.test.build_request()
        cancel = self.test.cancel_request()
        self.assertEqual(cancel['target_request_id'], original['request_id'])
        self.assertEqual(cancel['transaction_id'], original['transaction_id'])
        self.assertNotEqual(cancel['request_id'], original['request_id'])
        self.assertEqual(cancel['operation'], 'cancel')

    def test_execute_flag_required_before_initializing_ros(self):
        with patch.object(manual.rospy, 'init_node') as node, patch('sys.stderr'):
            with self.assertRaises(SystemExit): manual.main(['place'])
        node.assert_not_called()

    def test_test_mode_allows_only_explicit_empty_home_manual_recovery(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = None; s.runner = None; s.needs_recovery = True
        s.m20_enabled = True; s.m20_test_mode = True
        handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=1)
        with patch.object(server.threading, 'Thread'):
            s.goal(handle, 'home', confirm_empty=True)
        handle.set_accepted.assert_called_once()


if __name__ == '__main__': unittest.main()
