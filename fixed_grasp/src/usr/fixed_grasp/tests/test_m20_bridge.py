"""Measured arm stowage, local dispatch guards, and M20 wire/result adaptation."""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import rospy
    import server
    from core import Runner, Cancelled
    from m20_bridge import ArmObserver, LocalGoal, M20Bridge
    from fixed_grasp.msg import FixedGraspGoal
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class M20BridgeTests(unittest.TestCase):
    def observer(self):
        names = ['joint%d' % i for i in range(1, 7)]
        observer = ArmObserver(dict(joint_feedback_max_age_sec=.3, max_capture_motion_deg=.1),
            dict(identity=dict(joint_names=names), waypoints=dict(Home=dict(joints_rad=[0.]*6))))
        for stamp in (100., 100.05, 100.1, 100.15, 100.2):
            message = SimpleNamespace(header=SimpleNamespace(stamp=rospy.Time.from_sec(stamp)), name=names, position=[0.]*6)
            observer.joint(message, stamp)
        observer.fault(SimpleNamespace(joint=[0]*6), 100.2)
        observer.enable(SimpleNamespace(joint_en_flag=[True]*6), 100.2)
        for stamp in (100., 100.1, 100.2):
            observer.gripper(dict(status=1, enable=1, error=0, mode=3, current_force=0), stamp)
        return observer

    def test_only_actual_fresh_stationary_empty_home_is_safe_to_move(self):
        observer = self.observer()
        good = observer.snapshot(100.2, False, False, True)
        self.assertTrue(good['healthy']); self.assertTrue(good['ready']); self.assertTrue(good['arm_safe_to_move'])
        self.assertFalse(observer.snapshot(100.2, True, False, True)['arm_safe_to_move'])
        self.assertFalse(observer.snapshot(100.2, False, True, True)['ready'])
        self.assertFalse(observer.snapshot(100.2, False, False, False)['healthy'])
        self.assertFalse(observer.snapshot(100.8, False, False, True)['arm_safe_to_move'])
        observer.joints[-1][1][0] = np.radians(1.)
        self.assertFalse(observer.snapshot(100.2, False, False, True)['arm_safe_to_move'])

    def test_holding_remains_healthy_but_never_reports_stowed_empty_arm(self):
        observer = self.observer()
        observer.gripper(dict(status=1, enable=1, error=0, mode=6, current_force=500), 100.2)
        status = observer.snapshot(100.2, False, False, True)
        self.assertTrue(status['healthy']); self.assertFalse(status['ready']); self.assertFalse(status['arm_safe_to_move'])
        observer.gripper(None, 100.2)
        self.assertFalse(observer.snapshot(100.2, False, False, True)['healthy'])

    def test_continuous_status_stays_ready_with_10hz_gripper_and_20hz_joint_samples(self):
        observer = self.observer()
        for number in range(1, 31):
            now = 100.2+number*.05
            observer.joint(SimpleNamespace(header=SimpleNamespace(stamp=rospy.Time.from_sec(now)),
                name=observer.names, position=[0.]*6), now)
            observer.fault(SimpleNamespace(joint=[0]*6), now)
            observer.enable(SimpleNamespace(joint_en_flag=[True]*6), now)
            if number % 2 == 0:
                observer.gripper(dict(status=1, enable=1, error=0, mode=3, current_force=0), now)
            self.assertTrue(observer.snapshot(now, False, False, True)['ready'], str(number))

    def test_duplicate_joint_timestamps_faults_and_foreign_publishers_invalidate_stowage(self):
        observer = self.observer()
        message = SimpleNamespace(header=SimpleNamespace(stamp=rospy.Time.from_sec(100.2)),
                                  name=observer.names, position=[0.]*6)
        observer.joint(message, 100.2)
        self.assertFalse(observer.snapshot(100.2, False, False, True)['ready'])
        observer = self.observer(); observer.fault(SimpleNamespace(joint=[1, 0, 0, 0, 0, 0]), 100.2)
        self.assertFalse(observer.snapshot(100.2, False, False, True)['healthy'])
        bridge = M20Bridge.__new__(M20Bridge); bridge.observer = self.observer(); bridge.clock = lambda: 100.2
        message._connection_header = {'callerid': '/fake_joint_publisher'}
        bridge.hardware('joint', message)
        self.assertFalse(bridge.observer.snapshot(100.2, False, False, True)['healthy'])

    def test_local_goal_always_requests_full_transfer_and_checks_all_success_fields(self):
        for bad_field in (None, 'success', 'planning_success', 'execution_success', 'grasp_verified', 'stage'):
            with self.subTest(bad_field=bad_field):
                bridge = Mock()
                record = dict(tx='tx-1', block=1, request=json.dumps({'request_id': 'req-1'}))
                handle = LocalGoal(bridge, record)
                result = SimpleNamespace(success=True, planning_success=True, execution_success=True,
                                         grasp_verified=True, stage='DONE', reason='completed')
                if bad_field: setattr(result, bad_field, 'Lift' if bad_field == 'stage' else False)
                handle.set_succeeded(result)
                self.assertEqual(handle.get_goal().mode, 1); self.assertTrue(handle.get_goal().full_transfer)
                self.assertEqual(bridge.engine.finish.call_args.args[1], bad_field is None)
                self.assertEqual(handle.get_goal_id().id, 'm20:tx-1:req-1')

    def test_guard_loss_between_plan_and_dispatch_never_sends_arm_or_gripper_commands(self):
        b = Mock(); b.plan.return_value = dict.fromkeys(['Home', 'Pregrasp', 'Grasp', 'Lift'])
        allowed = [True]
        def guard():
            if not allowed[0]: raise Cancelled('M20 task authorization lost')
        def plan(*args):
            allowed[0] = False
            return dict.fromkeys(['Home', 'Pregrasp', 'Grasp', 'Lift'])
        b.plan.side_effect = plan
        runner = Runner(b, guard=guard)
        with self.assertRaises(Cancelled): runner.run(1)
        b.move.assert_not_called(); b.prepare_gripper.assert_not_called(); b.close.assert_not_called()

    def test_server_guard_is_checked_at_the_serialized_transport_send(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = Mock(); s.backend = None; s.needs_recovery = False
        handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=1)
        allowed = [True]
        def guard():
            if not allowed[0]: raise Cancelled('authorization lost at dispatch')
        runner = Runner(None, guard=guard)
        b = Mock(); b.config = {'scene_mode': 'existing'}; b.plan.return_value = {'Home': 'trajectory'}
        transport = b.s.motion_client.send_goal
        def move(*args):
            allowed[0] = False
            b.s.motion_client.send_goal('must not be sent')
        b.move.side_effect = move
        with patch.object(server, 'load_settings', return_value=b.config), patch.object(server, 'Backend', return_value=b), \
                patch.object(server.rospy, 'get_param'), patch.object(server.rospy, 'logwarn'):
            s.work(handle, runner, 'home')
        transport.assert_not_called(); handle.set_canceled.assert_called_once()

    def test_m20_mode_rejects_uncoordinated_manual_execution_but_preserves_preview(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = None; s.runner = None; s.needs_recovery = False; s.m20_enabled = True
        handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=1)
        s.goal(handle)
        self.assertIn('M20_MODE', handle.set_rejected.call_args.args[0].reason)
        preview = Mock(); preview.get_goal.return_value = FixedGraspGoal(mode=0)
        with patch.object(server.threading, 'Thread'):
            s.goal(preview)
        preview.set_accepted.assert_called_once()


if __name__ == '__main__': unittest.main()
