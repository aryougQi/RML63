"""Offline ROS types and adapter gates; never initializes a ROS node/master."""
import io
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    import server
    from fixed_grasp.msg import FixedGraspGoal, FixedGraspResult
    from core import Runner
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source workspace and built fixed_grasp local_setup.bash')
class ContractTests(unittest.TestCase):
    def model(self):
        b = backend.Backend.__new__(backend.Backend)
        b.config = backend.load_settings(Path(__file__).resolve().parents[1]/'config/fixed.yaml')
        b.config['release_mode'] = 'support_service'
        b.config['scene_mode'] = 'measured'
        b.check = lambda: None
        b.allow = False
        b.poses = {n: np.eye(4) for n in ['Home','Pregrasp','Grasp','Lift','Preplace','Place','Retreat']}
        b.poses['Lift'][2,3] = .10
        b.config['basket']['boxes'] = [dict(size_m=[.1,.1,.1], pose_tcp=np.eye(4).tolist())]
        b.scene_ready = False
        b.build_scene = Mock()
        b.s = Mock()
        return b

    def test_execute_gate_precedes_scene_and_hardware(self):
        b = self.model()
        with self.assertRaisesRegex(RuntimeError, 'EXECUTE disabled'): b.preflight(True, False)
        b.build_scene.assert_not_called(); b.s.verify_hardware.assert_not_called()

    def test_no_basket_cannot_report_plan(self):
        b = self.model(); b.config['basket']['boxes'] = []
        with self.assertRaisesRegex(RuntimeError, 'collision boxes'): b.preflight(False, False)
        b.build_scene.assert_not_called()

    def test_lift_has_no_extra_height_cap(self):
        for height in (.11, .5, 1.):
            b = self.model(); b.poses['Lift'][2,3] = height
            b.preflight(False, False)
            b.build_scene.assert_called_once()
            b.s.verify_hardware.assert_not_called()

    def test_invalid_lift_positions_rejected(self):
        for height in (0., float('nan'), float('inf')):
            b = self.model(); b.poses['Lift'][2,3] = height
            with self.assertRaisesRegex(ValueError, 'Lift'): b.preflight(False, False)
            b.build_scene.assert_not_called()

    def test_downward_or_lateral_lift_rejected(self):
        for xyz in ([0., 0., -.10], [.01, 0., .10]):
            b = self.model(); b.poses['Lift'][:3, 3] = xyz
            with self.assertRaisesRegex(ValueError, 'Lift'): b.preflight(False, False)
            b.build_scene.assert_not_called()

    def test_home_only_does_not_check_other_motion_stages(self):
        b = self.model()
        b.poses = {name: b.poses[name] for name in ('Home', 'Grasp')}
        b.preflight(False, False, home_only=True)
        b.build_scene.assert_called_once()
        b.s.verify_hardware.assert_not_called()

    def test_robot_only_planning_needs_no_external_geometry(self):
        b = self.model(); b.config['scene_mode'] = 'robot_only'; b.config['basket']['boxes'] = []
        with patch.object(backend, 'environment_geometry') as environment:
            b.preflight(False, False)
        environment.assert_not_called(); b.s.verify_hardware.assert_not_called()
        b.build_scene.assert_called_once()

    def test_robot_only_execute_keeps_hardware_and_open_checks(self):
        b = self.model(); b.config['scene_mode'] = 'robot_only'; b.config['basket']['boxes'] = []
        b.allow = True; b.require_open = Mock()
        with patch.object(backend, 'environment_geometry') as environment, patch.object(backend.rospy, 'logwarn'):
            b.preflight(True, False)
        environment.assert_not_called()
        b.s.verify_hardware.assert_called_once(); b.require_open.assert_called_once()
        b.s.require_environment_verified.assert_called_once()

    def test_robot_only_full_transfer_still_requires_support(self):
        b = self.model(); b.config['scene_mode'] = 'robot_only'; b.config['basket']['boxes'] = []
        b.allow = True
        with self.assertRaisesRegex(RuntimeError, 'support verification'): b.preflight(True, True)
        b.s.verify_hardware.assert_not_called(); b.build_scene.assert_not_called()

    def test_robot_only_holding_is_logical_and_keeps_monitoring(self):
        b = self.model(); b.config['scene_mode'] = 'robot_only'
        b.set_basket(True); self.assertTrue(b.attached)
        b.set_basket(False); self.assertFalse(b.attached)
        b.s.apply.assert_not_called()

    def test_robot_only_execution_permission_still_required(self):
        session = backend.Session.__new__(backend.Session)
        session.config = dict(scene_mode='robot_only')
        with patch.object(backend.RobotSession, 'verify_hardware') as hardware, patch.object(backend.rospy, 'get_param', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'execution disabled'): session.verify_hardware()
        hardware.assert_called_once_with(require_execution=False)

    def test_robot_only_requires_tool_scene_to_execute(self):
        session = backend.Session.__new__(backend.Session)
        session.config = dict(scene_mode='robot_only')
        with self.assertRaisesRegex(RuntimeError, 'not been built'): session.require_environment_verified()
        session.robot_scene_ready = True
        session.require_environment_verified()

    def test_backend_loads_derived_targets_without_fake_joints(self):
        from waypoint_offsets import derive_from_grasp
        measured = dict(joints_rad=[0.]*6, tcp_base=np.eye(4).tolist())
        data = derive_from_grasp(dict(schema=1, identity={'test': True}, waypoints=dict(Home=measured, Grasp=measured)))
        session = Mock(); session.active = ['j%d'%i for i in range(6)]; session.offsets = np.zeros(6)
        session.chain.fk.return_value = np.eye(4)
        geom = Mock(); geom.at_width.return_value = {'flange_tcp': np.eye(4)}
        scene = SimpleNamespace(robot_state=SimpleNamespace(attached_collision_objects=[]))
        proxy = Mock(return_value=SimpleNamespace(scene=scene))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.yaml'; path.write_text(yaml.safe_dump(data))
            config = self.model().config; config['scene_mode'] = 'robot_only'
            with patch.object(backend, 'Session', return_value=session), patch.object(backend, 'geometry', return_value=geom), patch.object(backend, 'identity', return_value={'test': True}), patch.object(backend, 'Gripper'), patch.object(backend.rospy, 'ServiceProxy', return_value=proxy):
                b = backend.Backend(config, path, False, lambda: None, 'offline')
            expected = np.eye(4); expected[2,3] = .15
            np.testing.assert_array_equal(b.poses['Pregrasp'], expected)
            np.testing.assert_array_equal(b.poses['Lift'], expected)
            self.assertNotIn('joints_rad', b.data['waypoints']['Pregrasp'])

    def test_rotation_rejected(self):
        b = self.model(); b.poses['Lift'][:3,:3] = np.diag([-1,-1,1])
        with self.assertRaisesRegex(ValueError, 'orientation'): b.preflight(False, False)

    def test_threshold_override_rejected(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as f:
            f.write('max_joint_jump_deg: 180\n'); f.flush()
            with self.assertRaisesRegex(ValueError, 'thresholds'): backend.load_settings(f.name)

    def test_preview_never_queries_hardware(self):
        b = self.model(); b.preflight(False, False)
        b.build_scene.assert_called_once(); b.s.verify_hardware.assert_not_called()

    def test_full_execute_requires_support_before_scene(self):
        b = self.model(); b.allow = True
        b.config['basket'].update(verified=True, source='measured')
        with patch.object(backend, 'environment_geometry', return_value={}):
            with self.assertRaisesRegex(RuntimeError, 'support verification'): b.preflight(True, True)
        b.build_scene.assert_not_called(); b.s.verify_hardware.assert_not_called()

    def test_action_wire_roundtrip(self):
        goal = FixedGraspGoal(mode=FixedGraspGoal.PLAN_ONLY, full_transfer=True)
        buf = io.BytesIO(); goal.serialize(buf)
        decoded = FixedGraspGoal().deserialize(buf.getvalue())
        self.assertEqual(decoded, goal)
        result = FixedGraspResult(success=True, planning_success=True)
        buf = io.BytesIO(); result.serialize(buf)
        self.assertFalse(FixedGraspResult().deserialize(buf.getvalue()).execution_success)

    def test_busy_goal_is_rejected(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = Mock(); s.needs_recovery = False
        s.runner = None
        handle = Mock(); s.goal(handle)
        handle.set_rejected.assert_called_once(); handle.set_accepted.assert_not_called()

    def test_second_action_dispatches_selection_and_shares_busy_and_cancel_locks(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = None; s.needs_recovery = False; s.backend = Mock()
        handle = Mock()
        handle.get_goal.return_value = FixedGraspGoal(mode=0, full_transfer=True)
        handle.get_goal_id.return_value = SimpleNamespace(id='same-id')
        with patch.object(server, 'load_task_waypoints'), patch.object(server.rospy, 'get_param'), \
                patch.object(server.threading, 'Thread') as worker:
            s.goal(handle, 2)
        self.assertEqual(worker.call_args.kwargs['args'][2], 2)
        handle.set_accepted.assert_called_once()
        other = Mock()
        s.goal(other, 1)
        other.set_rejected.assert_called_once()
        s.cancel(handle, 1)
        self.assertFalse(s.runner.cancel.is_set())
        s.cancel(handle, 2)
        self.assertTrue(s.runner.cancel.is_set())
        s.backend.stop.assert_called_once()

    def test_untaught_second_grasp_rejected_without_locking_first(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = None; s.needs_recovery = False
        handle = Mock()
        handle.get_goal.return_value = FixedGraspGoal(mode=1, full_transfer=True)
        with patch.object(server, 'load_task_waypoints', side_effect=ValueError('second grasp not taught')), \
                patch.object(server.rospy, 'get_param'), patch.object(server.threading, 'Thread') as worker:
            s.goal(handle, 2)
        handle.set_rejected.assert_called_once()
        handle.set_accepted.assert_not_called()
        worker.assert_not_called()
        self.assertIsNone(s.active)
        self.assertFalse(s.needs_recovery)

    def test_selected_second_grasp_reaches_backend_fk_validation_and_uses_shared_place(self):
        from grasp_profiles import new_grasp_profile, read_shared
        from waypoint_offsets import derive_from_grasp
        original = dict(joints_rad=[0.]*6, tcp_base=np.eye(4).tolist())
        shared = derive_from_grasp(dict(schema=1, identity={'model': 'same'}, waypoints={
            name: original for name in ('Home', 'Grasp', 'Preplace', 'Place', 'Retreat')}))
        pose2 = np.eye(4); pose2[0, 3] = .2
        session = Mock(); session.active = ['j%d'%i for i in range(6)]; session.offsets = np.zeros(6)
        session.chain.fk.side_effect = lambda q, offsets: pose2 if np.allclose(q, .1) else np.eye(4)
        geom = Mock(); geom.at_width.return_value = {'flange_tcp': np.eye(4)}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.yaml'; path.write_text(yaml.safe_dump(shared))
            _, digest = read_shared(path)
            profile = new_grasp_profile(shared, digest)
            profile['waypoints']['Grasp'] = dict(joints_rad=[.1]*6, tcp_base=pose2.tolist())
            profile = derive_from_grasp(profile)
            second = Path(directory)/'waypoints_2.yaml'; second.write_text(yaml.safe_dump(profile))
            config = self.model().config; config['scene_mode'] = 'existing'
            with patch.object(backend, 'Session', return_value=session), patch.object(backend, 'geometry', return_value=geom), \
                    patch.object(backend, 'identity', return_value={'model': 'same'}), patch.object(backend, 'Gripper'):
                first = backend.Backend(config, path, False, lambda: None, 'first')
                second_grasp = backend.Backend(config, path, False, lambda: None, 'second', grasp_waypoints=second)
        np.testing.assert_array_equal(first.poses['Grasp'], np.eye(4))
        np.testing.assert_array_equal(second_grasp.poses['Grasp'], pose2)
        for name in ('Home', 'Preplace', 'Place', 'Retreat'):
            np.testing.assert_array_equal(first.poses[name], second_grasp.poses[name])
        self.assertEqual(first.waypoints_sha256, second_grasp.waypoints_sha256)

    def test_client_selects_correct_action_without_changing_goal_wire_format(self):
        import client as grasp_client
        for grasp in (1, 2):
            with self.subTest(grasp=grasp):
                argv = ['client.py', '--execute', '--full-transfer']
                if grasp == 2: argv += ['--grasp', '2']
                with patch.object(grasp_client.rospy, 'myargv', return_value=argv), \
                        patch.object(grasp_client.rospy, 'init_node'), patch('builtins.print'), \
                        patch.object(grasp_client.actionlib, 'SimpleActionClient') as factory:
                    factory.return_value.wait_for_result.return_value = True
                    grasp_client.main()
                expected = '/fixed_grasp/task' if grasp == 1 else '/fixed_grasp/task_2'
                self.assertEqual(factory.call_args.args[0], expected)
                goal = factory.return_value.send_goal.call_args.args[0]
                self.assertEqual(goal.mode, 1)
                self.assertTrue(goal.full_transfer)

    def test_home_client_uses_shared_server_home_endpoint(self):
        import client as grasp_client
        with patch.object(grasp_client.rospy, 'myargv', return_value=['client.py', '--home', '--execute']), \
                patch.object(grasp_client.rospy, 'init_node'), patch('builtins.print'), \
                patch.object(grasp_client.actionlib, 'SimpleActionClient') as factory:
            factory.return_value.wait_for_result.return_value = True
            grasp_client.main()
        self.assertEqual(factory.call_args.args[0], '/fixed_grasp/home')
        goal = factory.return_value.send_goal.call_args.args[0]
        self.assertEqual(goal.mode, 1); self.assertFalse(goal.full_transfer)

    def test_home_client_rejects_grasp_and_full_transfer_options(self):
        import client as grasp_client
        for options in (['--grasp', '2'], ['--full-transfer']):
            with self.subTest(options=options), patch.object(grasp_client.rospy, 'myargv',
                    return_value=['client.py', '--home']+options), patch.object(grasp_client.rospy, 'init_node') as node, \
                    patch('sys.stderr', io.StringIO()):
                with self.assertRaises(SystemExit): grasp_client.main()
                node.assert_not_called()

    def test_home_rejected_while_busy_or_recovery_required(self):
        for active, recovery in ((Mock(), False), (None, True)):
            s = server.Server.__new__(server.Server)
            s.lock = threading.RLock(); s.active = active; s.needs_recovery = recovery
            s.runner = None
            handle = Mock()
            s.goal(handle, 'home')
            handle.set_rejected.assert_called_once(); handle.set_accepted.assert_not_called()

    def test_rejected_home_distinguishes_busy_from_previous_failure(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = None; s.needs_recovery = True
        s.recovery_stage = 'CLOSE'; s.recovery_reason = 'force contact not verified'
        handle = Mock(); s.goal(handle, 'home')
        result = handle.set_rejected.call_args.args[0]
        self.assertEqual(result.stage, 'CLOSE')
        self.assertIn('RECOVERY_REQUIRED', result.reason)
        self.assertIn(s.recovery_reason, result.reason)
        s.active = Mock(); s.runner = Runner(None); s.runner.stage = 'Lift'
        handle = Mock(); s.goal(handle, 'home')
        result = handle.set_rejected.call_args.args[0]
        self.assertEqual(result.stage, 'Lift')
        self.assertTrue(result.reason.startswith('BUSY:'))

    def test_home_retry_and_recovery_follow_actual_command_attempts(self):
        for failure in ('settings', 'constructor', 'preflight', 'plan', 'check_open',
                        'motion', 'gripper', 'cancel_before_dispatch', 'cancel_after_dispatch', 'scene_restore'):
            with self.subTest(failure=failure):
                s = server.Server.__new__(server.Server)
                s.lock = threading.RLock(); s.active = Mock(); s.backend = None; s.needs_recovery = False
                handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=1)
                runner = Runner(None)
                b = Mock(); b.config = {'scene_mode': 'existing'}
                b.plan.return_value = {'Home': 'trajectory'}
                error = RuntimeError('test failure: '+failure)
                if failure in ('preflight', 'plan'):
                    getattr(b, failure).side_effect = error
                elif failure == 'check_open':
                    b.require_home_empty.side_effect = error
                elif failure in ('motion', 'gripper'):
                    # Dispatch happens inside Backend methods after their checks.
                    transport = b.s.motion_client.send_goal if failure == 'motion' else b.gripper.position_pub.publish
                    transport.side_effect = error
                    b.move.side_effect = lambda *args: (
                        b.s.motion_client.send_goal('trajectory') if failure == 'motion'
                        else b.gripper.position_pub.publish('position'))
                elif failure == 'scene_restore':
                    b.plan.side_effect = error
                    b.restore_preview.side_effect = RuntimeError('restore failed')
                elif failure.startswith('cancel_'):
                    def cancel_during_move(*args):
                        if failure == 'cancel_after_dispatch':
                            b.s.motion_client.send_goal('trajectory')
                        runner.cancel.set()
                        b.s.motion_client.send_goal('blocked by cancellation')
                    b.move.side_effect = cancel_during_move
                with patch.object(server, 'load_settings', return_value=b.config,
                                  side_effect=error if failure == 'settings' else None), \
                        patch.object(server, 'Backend', return_value=b,
                                     side_effect=error if failure == 'constructor' else None), \
                        patch.object(server.rospy, 'get_param'), patch.object(server.rospy, 'logwarn'):
                    s.work(handle, runner, 'home')
                if failure.startswith('cancel_'):
                    handle.set_canceled.assert_called_once()
                else:
                    handle.set_aborted.assert_called_once()
                self.assertIsNone(s.active)
                self.assertEqual(s.needs_recovery, failure in ('motion', 'gripper', 'cancel_after_dispatch', 'scene_restore'))
                if failure not in ('settings', 'constructor', 'motion', 'gripper', 'cancel_after_dispatch'):
                    b.restore_preview.assert_called_once()
                if not s.needs_recovery:
                    retry = Mock(); retry.get_goal.return_value = FixedGraspGoal(mode=1)
                    with patch.object(server.threading, 'Thread'):
                        s.goal(retry, 'home')
                    retry.set_accepted.assert_called_once()
                else:
                    retry = Mock(); s.goal(retry, 'home')
                    self.assertIn(s.recovery_reason, retry.set_rejected.call_args.args[0].reason)

    def test_successful_grasp_held_at_lift_still_blocks_home(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = Mock(); s.backend = None; s.needs_recovery = False
        handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=1)
        runner = Runner(None)
        b = Mock(); b.config = {'scene_mode': 'existing'}
        b.plan.return_value = dict.fromkeys(['Home', 'Pregrasp', 'Grasp', 'Lift'])
        b.move.side_effect = lambda *args: b.s.motion_client.send_goal('trajectory')
        with patch.object(server, 'load_settings', return_value=b.config), \
                patch.object(server, 'Backend', return_value=b), patch.object(server.rospy, 'get_param'):
            s.work(handle, runner)
        handle.set_succeeded.assert_called_once()
        self.assertTrue(s.needs_recovery)
        retry = Mock(); s.goal(retry, 'home')
        self.assertIn('basket held at Lift', retry.set_rejected.call_args.args[0].reason)

    def test_server_dispatches_home_without_grasp_or_secondary_teaching(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = Mock(); s.backend = None; s.needs_recovery = False
        handle = Mock(); handle.get_goal.return_value = FixedGraspGoal(mode=1)
        runner = Mock(planned=True, executed=True, grasp_verified=False, holding=False, stage='DONE')
        runner.cancel.is_set.return_value = False
        b = Mock(); b.config = {'scene_mode': 'existing'}
        with patch.object(server, 'load_settings', return_value=b.config), patch.object(server, 'Backend', return_value=b) as factory, \
                patch.object(server.rospy, 'get_param', side_effect=lambda name, default=None:
                    {'~config': 'config', '~waypoints': 'shared', '~allow_execution': True}[name]):
            s.work(handle, runner, 'home')
        runner.run_home.assert_called_once_with(1); runner.run.assert_not_called()
        self.assertIsNone(factory.call_args.kwargs['grasp_waypoints'])
        result = handle.set_succeeded.call_args.args[0]
        self.assertTrue(result.success); self.assertTrue(result.execution_success)
        self.assertFalse(result.grasp_verified)
        self.assertFalse(s.needs_recovery)

    def test_cancel_only_matching_goal(self):
        s = server.Server.__new__(server.Server)
        s.lock = threading.RLock(); s.active = Mock(); s.backend = Mock(); s.runner = Runner(None)
        s.active.get_goal_id.return_value = SimpleNamespace(id='owned')
        handle = Mock(); handle.get_goal_id.return_value = SimpleNamespace(id='other')
        s.cancel(handle); self.assertFalse(s.runner.cancel.is_set()); s.backend.stop.assert_not_called()
        handle.get_goal_id.return_value = SimpleNamespace(id='owned')
        s.cancel(handle); self.assertTrue(s.runner.cancel.is_set()); s.backend.stop.assert_called_once()


if __name__ == '__main__': unittest.main()
