"""Existing-scene mode must not build, remove, or replace collision geometry."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    from moveit_msgs.msg import RobotState, RobotTrajectory, CollisionObject, AttachedCollisionObject, Constraints, JointConstraint
    from trajectory_msgs.msg import JointTrajectoryPoint
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class ExistingSceneTests(unittest.TestCase):
    def model(self):
        b = backend.Backend.__new__(backend.Backend)
        b.config = backend.load_settings(Path(__file__).resolve().parents[1]/'config/fixed.yaml')
        b.config['release_mode'] = 'support_service'
        b.config['scene_mode'] = 'existing'; b.config['basket']['boxes'] = []
        b.check = Mock(); b.s = Mock(); b.allow = True
        b.poses = {n: np.eye(4) for n in ['Home','Pregrasp','Grasp','Lift','Preplace','Place','Retreat']}
        b.poses['Lift'][2,3] = .15
        b.scene_ready = False; b.scene_touched = False
        b.current_scene = Mock(); b.build_scene = Mock(); b.require_open = Mock()
        return b

    def test_preflight_and_cleanup_do_not_access_scene_services(self):
        b = self.model()
        b.preflight(False, False)
        b.set_basket(True); self.assertTrue(b.attached)
        b.set_basket(False); self.assertFalse(b.attached)
        b.restore_preview()
        b.build_scene.assert_not_called(); b.current_scene.assert_not_called(); b.s.apply.assert_not_called()

    def test_execution_keeps_hardware_open_and_permission_checks(self):
        b = self.model()
        with patch.object(backend.rospy, 'logwarn'):
            b.preflight(True, False)
        b.s.verify_hardware.assert_called_once(); b.require_open.assert_called_once()
        b.build_scene.assert_not_called(); b.current_scene.assert_not_called()
        b.allow = False
        with self.assertRaisesRegex(RuntimeError, 'EXECUTE disabled'): b.preflight(True, False)

    def test_full_transfer_still_requires_support(self):
        b = self.model()
        with self.assertRaisesRegex(RuntimeError, 'support verification'): b.preflight(True, True)
        b.s.verify_hardware.assert_not_called()

    def test_initial_aperture_is_unrestricted_in_existing_scene(self):
        b = self.model(); b.gripper = Mock()
        for position in (0, 230, 643, 714, 786, 1000):
            with self.subTest(position=position):
                b.gripper.reset_mock()
                b.gripper.query.return_value = dict(status=1, enable=1, error=0,
                                                    mode=2, actpos=position, current_force=0)
                with patch.object(backend.time, 'sleep'):
                    backend.Backend.require_open(b)
                self.assertEqual(b.gripper.query.call_count, 3)
                b.gripper.open.assert_not_called(); b.gripper.close.assert_not_called()

    def test_unrestricted_aperture_keeps_feedback_gates(self):
        b = self.model(); b.gripper = Mock()
        for bad in (dict(status=0), dict(enable=0), dict(error=1), dict(mode=6),
                    dict(current_force=50), dict(current_force=-50)):
            with self.subTest(bad=bad):
                state = dict(status=1, enable=1, error=0, mode=2, actpos=714, current_force=0)
                state.update(bad); b.gripper.query.return_value = state
                with self.assertRaisesRegex(RuntimeError, 'gripper preflight failed'):
                    backend.Backend.require_open(b)

    def test_generated_envelope_modes_keep_aperture_check(self):
        b = self.model(); b.gripper = Mock()
        b.gripper.query.return_value = dict(status=1, enable=1, error=0, mode=2,
                                            actpos=1000, current_force=0)
        for mode in ('robot_only', 'measured'):
            b.config['scene_mode'] = mode
            with self.assertRaisesRegex(RuntimeError, 'gripper preflight failed'):
                backend.Backend.require_open(b)

    def test_state_diff_inherits_existing_attached_objects(self):
        session = backend.Session.__new__(backend.Session); session.config = dict(scene_mode='existing')
        state = RobotState()
        state.attached_collision_objects = [AttachedCollisionObject(object=CollisionObject(id='user_tool'))]
        with patch.object(backend.RobotSession, 'state_for', return_value=state):
            result = session.state_for(np.zeros(6))
        self.assertTrue(result.is_diff)
        self.assertEqual(result.attached_collision_objects, [])

    def test_prepare_uses_configured_width_after_feedback_check(self):
        b = self.model(); b.gripper = Mock()
        b.prepare_gripper()
        b.require_open.assert_called_once()
        b.gripper.open.assert_called_once_with(.050)
        b.gripper.reset_mock()
        b.require_open.side_effect = RuntimeError('holding or unknown')
        with self.assertRaisesRegex(RuntimeError, 'holding or unknown'):
            b.prepare_gripper()
        b.gripper.open.assert_not_called()

    def test_state_advance_keeps_diff_semantics(self):
        state = RobotState(is_diff=True)
        state.joint_state.name = ['j1']; state.joint_state.position = [0.]
        traj = RobotTrajectory(); traj.joint_trajectory.joint_names = ['j1']
        traj.joint_trajectory.points = [JointTrajectoryPoint(positions=[.1])]
        result = backend.state_after_shared_geometry(state, traj)
        self.assertTrue(result.is_diff)
        self.assertEqual(result.joint_state.position, [.1])

    def test_existing_mode_still_calls_original_collision_checker(self):
        session = backend.Session.__new__(backend.Session)
        session.config = dict(scene_mode='existing'); session.active = ['j1']; session.offsets = np.zeros(1)
        session.robot = Mock(); session.robot.get_joint.return_value.bounds.return_value = (-1.,1.)
        with patch.object(backend.RobotSession, 'check_state') as check:
            session.check_state(np.array([.1]))
            check.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, 'joint limit'): session.check_state(np.array([2.]))

    def test_planner_preserves_existing_constraints_without_adding_travel_cap(self):
        session = backend.Session.__new__(backend.Session)
        session.config = dict(max_joint_travel_deg=90)
        session.cancel_check = lambda: None
        session.active = ['j1']; session.robot = Mock(); session.group = Mock()
        session.robot.get_joint.return_value.bounds.return_value = (-3., 3.)
        original = Constraints(name='existing', joint_constraints=[JointConstraint(joint_name='other')])
        session.group.get_path_constraints.return_value = original
        state = RobotState(); state.joint_state.name = ['j1']; state.joint_state.position = [0.]
        session.group.plan.side_effect = RuntimeError('simulated planning failure')
        with self.assertRaisesRegex(RuntimeError, 'simulated planning failure'):
            session.plan_joint(np.array([.1]), state)
        session.group.set_path_constraints.assert_not_called()
        self.assertEqual(len(original.joint_constraints), 1)

    def test_target_beyond_90_degrees_reaches_original_planner(self):
        session = backend.Session.__new__(backend.Session)
        session.config = dict(max_joint_travel_deg=90)
        session.cancel_check = lambda: None
        session.active = ['j1']; session.group = Mock()
        session.group.get_path_constraints.return_value = Constraints()
        state = RobotState(); state.joint_state.name = ['j1']; state.joint_state.position = [0.]
        trajectory = Mock()
        session.group.plan.return_value = (True, trajectory)
        session.dense_timed = Mock(return_value=trajectory)
        session.check_trajectory = Mock()
        self.assertIs(session.plan_joint(np.array([2.]), state), trajectory)
        session.group.set_joint_value_target.assert_called_once_with({'j1': 2.})
        session.check_trajectory.assert_called_once_with(trajectory, state)
        session.group.set_path_constraints.assert_not_called()


if __name__ == '__main__': unittest.main()
