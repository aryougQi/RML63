"""Compiled calibrated IK and planning retries; no ROS nodes or controllers."""
import gc
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    import xacro
    from fast_ik import CalibratedIK
    from kinematics_config import load_config
    from core import Cancelled
    from planner_metrics import CollisionFailure
    from moveit_msgs.msg import RobotState
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace with python3-pykdl')
class CalibratedIKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[1]
        cls.config = backend.load_settings(cls.root/'config/fixed.yaml')
        path = Path(backend.PACK.get_path('rml63_tools_description'))/'urdf/rml63_with_gripper.urdf.xacro'
        cls.xml = xacro.process_file(str(path), mappings={'fixed_gripper': 'true'}).toxml()
        _, cls.chain, cls.offsets, _ = load_config(cls.config['calibration_file'], cls.xml)
        cls.names = ['joint%d' % i for i in range(1, 7)]
        joints = {j.get('name'): j for j in ET.fromstring(cls.xml).findall('joint')}
        cls.bounds = [(float(joints[n].find('limit').get('lower')),
                       float(joints[n].find('limit').get('upper'))) for n in cls.names]
        cls.data = yaml.safe_load((cls.root/'config/waypoints.yaml').read_text())

    def session(self):
        s = backend.Session.__new__(backend.Session)
        s.config = self.config.copy(); s.chain = self.chain; s.offsets = self.offsets
        s.active = self.names; s.cancel_check = Mock(); s.check_state = Mock()
        s.robot = Mock()
        s.robot.get_joint.side_effect = lambda n: Mock(bounds=Mock(return_value=self.bounds[self.names.index(n)]))
        return s

    def test_compiled_fk_matches_existing_calibrated_model_at_many_joint_states(self):
        solver = CalibratedIK(self.chain, self.offsets, self.bounds)
        for q in np.random.RandomState(63).uniform(-1., 1., (20, 6)):
            np.testing.assert_allclose(solver.fk(q), self.chain.fk(q, self.offsets), atol=1e-12)

    def test_solver_dependencies_survive_gc_and_solutions_respect_both_joint_coordinates(self):
        solver = CalibratedIK(self.chain, self.offsets, self.bounds)
        gc.collect()
        for name in ('Home', 'Grasp', 'Preplace', 'Place', 'Retreat'):
            q = np.asarray(self.data['waypoints'][name]['joints_rad'])
            target = self.chain.fk(q, self.offsets)
            answers = list(solver.solutions(target, q+np.array([.02, -.02, .03, -.02, .02, 0.])))
            self.assertTrue(answers)
            for answer in answers:
                np.testing.assert_allclose(self.chain.fk(answer, self.offsets), target, atol=1e-5)
                self.assertTrue(np.all(answer >= solver.lower)); self.assertTrue(np.all(answer <= solver.upper))

    def test_nonfinite_seed_and_pose_never_reach_native_solver(self):
        solver = CalibratedIK(self.chain, self.offsets, self.bounds)
        for target, seed in ((np.eye(4), np.full(6, float('nan'))),
                             (np.full((4, 4), float('nan')), np.zeros(6))):
            with self.assertRaises(ValueError): list(solver.solutions(target, seed))

    def test_session_fast_solution_still_requires_collision_validation(self):
        s = self.session(); q = np.asarray(self.data['waypoints']['Grasp']['joints_rad'])
        target = self.chain.fk(q, self.offsets)
        with patch.object(backend.RobotSession, 'solve') as numeric:
            answer = s.solve(target, q+.02)
        numeric.assert_not_called(); s.check_state.assert_called_once()
        np.testing.assert_allclose(self.chain.fk(answer, self.offsets), target, atol=1e-5)
        s.check_state.side_effect = CollisionFailure('blocked')
        with self.assertRaises(CollisionFailure): s.solve(target, q+.02)

    def test_failed_or_disabled_fast_solver_uses_original_numerical_solver(self):
        q = np.asarray(self.data['waypoints']['Grasp']['joints_rad'])
        for disabled in (False, True):
            with self.subTest(disabled=disabled):
                s = self.session(); s.config['fast_ik_enabled'] = not disabled
                s._fast_ik = Mock(); s._fast_ik.solutions.return_value = []
                with patch.object(backend.RobotSession, 'solve', return_value=q) as numeric:
                    answer = s.solve(self.chain.fk(q, self.offsets), q)
                numeric.assert_called_once(); np.testing.assert_array_equal(answer, q)

    def test_cartesian_branch_jump_falls_back_without_accepting_fast_jump(self):
        s = self.session(); s._local_ik = True
        seed = np.zeros(6); far = np.full(6, .2)
        s._fast_ik = Mock(); s._fast_ik.solutions.return_value = [far]
        with patch.object(backend.RobotSession, 'solve', return_value=seed) as numeric:
            answer = s.solve(np.eye(4), seed)
        numeric.assert_called_once(); s.check_state.assert_not_called()
        np.testing.assert_array_equal(answer, seed)

    def test_missing_dependency_or_chain_mismatch_keeps_numerical_fallback(self):
        q = np.asarray(self.data['waypoints']['Grasp']['joints_rad'])
        target = self.chain.fk(q, self.offsets)
        for unavailable in ('dependency', 'model'):
            with self.subTest(unavailable=unavailable):
                s = self.session()
                guard = (patch.dict(sys.modules, {'fast_ik': None}) if unavailable == 'dependency' else
                         patch('fast_ik.CalibratedIK', side_effect=ValueError('FK mismatch')))
                with guard, patch.object(backend.rospy, 'logwarn'), \
                        patch.object(backend.RobotSession, 'solve', return_value=q) as numeric:
                    np.testing.assert_array_equal(s.solve(target, q), q)
                numeric.assert_called_once()

    def test_cancel_between_fast_solutions_prevents_return_or_fallback(self):
        s = self.session(); s._fast_ik = Mock(); s._fast_ik.solutions.return_value = [np.zeros(6)]
        s.cancel_check.side_effect = [None, Cancelled('cancelled')]
        with patch.object(backend.RobotSession, 'solve') as numeric:
            with self.assertRaises(Cancelled): s.solve(np.eye(4), np.zeros(6))
        numeric.assert_not_called(); s.check_state.assert_not_called()


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class PlanningRetryTests(unittest.TestCase):
    def session(self):
        s = backend.Session.__new__(backend.Session)
        s.config = dict(planning_attempts=3, joint_plan_timeout_sec=8.)
        s.cancel_check = Mock(); s.group = Mock(); s.active = ['j1']
        s.dense_timed = Mock(side_effect=lambda trajectory, start: trajectory)
        s.check_trajectory = Mock()
        return s

    def test_first_failed_search_retries_from_same_start_and_validates_success(self):
        s = self.session(); state = RobotState(); trajectory = Mock()
        s.group.plan.side_effect = [(False, None), (True, trajectory)]
        self.assertIs(s.plan_joint(np.array([.1]), state), trajectory)
        self.assertEqual(s.group.plan.call_count, 2)
        self.assertTrue(all(call.args[0] is state for call in s.group.set_start_state.call_args_list))
        s.check_trajectory.assert_called_once_with(trajectory, state)

    def test_calibrated_collision_retries_with_a_new_moveit_route(self):
        s = self.session(); first = Mock(); second = Mock()
        s.group.plan.side_effect = [(True, first), (True, second)]
        s.check_trajectory.side_effect = [CollisionFailure('calibrated collision'), None]
        self.assertIs(s.plan_joint(np.array([.1]), RobotState()), second)
        self.assertEqual(s.check_trajectory.call_count, 2)

    def test_retries_share_total_search_budget(self):
        s = self.session(); s.group.plan.return_value = (False, None)
        with patch.object(backend.time, 'monotonic', side_effect=[0., 0., 1., 3.]):
            with self.assertRaises(RuntimeError): s.plan_joint(np.array([.1]), RobotState())
        values = [call.args[0] for call in s.group.set_planning_time.call_args_list]
        np.testing.assert_allclose(values, [8./3, 4., 5.])
        s.dense_timed.assert_not_called()

    def test_cancel_and_invalid_trajectory_do_not_trigger_more_searches(self):
        for error in (Cancelled('cancelled'), RuntimeError('trajectory timing invalid')):
            with self.subTest(error=error):
                s = self.session(); s.group.plan.return_value = (True, Mock())
                s.check_trajectory.side_effect = error
                with self.assertRaises(type(error)): s.plan_joint(np.array([.1]), RobotState())
                self.assertEqual(s.group.plan.call_count, 1)


if __name__ == '__main__': unittest.main()
