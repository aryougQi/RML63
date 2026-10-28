"""Safety-equivalence and continuous-seed tests; no hardware or ROS services."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE/'scripts'))
from grasp_geometry import sample_bounds,bounded_sample_clearance,GripperGeometry,transform
from grasp_ros import RobotSession
from planner_metrics import PlannerMetrics,PlanningTimeout,IKFailure,CollisionFailure


class BroadphaseEquivalence(unittest.TestCase):
    def test_randomized_old_exact_verdict_matches_new_bound(self):
        rng=np.random.RandomState(63);margin=.004
        for _ in range(100):
            obstacles=rng.uniform(-.1,.1,(80,3));tree=cKDTree(obstacles)
            groups=[rng.normal(size=(70,3))*.015+rng.uniform(-.1,.1,3) for _ in range(4)]
            pose=np.eye(4);pose[:3,3]=rng.uniform(-.02,.02,3)
            angle=rng.uniform(-np.pi,np.pi)
            pose[:3,:3]=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
            exact=float(tree.query(np.vstack(groups)@pose[:3,:3].T+pose[:3,3],k=1)[0].min())
            bound,pruned,queried=bounded_sample_clearance(tree,sample_bounds(groups),pose,margin)
            self.assertLessEqual(bound,exact+1e-12)
            self.assertEqual(bound<margin,exact<margin)
            self.assertLessEqual(queried,280)

    def test_single_thin_obstacle_is_retained_and_detected(self):
        points=np.array([[0,0,0],[.1,0,0]])
        tree=cKDTree([[.1,.003,0]])
        value,pruned,queried=bounded_sample_clearance(tree,sample_bounds([points]),np.eye(4),.004)
        self.assertLess(value,.004);self.assertEqual(queried,2)

    def test_cad_cache_is_independent_of_scene_and_returns_copies(self):
        src=HERE.parents[2]
        g=GripperGeometry(src/'rml63_tools_description/upstream/EG2-4C2.urdf',src/'rml63_tools_description/meshes/eg2_4c2',[0,0,.006909],[0,0,-np.pi/3])
        a=g.at_width(.016);expected=a['flange_tcp'].copy();a['flange_tcp'][2,3]=999
        np.testing.assert_allclose(g.at_width(.016)['flange_tcp'],expected)
        g.mount[2,3]+=.001
        self.assertAlmostEqual(g.at_width(.016)['flange_tcp'][2,3],expected[2,3]+.001)


class CartesianRefinement(unittest.TestCase):
    def session(self):
        s=RobotSession.__new__(RobotSession);s.config=dict(max_joint_jump_deg=5,cartesian_local_retry_depth=2)
        s.active=['joint%d'%i for i in range(1,7)]
        s.metrics=PlannerMetrics();return s

    def test_previous_solution_is_seed_for_next_point(self):
        s=self.session();calls=[]
        def solve(target,seed):
            calls.append(seed.copy());q=seed.copy();q[0]=target[0,3];return q
        s.solve=solve;start=np.eye(4);end=start.copy();end[0,3]=.018
        points=s._cartesian_joints(np.eye(4),start,end,np.zeros(6),.003)
        np.testing.assert_allclose(calls[0],np.zeros(6))
        for previous,seed in zip(points,calls[1:]):np.testing.assert_allclose(previous,seed)
        self.assertGreaterEqual(len(points),6)
        self.assertAlmostEqual(points[-1][0],.018)

    def test_coarse_ik_failure_subdivides_before_rejecting(self):
        s=self.session()
        def solve(target,seed):
            if abs(target[0,3]-seed[0])>.0031:raise IKFailure('step too large for local numeric convergence')
            q=seed.copy();q[0]=target[0,3];return q
        s.solve=solve;start=np.eye(4);end=start.copy();end[0,3]=.018
        q=s.coarse_check(np.eye(4),start,end,np.zeros(6))
        self.assertAlmostEqual(q[0],.018)
        self.assertGreater(s.metrics.counts['local_ik_subdivisions'],0)
        self.assertFalse(hasattr(q,'joint_trajectory'))

    def test_collision_and_timeout_do_not_retry_or_relax(self):
        for error in (CollisionFailure('obstacle'),PlanningTimeout('budget')):
            s=self.session();s.solve=MagicMock(side_effect=error)
            start=np.eye(4);end=start.copy();end[0,3]=.018
            with self.assertRaises(type(error)):s.coarse_check(np.eye(4),start,end,np.zeros(6))
            self.assertEqual(s.solve.call_count,1)

    def test_local_subdivision_is_bounded(self):
        s=self.session();s.solve=MagicMock(side_effect=IKFailure('unreachable'))
        start=np.eye(4);end=start.copy();end[0,3]=.018
        with self.assertRaises(IKFailure):s.coarse_check(np.eye(4),start,end,np.zeros(6))
        self.assertEqual(s.solve.call_count,3)

    def test_cannot_expand_fine_spacing(self):
        s=self.session();s.config['cartesian_fine_step_m']=.004
        with self.assertRaisesRegex(ValueError,'3 mm'):
            s.plan_descent(np.eye(4),np.eye(4),np.eye(4),None)

    def test_orientation_is_not_silently_discarded(self):
        s=self.session();target=np.eye(4);target[:3,:3]=np.diag([-1,-1,1])
        with self.assertRaises(ValueError):s.coarse_check(np.eye(4),np.eye(4),target,np.zeros(6))

    def test_coarse_pass_cannot_hide_collision_between_coarse_points(self):
        from moveit_msgs.msg import RobotState
        s=self.session();s.group=MagicMock()
        def solve(target,seed):
            if abs(target[0,3]-.003)<1e-8:raise CollisionFailure('thin obstacle at a fine waypoint')
            q=seed.copy();q[0]=target[0,3];return q
        s.solve=solve;start=np.eye(4);end=start.copy();end[0,3]=.018
        s.coarse_check(np.eye(4),start,end,np.zeros(6))
        state=RobotState();state.joint_state.name=s.active;state.joint_state.position=[0.]*6
        with self.assertRaisesRegex(CollisionFailure,'fine waypoint'):
            s.plan_descent(np.eye(4),start,end,state)
        s.group.retime_trajectory.assert_not_called()


class RealCalibratedIK(unittest.TestCase):
    def test_execution_rejects_changed_start_before_sending_any_goal(self):
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        import rospy
        s=RobotSession.__new__(RobotSession);s.active=['joint%d'%i for i in range(1,7)]
        s.verify_hardware=MagicMock();s.measured=lambda:(np.array([.1,0,0,0,0,0]),rospy.Time(1))
        s.motion_client=MagicMock()
        trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=s.active
        trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*6)]
        with self.assertRaisesRegex(RuntimeError,'changed after planning'):
            s.execute_one(trajectory,np.eye(4),np.eye(4))
        s.motion_client.wait_for_server.assert_not_called();s.motion_client.send_goal.assert_not_called()

    def make_session(self):
        from publish_joint_calibration import load_config
        s=RobotSession.__new__(RobotSession);s.active=['joint%d'%i for i in range(1,7)]
        xml=(HERE/'reports/robot.urdf').read_text()
        src=HERE.parents[2]
        _,s.chain,s.offsets,_=load_config(src/'usr/calibration/rml63_handeye/config/active_joint_calibration.yaml',xml)
        bounds={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in ET.fromstring(xml).findall('joint') if j.get('name') in s.active}
        s.robot=SimpleNamespace(get_joint=lambda name:SimpleNamespace(bounds=lambda:bounds[name]))
        s.metrics=PlannerMetrics();s.check_state=MagicMock()
        return s,bounds

    def test_offsets_applied_once_and_joint_semantics_unchanged(self):
        s,bounds=self.make_session();q=np.array([.15,.27,-1.50,-.09,-1.81,0.])
        desired=s.chain.fk(q,s.offsets);solved=s.solve(desired,q+.004)
        np.testing.assert_allclose(s.chain.fk(solved,s.offsets),desired,atol=1e-5)
        np.testing.assert_allclose(s.check_state.call_args[0][0],solved)
        for n,v in zip(s.active,solved):self.assertTrue(bounds[n][0]<v<bounds[n][1])

    def test_actual_numeric_ik_rejects_unreachable_before_collision(self):
        s,_=self.make_session();target=np.eye(4);target[0,3]=2.
        with self.assertRaises(IKFailure):s.solve(target,np.zeros(6))
        s.check_state.assert_not_called()

if __name__=='__main__':unittest.main()
