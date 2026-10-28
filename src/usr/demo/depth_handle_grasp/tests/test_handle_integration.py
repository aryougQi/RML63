"""Original CLI wiring with a fake robot; never connects to ROS/hardware."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
import numpy as np
import yaml
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE/'scripts'))
import grasp_demo
from handle_pipeline import side_place_targets, plan_side_place, precheck_side_place
from grasp_geometry import transform, validate


class HandleIntegrationTests(unittest.TestCase):
    def test_unverified_execute_is_rejected_before_ros_capture_or_planning(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            config=root/'unverified.yaml';config.write_text('place_aside: true\nside_place_scene_verified: false\n')
            with patch.object(grasp_demo,'HERE',root), \
                 patch.object(sys,'argv',['grasp_demo.py','--config',str(config),'--execute']), \
                 patch.object(grasp_demo,'RobotSession') as session, \
                 patch.object(grasp_demo.moveit_commander,'roscpp_initialize') as cpp, \
                 patch.object(grasp_demo.rospy,'init_node') as node, \
                 patch.object(grasp_demo,'acquire') as capture:
                self.assertEqual(grasp_demo.main(),2)
                session.assert_not_called();cpp.assert_not_called();node.assert_not_called();capture.assert_not_called()
            run=Path(json.loads((root/'reports/latest_run.json').read_text())['run_directory'])
            result=json.loads((run/'status.json').read_text())
            self.assertIn('configuration rejected before capture/planning',result['error'])

    def test_later_endpoint_collision_skips_expensive_path_planning(self):
        session = MagicMock()
        session.solve.side_effect = [np.zeros(6), RuntimeError('state collision')]
        with self.assertRaisesRegex(RuntimeError, 'endpoint transfer: state collision'):
            precheck_side_place(session,np.eye(4),np.eye(4),np.zeros(6),
                                {'open_width_m':.045,'contact_width_m':.016})
        session.plan_joint.assert_not_called()
        session.plan_descent.assert_not_called()
        session.set_tool_width.assert_called_with(.045)

    def test_expired_budget_stops_before_ik_or_collision_work(self):
        from grasp_ros import RobotSession
        import time
        session=RobotSession.__new__(RobotSession)
        session.planning_deadline=time.monotonic()-1
        session.validity=MagicMock()
        with self.assertRaisesRegex(RuntimeError,'time budget exceeded'):
            session.check_state(np.zeros(6))
        with self.assertRaisesRegex(RuntimeError,'time budget exceeded'):
            session.solve(np.eye(4),np.zeros(6))
        session.validity.assert_not_called()

    def test_side_place_geometry_and_limits(self):
        grasp = transform([-.3, 0, .14], np.diag([1., -1., -1.]))
        targets = side_place_targets(grasp, {})
        np.testing.assert_allclose(targets['place'][:3, 3], [-.3, -.05, .14])
        self.assertAlmostEqual(targets['lift'][2, 3], .17)
        for target in targets.values():
            validate(target); np.testing.assert_array_equal(target[:3, :3], grasp[:3, :3])
        for offset in ([0, 0, 0], [0, .2, 0], [0, .05, .01], [0, float('nan'), 0]):
            with self.assertRaises(ValueError): side_place_targets(grasp, {'side_place_offset_base_m': offset})

    def test_original_cli_first_complete_returns_without_other_positions(self):
        self._run_original_cli('first_feasible')

    def test_original_cli_best_mode_compares_complete_candidates(self):
        self._run_original_cli('best_within_budget')

    def test_later_budget_failures_cannot_erase_completed_solution(self):
        self._run_original_cli('best_within_budget',fail_later=True)

    def _run_original_cli(self,mode,fail_later=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = yaml.safe_load((HERE/'config/grasp.yaml').read_text())
            visual = root/'visual.json'
            visual.write_text(json.dumps(dict(thickness_m=.007, width_m=.016,
                max_face_approach_angle_deg=25., max_axis_approach_dot=.35)))
            config['visual_config'] = str(visual)
            # Keep this wiring test small while preserving multiple positions.
            config['inward_grasp_tilt_candidates_deg'] = [0]
            config['grasp_pad_raise_candidates_m'] = [0]
            config['prefer_inward_grasp'] = False
            config['planning_mode'] = mode
            cfg_path = root/'grasp.yaml'; cfg_path.write_text(yaml.safe_dump(config))
            target = transform([-.3, 0, .14], np.diag([1., -1., -1.]))
            capture = dict(base_depth=np.eye(4).tolist(), base_grasp=target.tolist(), camera_pose=target.tolist(),
                visual_result=dict(success=True, coordinate_frame='depth_optical', units='metres',
                    pose={'valid':True}, support_plane={}, measurement=dict(
                        surface_center_m=[-.3, 0, .1435], axis_direction=[1, 0, 0], outward_normal=[0, 0, 1],
                        length_m=.2, width_m=.016, plane_rmse_m=.0005, plane_inlier_ratio=.95)))
            session = MagicMock()
            session.chain.fk.return_value = transform([-.3, 0, .3], np.diag([1., -1., -1.]))
            session.offsets = np.zeros(6)
            session.cal = {'provenance':{'independent_validation':{'position_rms_mm':.5}}}
            session.minimum_observed_clearance_m = .02
            session.add_scene.return_value = {'unobserved_geometry_verified': False}
            names = ['joint%d' % i for i in range(1, 7)]
            def state(q):
                result = RobotState(); result.joint_state.name = names; result.joint_state.position = list(q)
                return result
            session.state_for.side_effect = state
            session.solve.side_effect = lambda pose, seed: np.r_[pose[:3, 3], np.zeros(3)]
            def trajectory(start, end):
                result = RobotTrajectory(); result.joint_trajectory.joint_names = names
                result.joint_trajectory.points = [
                    JointTrajectoryPoint(positions=list(start), time_from_start=rospy.Duration(0)),
                    JointTrajectoryPoint(positions=list(end), time_from_start=rospy.Duration(1))]
                return result
            joint_calls=[]
            def joint(q,initial):
                joint_calls.append(True)
                if fail_later and len(joint_calls)>1:
                    from planner_metrics import PlanningTimeout
                    raise PlanningTimeout('later candidate time budget exceeded')
                return trajectory(initial.joint_state.position,q)
            session.plan_joint.side_effect=joint
            def cart(tcp, start, end, initial):
                return trajectory(initial.joint_state.position, np.r_[end[:3, 3], np.zeros(3)])
            session.plan_descent.side_effect = cart
            acquired = (capture, np.zeros(6), rospy.Time(1), np.zeros((1,1)), {}, np.zeros((1,1)))
            with patch.object(grasp_demo, 'HERE', root), patch.object(grasp_demo, 'RobotSession', return_value=session), \
                 patch.object(grasp_demo, 'acquire', return_value=acquired), \
                 patch.object(grasp_demo, 'GripperIO') as hardware, \
                 patch.object(grasp_demo.moveit_commander, 'roscpp_initialize'), \
                 patch.object(grasp_demo.rospy, 'init_node'), patch.object(grasp_demo.rospy, 'sleep'), \
                 patch.object(sys, 'argv', ['grasp_demo.py', '--config', str(cfg_path), '--hold-seconds', '0']):
                result = grasp_demo.main()
            self.assertEqual(result, 0)
            report = Path(json.loads((root/'reports/latest_run.json').read_text())['run_directory'])
            candidates = json.loads((report/'handle_candidates.json').read_text())
            self.assertFalse(candidates['requires_basket_pose']); self.assertFalse(candidates['requires_place_target'])
            planning = json.loads((report/'handle_planning.json').read_text())
            checkpoint=json.loads((report/'complete_solution_checkpoint.json').read_text())
            self.assertEqual(checkpoint['candidate_id'],planning['selected_candidate_id'])
            if mode=='first_feasible' or fail_later:self.assertEqual(len(planning['complete']),1)
            else:self.assertGreaterEqual(len(planning['complete']), 2)
            self.assertEqual(planning['complete'][0]['candidate_id'], planning['selected_candidate_id'])
            paths = json.loads((report/'planned_trajectories.json').read_text())
            self.assertEqual(set(paths), {'pregrasp','descent','lift','transfer','place','retreat'})
            hardware.assert_not_called(); session.execute_one.assert_not_called()
            session.verify_hardware.assert_called_once_with(require_execution=False)

    def test_failed_cartesian_path_rejects_whole_side_task(self):
        session = MagicMock(); session.plan_descent.side_effect = RuntimeError('collision')
        with self.assertRaisesRegex(RuntimeError, 'collision'):
            plan_side_place(session, np.eye(4), np.eye(4), RobotState(),
                            {'open_width_m':.045,'contact_width_m':.016})
        session.set_tool_width.assert_called_with(.045)
        session.execute_one.assert_not_called()

if __name__ == '__main__': unittest.main()
