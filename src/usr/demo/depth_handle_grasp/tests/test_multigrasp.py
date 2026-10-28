from pathlib import Path
import sys
import unittest
import numpy as np
import yaml
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE/'scripts'))
from grasp_candidates import generate
from grasp_scoring import antipodal, evaluate, rank
from grasp_geometry import GripperGeometry, validate


class MultiGraspTests(unittest.TestCase):
    def setUp(self):
        src = HERE.parents[2]
        self.gripper = GripperGeometry(src/'rml63_tools_description/upstream/EG2-4C2.urdf',
            src/'rml63_tools_description/meshes/eg2_4c2', [0, 0, .006909], [0, 0, -np.pi/3])
        self.cfg = yaml.safe_load((HERE/'config/multigrasp.yaml').read_text())
        self.cfg['robust_trials'] = 4
        self.visual = dict(thickness_m=.007)
        self.result = dict(success=True, pose={'unused': True}, coordinate_frame='depth_optical', units='metres',
            measurement=dict(surface_center_m=[-.3, 0, .2], axis_direction=[1, 0, 0],
            outward_normal=[0, 0, 1], length_m=.2, width_m=.016, plane_rmse_m=.0007, plane_inlier_ratio=.95))

    def generate(self):
        return generate(self.result, np.eye(4), self.visual, self.cfg,
                        self.gripper.at_width(.016)['pad_bounds_tcp_m'])[0]

    def test_line_sampling_and_rotations(self):
        candidates = self.generate()
        self.assertGreater(len(candidates), 6)
        self.assertLessEqual(len(candidates), self.cfg['max_candidates'])
        for c in candidates:
            validate(c.tcp)
            self.assertAlmostEqual(c.tcp[2, 3], .1965)
            self.assertGreaterEqual(c.end_clearance_m, self.cfg['end_margin_m'])
            self.assertAlmostEqual(c.tcp[:3, 1] @ c.tangent, 0)
        self.assertEqual(len({tuple(c.tcp[:3, 3]) for c in candidates[:6]}), 1)

    def test_bow_uses_local_tangent_and_refuses_missing_curve(self):
        m = self.result['measurement']; m['fit_model'] = 'quadratic_rectangular_upper_face'
        with self.assertRaisesRegex(ValueError, 'centreline'): self.generate()
        x = np.linspace(-.1, .1, 100)
        m['surface_centerline_m'] = np.c_[-.3+x, np.zeros(100), .2+x*x].tolist()
        m['centerline_outward_normals'] = np.c_[-2*x, np.zeros(100), np.ones(100)].tolist()
        candidates = self.generate()
        self.assertTrue(any(abs(c.tangent[2]) > .01 for c in candidates))
        for c in candidates: validate(c.tcp)

    def test_transformed_curve(self):
        t = np.eye(4); t[:3, 3] = [.1, .2, .3]
        a = self.generate()[0]
        b = generate(self.result, t, self.visual, self.cfg,
                     self.gripper.at_width(.016)['pad_bounds_tcp_m'])[0][0]
        np.testing.assert_allclose(b.tcp[:3, 3], a.tcp[:3, 3]+t[:3, 3])

    def test_antipodal_normal_convention_and_friction(self):
        q, valid = antipodal([0, 0, 0], [0, 1, 0], [0, 1, 0], [0, -1, 0], .4)
        self.assertEqual(q, 1); self.assertTrue(valid)
        self.assertFalse(antipodal([0, 0, 0], [0, 1, 0], [1, 0, 0], [-1, 0, 0], .4)[1])
        self.assertFalse(antipodal([0, 0, 0], [0, 1, 0], [0, -1, 0], [0, 1, 0], .4)[1])
        with self.assertRaises(ValueError): antipodal([0]*3, [0]*3, [1, 0, 0], [-1, 0, 0], .4)

    def test_missing_contact_and_inferred_confidence(self):
        c = self.generate()[0]
        r = evaluate(c, self.cfg, self.gripper)
        self.assertTrue(r['accepted'], r)
        self.assertFalse(r['strict_antipodal_verified'])
        self.assertEqual(r['contact_source'], 'inferred_rectangular_section')
        self.assertLess(r['components']['anti'], 1)
        self.assertIsNone(r['clearance_m'])
        self.cfg['allow_section_inference'] = False
        self.assertIn('contact_geometry_missing', evaluate(c, self.cfg, self.gripper)['reasons'])

    def test_hard_constraints_and_sorting(self):
        candidates = self.generate()
        candidates[0].width_m = .08
        candidates[1].end_clearance_m = 0
        r = rank(candidates[:8], self.cfg, self.gripper)
        self.assertTrue(r[0]['accepted']); self.assertFalse(r[-1]['accepted'])
        scores = [x['score'] for x in r if x['accepted']]
        self.assertEqual(scores, sorted(scores, reverse=True))
        self.assertEqual(r[0]['robustness']['trials'], 4)
        self.assertFalse(r[0]['robustness']['real_success_probability'])

    def test_obstacle_rejection_and_no_short_rod_samples(self):
        c = self.generate()[0]
        from grasp_geometry import flange_target
        g = self.gripper.at_width(.016)
        link = flange_target(c.tcp, g['flange_tcp']) @ self.gripper.link('4C2_Link5', g['linkage_angle_rad'])
        p = self.gripper.meshes['4C2_Link5']['vertices'][0, 0]
        obstacle = (link[:3, :3] @ p+link[:3, 3])[None, :]
        self.assertIn('observed_obstacle_near_CAD', evaluate(c, self.cfg, self.gripper, obstacle)['reasons'])
        self.result['measurement']['length_m'] = .01
        self.assertEqual(self.generate(), [])

    def test_bad_quality_and_bad_sampling(self):
        self.cfg['sample_spacing_m'] = 0
        with self.assertRaises(ValueError): self.generate()
        self.result['measurement']['plane_rmse_m'] = float('nan')
        with self.assertRaises(ValueError): self.generate()



class PlanningGateTests(unittest.TestCase):
    def test_missing_place_and_execution_fail_closed(self):
        from grasp_mtc import validate_place, require_execution_disabled
        cfg = yaml.safe_load((HERE/'config/multigrasp.yaml').read_text())
        with self.assertRaisesRegex(ValueError, 'basket_pose_base'): validate_place(cfg)
        with self.assertRaisesRegex(RuntimeError, 'execution blocked'): require_execution_disabled(True)
        require_execution_disabled(False)
        cfg['basket_pose_base'] = cfg['place_pose_base'] = np.eye(4).tolist()
        with self.assertRaisesRegex(ValueError, 'verification'): validate_place(cfg)

    def test_trajectory_cost_uses_timed_paths(self):
        import rospy
        from moveit_task_constructor_msgs.msg import Solution, SubTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from grasp_mtc import trajectory_cost
        cfg = yaml.safe_load((HERE/'config/multigrasp.yaml').read_text())
        msg = Solution(); sub = SubTrajectory()
        sub.trajectory.joint_trajectory.joint_names = ['joint1', 'joint2']
        sub.trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[0., 0.], time_from_start=rospy.Duration(0)),
            JointTrajectoryPoint(positions=[.3, .4], time_from_start=rospy.Duration(2))]
        msg.sub_trajectory = [sub]
        cost, terms = trajectory_cost(msg, cfg['cost_weights'], None, cfg)
        self.assertAlmostEqual(terms['length'], .5)
        self.assertAlmostEqual(terms['duration'], 2.)
        self.assertAlmostEqual(terms['joint_motion'], .7)
        self.assertAlmostEqual(terms['clearance'], 1.)
        sub.trajectory.joint_trajectory.points[-1].time_from_start = rospy.Duration(0)
        with self.assertRaisesRegex(ValueError, 'time parameterization'):
            trajectory_cost(msg, cfg['cost_weights'], None, cfg)

if __name__ == '__main__': unittest.main()
