import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from handle_detect.geograsp import plan_grasp
from handle_detect.data import save_frame
from handle_detect.synthetic import render
from scripts.compare_grasps import compare

ROOT = Path(__file__).resolve().parents[1]
GC = json.loads((ROOT/'configs/geograsp.json').read_text())
BASE = json.loads((ROOT/'configs/default.json').read_text())


def pose_config():
    c = copy.deepcopy(BASE)
    c.update(shape='curved_rectangular_top_face', length_m=.2, width_m=.016,
             thickness_m=.007, dimensions_verified=True, approach_verified=True,
             approach_camera=[0, 0, 1], roi_xywh=[65, 65, 190, 110],
             roi_confirms_handle_identity=True)
    return c


def cloud(angle=0., gap=False):
    x, y = np.meshgrid(np.linspace(-.1, .1, 120), np.linspace(-.008, .008, 16))
    z = .6+.006*(x/.1)**2+.1*y
    z += np.random.default_rng(19).normal(0, .0002, z.shape)
    p = np.column_stack((x.ravel()*np.cos(angle)-y.ravel()*np.sin(angle),
                         x.ravel()*np.sin(angle)+y.ravel()*np.cos(angle), z.ravel()))
    return p[abs(x.ravel()) > .008] if gap else p


class GeoGraspTests(unittest.TestCase):
    def test_known_ribbon_contacts_and_rotation(self):
        for angle in [0., .7, 1.4, 2.3]:
            r = plan_grasp(cloud(angle), pose_config(), GC)
            self.assertTrue(r['success'], r['reason'])
            c = r['selected_candidate']
            self.assertEqual(c['axial_fraction'], 0.)
            self.assertLess(np.linalg.norm(np.asarray(c['surface_center_m'])-[0, 0, .6]), .001)
            self.assertLess(abs(c['width_m']-.016), .002)
            contacts = np.array(c['inferred_contacts_m'])
            np.testing.assert_allclose(contacts.mean(0), r['pose']['grasp_point_m'], atol=1e-10)
            rmat = np.array(r['pose']['rotation_matrix'])
            np.testing.assert_allclose(rmat.T @ rmat, np.eye(3), atol=1e-9)
            self.assertAlmostEqual(np.linalg.det(rmat), 1.)
            self.assertGreater(abs(np.dot(c['local_axis'], [np.cos(angle), np.sin(angle), 0])), .999)
            self.assertFalse(r['observed_antipodal_verified'])
            self.assertFalse(r['force_closure_verified'])
            self.assertFalse(r['clearance_checked'])
            self.assertFalse(c['gripper_opening_verified'])

    def test_cannot_bridge_unobserved_center(self):
        gc = copy.deepcopy(GC)
        gc['candidate_fractions'] = [0.]
        r = plan_grasp(cloud(gap=True), pose_config(), gc)
        self.assertFalse(r['success'])
        self.assertIsNone(r['pose'])

    def test_insufficient_opening_and_unverified_dimensions(self):
        gc = copy.deepcopy(GC)
        gc['gripper_max_opening_m'] = .010
        r = plan_grasp(cloud(), pose_config(), gc)
        self.assertFalse(r['success'])
        self.assertIsNone(r['pose'])
        c = pose_config(); c['dimensions_verified'] = False
        self.assertFalse(plan_grasp(cloud(), c, GC)['success'])
        c = pose_config(); c['approach_verified'] = False
        self.assertFalse(plan_grasp(cloud(), c, GC)['success'])

    def test_invalid_degenerate_and_noisy_clouds(self):
        for p in [np.empty((0, 3)), np.full((100, 3), np.nan),
                  np.column_stack((np.linspace(-.1, .1, 100), np.zeros(100), np.full(100, .6)))]:
            r = plan_grasp(p, pose_config(), GC)
            self.assertFalse(r['success'])
            self.assertIsNone(r['pose'])
        gc = copy.deepcopy(GC); gc['score_weights'] = {'flatness': 0}
        self.assertFalse(plan_grasp(cloud(), pose_config(), gc)['success'])

    def test_comparison_inputs_and_empty_run_clear_artifacts(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            c = pose_config(); c.update(shape='rectangular_top_face', length_m=.24, width_m=.024, thickness_m=.018)
            for seed in range(2):
                d, m = render(seed)
                save_frame(root/('frame_%06d' % seed), d, m)
            output = root/'visuals'
            s = compare(sorted(root.glob('frame_*')), c, GC, output)
            self.assertEqual(s['baseline']['successful_frames'], 2)
            self.assertEqual(s['simplified_geograsp']['successful_frames'], 2)
            self.assertTrue((output/'grasp_comparison.png').exists())
            s = compare([], c, GC, output)
            self.assertEqual(s['frame_count'], 0)
            self.assertFalse((output/'grasp_comparison.png').exists())
            r = json.loads((output/'geograsp_comparison/geograsp_pose.json').read_text())
            self.assertFalse(r['success'])
            self.assertIsNone(r['pose'])

    def test_unstable_sequence_cannot_publish_final_pose(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            c = pose_config()
            c.update(shape='rectangular_top_face', length_m=.24, width_m=.024,
                     thickness_m=.018, max_position_std_m=1e-12)
            for seed in range(30):
                d, m = render(seed)
                save_frame(root/('frame_%06d' % seed), d, m)
            output = root/'visuals'
            summary = compare(sorted(root.glob('frame_*')), c, GC, output)
            self.assertGreater(summary['simplified_geograsp']['successful_frames'], 0)
            self.assertFalse(summary['simplified_geograsp']['stability_pass'])
            final = json.loads((output/'geograsp_comparison/geograsp_pose.json').read_text())
            self.assertFalse(final['success'])
            self.assertIsNone(final['pose'])
