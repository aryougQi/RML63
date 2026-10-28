import copy
import json
from pathlib import Path
import unittest
import numpy as np
from handle_detect.geometry import fit_curved_handle, make_pose, Reject, points_in_handle_section
from handle_detect.support import fit_support_plane

BASE = json.loads((Path(__file__).resolve().parents[1]/'configs/default.json').read_text())


def curved_points(angle=0., bow=.010, noise=.00035):
    rng = np.random.default_rng(42)
    x, y = np.meshgrid(np.linspace(-.1, .1, 120), np.linspace(-.008, .008, 16))
    z = .6+bow*(x/.1)**2+.2*y+rng.normal(0, noise, x.shape)
    points = np.column_stack((x.ravel()*np.cos(angle)-y.ravel()*np.sin(angle),
                              x.ravel()*np.sin(angle)+y.ravel()*np.cos(angle), z.ravel()))
    core = points[((abs(x) < .097) & (abs(y) < .006)).ravel()]
    return points, core


def config():
    c = copy.deepcopy(BASE)
    c.update(shape='curved_rectangular_top_face', length_m=.2, width_m=.016,
             thickness_m=.007, dimensions_verified=True, approach_camera=[0, 0, 1],
             approach_verified=True, max_bow_sag_m=.015)
    return c


class CurvedTests(unittest.TestCase):
    def test_section_classification_keeps_adjacent_obstacles(self):
        cfg=config()
        fit=dict(surface_center=np.array([0.,0.,.6]),axis=np.array([1.,0.,0.]),
                 normal=np.array([0.,0.,-1.]),length=.2)
        points=np.array([[0,0,.6],[0,.0085,.6],[0,0,.605],
                         [0,.013,.6],[0,0,.62],[.11,0,.6],[0,0,.59]])
        original=points.copy()
        inside=points_in_handle_section(points,fit,cfg,.002)
        np.testing.assert_array_equal(inside,[True,True,True,False,False,False,False])
        np.testing.assert_array_equal(points,original)
        cfg['dimensions_verified']=False
        with self.assertRaises(Reject):points_in_handle_section(points,fit,cfg,.002)

    def test_curved_truth_and_boundary_rejection(self):
        for angle in [0., .6, 1.4, 2.4]:
            points, core = curved_points(angle)
            # Mixed foreground/background pixels at the boundary, not fabricated surface.
            mixed = points[::4].copy()
            mixed[:, 2] += .03
            f = fit_curved_handle(core, config(), np.vstack((points, mixed)))
            self.assertLess(np.linalg.norm(f['surface_center']-[0, 0, .6]), .0008)
            self.assertLess(abs(f['length']-.2), .006)
            self.assertLess(abs(f['width']-.016), .0015)
            self.assertLess(abs(f['bow_sag_m']-.01), .001)
            self.assertGreater(abs(f['axis'] @ [np.cos(angle), np.sin(angle), 0]), .999)
            self.assertLess(f['rmse'], .0007)

    def test_tilted_pose(self):
        points, core = curved_points()
        f = fit_curved_handle(core, config(), points)
        for tilt in [-15, 0, 15]:
            c = config()
            c['approach_policy'] = 'fixed_test'
            c['max_face_approach_angle_deg'] = 45
            c['tilt_about_handle_deg'] = tilt
            pose = make_pose(f, c)
            r = np.array(pose['rotation_matrix'])
            np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-9)
            self.assertAlmostEqual(np.linalg.det(r), 1.)
            self.assertAlmostEqual(r[2, 2], np.cos(np.radians(tilt)), places=6)
            np.testing.assert_allclose(pose['center_m'], f['surface_center']-f['normal']*.0035)
        c['tilt_about_handle_deg'] = 40
        with self.assertRaises(Reject):
            make_pose(f, c)

    def test_vertical_first_uses_tilt_only_when_needed(self):
        points, core = curved_points()
        f = fit_curved_handle(core, config(), points)
        c = config()
        c.update(approach_policy='vertical_first', tilt_about_handle_deg=15,
                 max_face_approach_angle_deg=25)
        self.assertEqual(make_pose(f, c)['tilt_about_handle_deg'], 0)
        f['axis'] = np.array([1., 0., 0.])
        f['normal'] = np.array([0., np.sin(np.radians(32)), -np.cos(np.radians(32))])
        pose = make_pose(f, c)
        self.assertEqual(pose['tilt_about_handle_deg'], 10)
        self.assertFalse(pose['clearance_checked'])

    def test_wrong_shape_geometry_rejected(self):
        points, core = curved_points(bow=.04)
        with self.assertRaises(Reject):
            fit_curved_handle(core, config(), points)
        points, core = curved_points()
        c = config()
        c['length_m'] = .35
        with self.assertRaises(Reject):
            fit_curved_handle(core, c, points)

    def test_support_plane_truth_and_failure(self):
        rng = np.random.default_rng(55)
        v, u = np.indices((100, 120))
        k = dict(width=120, height=100, fx=150., fy=150., ppx=60., ppy=50.)
        z = .7/(1-.15*(u-60)/150-.3*(v-50)/150)
        d = np.rint((z+rng.normal(0, .0003, z.shape))/.00025).astype(np.uint16)
        m = dict(intrinsics=k, depth_scale_m_per_unit=.00025)
        c = dict(horizontal_confirmed=True, roi_xywh=[10, 10, 100, 80])
        plane = fit_support_plane(d, m, c)
        expected = np.array([-.15, -.3, 1.]); expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(plane['approach_camera'], expected, atol=.001)
        self.assertLess(plane['rmse_m'], .0005)
        for bad in [np.zeros_like(d), rng.integers(1000, 5000, d.shape, dtype=np.uint16)]:
            with self.assertRaises(Reject):
                fit_support_plane(bad, m, c)
        c['horizontal_confirmed'] = False
        with self.assertRaises(Reject):
            fit_support_plane(d, m, c)
