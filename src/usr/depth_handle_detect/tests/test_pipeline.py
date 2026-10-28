import copy
import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from handle_detect.synthetic import render
from handle_detect.detection import detect
from handle_detect.data import load_frame, save_frame, points_from_pixels
from handle_detect.stability import evaluate

cv2.setNumThreads(1)
BASE = json.loads((Path(__file__).resolve().parents[1]/'configs/default.json').read_text())


def config():
    c = copy.deepcopy(BASE)
    c.update(roi_xywh=[65, 65, 190, 110], roi_confirms_handle_identity=True,
             dimensions_verified=True, approach_camera=[0, 0, 1], approach_verified=True)
    return c


class PipelineTests(unittest.TestCase):
    def test_confirmed_foreground_component_does_not_absorb_other_objects(self):
        d,m=render(angle=0.)
        row=d[120]*m['depth_scale_m_per_unit']
        edge=np.flatnonzero((row>.79)&(row<.81))[-1]
        d[120,edge+1:edge+3]=int(.86/m['depth_scale_m_per_unit'])
        d[30:33,30:33]=int(.8/m['depth_scale_m_per_unit'])
        c=config();c.update(roi_xywh=[20,20,280,200],depth_range_m=[.7,.9],
            segmentation_max_local_depth_range_m=.022,recover_target_foreground_component=True)
        r,mask,_=detect(d,m,c)
        self.assertTrue(r['success'],r)
        self.assertFalse(mask[30:33,30:33].any())
        self.assertFalse(mask[d*.00025>1.].any())
        self.assertTrue(mask[120,edge+1:edge+3].all())
        self.assertGreater(r['measurement']['foreground_component_points_added'],0)

    def test_depth_boundary_filter_keeps_native_data_and_known_center(self):
        d, m = render()
        original = d.copy()
        c = config()
        c.update(roi_xywh=[20, 20, 280, 200],
                 segmentation_max_local_depth_range_m=.022)
        r, mask, fit = detect(d, m, c)
        self.assertTrue(r['success'], r)
        np.testing.assert_array_equal(d, original)
        self.assertLess(np.linalg.norm(np.asarray(r['pose']['center_m'])-[0, 0, .809]), .002)
        self.assertLess(abs(r['measurement']['width_m']-.024), .002)
        self.assertLess(abs(r['measurement']['length_m']-.24), .006)
        target_points={tuple(point) for point in points_from_pixels(d,m,mask>0)}
        self.assertTrue(all(tuple(point) in target_points for point in fit['points']))

    def test_invalid_depth_boundary_filter_rejected(self):
        d, m = render()
        for limit in [0, -1, float('nan'), float('inf')]:
            c = config()
            c['segmentation_max_local_depth_range_m'] = limit
            with self.assertRaises(ValueError):
                detect(d, m, c)

    def test_known_truth_and_rotation(self):
        for theta in [0., .35, 1.2, 2.2]:
            d, m = render(angle=theta)
            c = config()
            c['roi_xywh'] = [20, 20, 280, 200]
            r, _, _ = detect(d, m, c)
            self.assertTrue(r['success'], r)
            self.assertLess(np.linalg.norm(np.array(r['pose']['center_m'])-[0, 0, .809]), .002)
            self.assertLess(abs(r['measurement']['length_m']-.24), .006)
            axis = np.array(r['measurement']['axis_direction'])
            self.assertGreater(abs(axis @ [np.cos(theta), np.sin(theta), 0]), .999)
            rot = np.array(r['pose']['rotation_matrix'])
            np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-8)
            self.assertAlmostEqual(np.linalg.det(rot), 1.)

    def test_fail_closed(self):
        d, m = render()
        for key, value in [('dimensions_verified', False), ('approach_verified', False),
                           ('roi_confirms_handle_identity', False), ('shape', 'cylinder'),
                           ('length_m', .5), ('approach_camera', [1, 0, 0])]:
            c = config()
            c[key] = value
            r, _, _ = detect(d, m, c)
            self.assertFalse(r['success'], key)
            self.assertIsNone(r['pose'])

    def test_invalid_and_empty(self):
        d, m = render(empty=True)
        for data in [d, np.zeros_like(d)]:
            r, _, _ = detect(data, m, config())
            self.assertFalse(r['success'])
            self.assertIsNone(r['pose'])
        with tempfile.TemporaryDirectory() as td:
            save_frame(Path(td)/'bad', d.astype(np.uint8), m)
            with self.assertRaises(ValueError):
                load_frame(Path(td)/'bad')

    def test_multiple_candidates(self):
        d, m = render(second=True)
        c = config()
        c['roi_xywh'] = [20, 20, 280, 200]
        r, _, _ = detect(d, m, c)
        self.assertFalse(r['success'], r)
        self.assertGreaterEqual(r['candidate_count'], 2)
        self.assertIsNone(r['pose'])

    def test_outliers_and_clipping(self):
        d, m = render()
        c = config()
        c['roi_xywh'] = [135, 90, 70, 60]
        self.assertFalse(detect(d, m, c)[0]['success'])
        d[95:145, 100:220:2] = 0
        self.assertFalse(detect(d, m, config())[0]['success'])

    def test_duplicate_sequence_fails(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            d, m = render()
            save_frame(root/'a', d, m)
            save_frame(root/'b', d, m)
            s = evaluate([root/'a', root/'b'], config(), root/'outputs', expected=2)
            self.assertFalse(s['stability_pass'])
            self.assertEqual(s['successful_frames'], 1)
            pose = json.loads((root/'outputs/handle_pose.json').read_text())
            self.assertIsNone(pose['pose'])

    def test_empty_sequence_clears_previous_pose(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root/'handle_pose.json').write_text('{"success":true,"pose":{"center_m":[1,2,3]}}')
            (root/'handle_points.ply').write_text('stale cloud')
            s = evaluate([], config(), root)
            self.assertFalse(s['stability_pass'])
            self.assertIsNone(json.loads((root/'handle_pose.json').read_text())['pose'])
            self.assertFalse((root/'handle_points.ply').exists())

    def test_frame_gaps_cannot_pass_continuous_test(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            for name, seed in [('a', 0), ('b', 2)]:
                d, m = render(seed)
                save_frame(root/name, d, m)
            s = evaluate([root/'a', root/'b'], config(), root/'outputs', expected=2)
            self.assertEqual(s['successful_frames'], 2)
            self.assertFalse(s['sequence_contiguous'])
            self.assertFalse(s['stability_pass'])

if __name__ == '__main__':
    unittest.main()
