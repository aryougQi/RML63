"""SYNTHETIC UNIT TESTS only. No camera imports/calls, no real-data/grasp claims.

Temporary arrays exercise file integrity and numerical behavior. They are never
stored under the project data/outputs/reports or reported as real L515 data.
"""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from pnp_detect.rgbd_io import (SCHEMA, pair_gate, read_pair, read_session, save_pair,
                                sha256, validate_arrays, write_json_new, write_png_new)
from pnp_detect.handle_depth_quality import (depth_statistics, evaluate_session, polygon_mask,
                                            temporal_statistics, validate_annotation)


def synthetic_metadata():
    # Intentionally tiny synthetic calibration, only for validating IO contracts.
    intr = dict(width=8, height=6, fx=10., fy=11., ppx=4., ppy=3.,
                coefficients=[0.] * 5, distortion_model='None', distortion_model_id=0,
                source='active_sdk_stream_profile')
    stream = dict(resolution_wh=[8, 6], intrinsics=intr, format='Z16',
                  depth_scale_m_per_unit=.001, frame_number=1, timestamp_ms=100.,
                  timestamp_domain_id=0, timestamp_domain='Hardware Clock')
    result = dict(depth_raw=deepcopy(stream), depth_aligned_to_color=deepcopy(stream), color=deepcopy(stream),
                  depth_to_color=dict(rotation_matrix=np.eye(3).tolist(), rotation_column_major=np.eye(3).flatten(order='F').tolist(), translation_m=[0., 0., 0.]))
    result['color']['format'] = 'RGB8'
    return result


POLICY = dict(min_depth_m=.1, max_depth_m=10., reference_tolerance_m=.01, min_temporal_observations=3)


class RGBDIOTests(unittest.TestCase):
    def test_raw_uint16_and_rgb_roundtrip_without_quantization(self):
        with tempfile.TemporaryDirectory(prefix='SYNTHETIC_rgbd_test_') as directory:
            root = Path(directory)
            rgb = np.arange(144, dtype=np.uint8).reshape(6, 8, 3)
            depth = np.arange(48, dtype=np.uint16).reshape(6, 8) * 1300
            frame = save_pair(root, root, 0, rgb, depth, depth, synthetic_metadata())
            got = read_pair(frame, root)
            for actual, expected in zip(got, [rgb, depth, depth]):
                np.testing.assert_array_equal(actual, expected)
            self.assertEqual(got[1].dtype, np.uint16)
            self.assertEqual(int(got[1][5, 7]), 61100)
            with self.assertRaises(FileExistsError):
                save_pair(root, root, 0, rgb, depth, depth, synthetic_metadata())

    def test_no_existing_json_or_png_is_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'report.json'
            write_json_new(path, {'synthetic_test': True})
            original = path.read_bytes()
            with self.assertRaises(FileExistsError):
                write_json_new(path, {'changed': True})
            self.assertEqual(path.read_bytes(), original)
            png = Path(directory) / 'image.png'
            write_png_new(png, np.zeros((3, 3), dtype=np.uint8))
            with self.assertRaises(FileExistsError):
                write_png_new(png, np.ones((3, 3), dtype=np.uint8))

    def test_reject_color_preview_and_eight_bit_depth(self):
        for bad in [np.zeros((6, 8), dtype=np.uint8), np.zeros((6, 8, 3), dtype=np.uint8)]:
            with self.assertRaises(ValueError):
                validate_arrays(np.zeros((6, 8, 3), dtype=np.uint8), bad,
                                np.zeros((6, 8), dtype=np.uint16), synthetic_metadata())

    def test_reject_wrong_scale_intrinsics_extrinsics_and_alignment_frame(self):
        def check(change):
            metadata = synthetic_metadata()
            change(metadata)
            with self.assertRaises(ValueError):
                validate_arrays(np.zeros((6, 8, 3), dtype=np.uint8), np.ones((6, 8), dtype=np.uint16),
                                np.ones((6, 8), dtype=np.uint16), metadata)
        check(lambda m: m['depth_raw'].update(depth_scale_m_per_unit=0))
        check(lambda m: m['color']['intrinsics'].update(fx=float('nan')))
        check(lambda m: m['depth_to_color'].update(rotation_matrix=np.zeros((3, 3)).tolist()))
        check(lambda m: m['depth_aligned_to_color'].update(frame_number=9))

    def test_pair_gate_rejects_different_clocks_skew_and_repeated_frames(self):
        metadata = synthetic_metadata()
        seen = {'color': set(), 'depth': set()}
        self.assertTrue(pair_gate(metadata, 20, seen)[0])
        metadata['color']['timestamp_domain_id'] = 1
        self.assertEqual(pair_gate(metadata, 20, seen)[1], 'incomparable_timestamp_domains')
        metadata = synthetic_metadata()
        metadata['color']['timestamp_ms'] += 21
        self.assertEqual(pair_gate(metadata, 20, seen)[1], 'timestamp_skew_exceeded')
        metadata = synthetic_metadata()
        seen['depth'].add(1)
        self.assertEqual(pair_gate(metadata, 20, seen)[1], 'repeated_color_or_depth_frame')

    def test_synthetic_manifest_never_accepted_as_real_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'SYNTHETIC_manifest.json'
            write_json_new(path, {'schema': SCHEMA, 'data_origin': 'SYNTHETIC_UNIT_TEST', 'frames': [1]})
            with self.assertRaises(ValueError):
                read_session(path, directory)


class ManualQualityTests(unittest.TestCase):
    def test_missing_depth_is_not_filled_or_reported_as_a_pass(self):
        depth = np.zeros((6, 8), dtype=np.uint16)
        result = depth_statistics(depth, np.ones((6, 8), dtype=bool), .001, POLICY)
        self.assertEqual(result['valid_fraction'], 0)
        self.assertEqual(result['zero_missing_count'], 48)
        self.assertIsNone(result['median_depth_m'])

    def test_units_missing_and_spatial_statistics(self):
        depth = np.array([[0, 1000], [1000, 3000]], dtype=np.uint16)
        result = depth_statistics(depth, np.ones((2, 2), dtype=bool), .001, POLICY, reference=1.)
        self.assertEqual(result['valid_fraction'], .75)
        self.assertEqual(result['median_depth_m'], 1.)
        self.assertAlmostEqual(result['reference_comparison']['fraction_within_reference_tolerance'], 2/3)

    def test_temporal_statistics_exclude_holes_and_need_observations(self):
        result = temporal_statistics([[1, np.nan], [1.001, np.nan], [.999, 2]], 3)
        self.assertEqual(result['eligible_pixel_count'], 1)
        self.assertAlmostEqual(result['per_pixel_std_median_mm'], np.std([1, 1.001, .999])*1000)
        self.assertIsNone(temporal_statistics([[np.nan], [np.nan]], 3)['per_pixel_std_median_mm'])

    def test_polygon_rejects_outside_self_crossing_and_degenerate_points(self):
        for p in [[[0,0],[9,0],[4,5]], [[0,0],[5,5],[0,5],[5,0]], [[0,0],[1,1],[2,2]]]:
            with self.assertRaises(ValueError):
                polygon_mask(p, [8, 6])
        self.assertTrue(polygon_mask([[1,1],[5,1],[5,4],[1,4]], [8,6]).any())

    def test_manual_annotation_binds_hash_resolution_and_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'SYNTHETIC_manifest.json'
            write_json_new(path, {'test_only': True})
            manifest = dict(frames=[dict(frame_id='frame_000000', files={'color': {'sha256': 'test_hash'}},
                                         metadata={'color': {'resolution_wh': [8, 6]}})])
            annotation = dict(schema='pnp_detect.manual_handle_regions.v1', source='manual',
                session_manifest_sha256=sha256(path), reference_frame_id='frame_000000',
                reference_rgb_sha256='test_hash', image_resolution_wh=[8, 6], coordinate_frame='native_color_pixels',
                scene_static_confirmed=False, handle_polygon=[[1,1],[5,1],[5,4],[1,4]])
            validate_annotation(annotation, path, manifest)
            for key, value in [('source','automatic'), ('reference_rgb_sha256','wrong'),
                               ('session_manifest_sha256','wrong'), ('image_resolution_wh',[4,3])]:
                bad = dict(annotation, **{key: value})
                with self.assertRaises(ValueError):
                    validate_annotation(bad, path, manifest)

    def test_static_sequence_metrics_remain_unverified_for_grasp(self):
        with tempfile.TemporaryDirectory(prefix='SYNTHETIC_quality_') as directory:
            root = Path(directory)
            frames = []
            for index in range(3):
                depth = np.full((6,8), 1000+index, dtype=np.uint16)
                metadata = synthetic_metadata()
                frame = save_pair(root, root, index, np.zeros((6,8,3), dtype=np.uint8), depth, depth, metadata)
                frames.append(frame)
            annotation = dict(reference_frame_id=frames[0]['frame_id'], image_resolution_wh=[8,6],
                              scene_static_confirmed=True)
            mask = polygon_mask([[1,1],[5,1],[5,4],[1,4]], [8,6])
            report = evaluate_session({'frames':frames}, annotation, (mask,None), root, POLICY)
            self.assertEqual(report['evaluated_frame_count'], 3)
            self.assertGreater(report['temporal']['eligible_pixel_count'], 0)
            self.assertFalse(report['robot_ready'])
            self.assertFalse(report['handle_surface_depth_identity_verified'])
            self.assertFalse(report['depth_validated_for_grasp'])
            annotation['scene_static_confirmed'] = False
            report = evaluate_session({'frames':frames}, annotation, (mask,None), root, POLICY)
            self.assertEqual(report['evaluated_frame_count'], 1)
            self.assertIsNone(report['temporal'])


if __name__ == '__main__':
    unittest.main()
