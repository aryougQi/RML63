#!/usr/bin/env python3
"""Compare the existing ribbon fit and an original GeoGrasp-inspired planner."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import argparse
import csv
import json
from pathlib import Path
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from handle_detect.data import load_frame, points_from_pixels, write_json
from handle_detect.detection import detect
from handle_detect.geograsp import plan_grasp
from handle_detect.paths import detail_directory
from handle_detect.visualization import project


def method_stats(results, config, contiguous):
    valid = [r for r in results if r['success']]
    s = dict(total_frames=len(results), successful_frames=len(valid),
             success_rate=len(valid)/max(1, len(results)), position_std_m=None,
             max_axis_deviation_deg=None, max_tool_rotation_deviation_deg=None,
             stability_pass=False)
    if len(valid) < 2:
        return s
    centers = np.array([r['pose']['grasp_point_m'] for r in valid])
    rotations = Rotation.from_matrix(np.array([r['pose']['rotation_matrix'] for r in valid]))
    # A two-finger grasp is unchanged by rotating 180 degrees about tool Z.
    matrices = rotations.as_matrix()
    reference = matrices[0]
    for matrix in matrices:
        alternate = matrix @ np.diag([-1., -1., 1.])
        if np.trace(reference.T @ alternate) > np.trace(reference.T @ matrix):
            matrix[:] = alternate
    rotations = Rotation.from_matrix(matrices)
    deviation = np.degrees((rotations.mean().inv()*rotations).magnitude())
    axes = np.array([r['axis'] for r in valid])
    _, vectors = np.linalg.eigh(axes.T @ axes)
    angles = np.degrees(np.arccos(np.clip(abs(axes @ vectors[:, -1]), 0, 1)))
    std = np.std(centers, axis=0, ddof=1)
    s.update(position_std_m=std.tolist(), max_axis_deviation_deg=float(max(angles)),
             max_tool_rotation_deviation_deg=float(max(deviation)),
             selected_tilt_degrees=sorted(set(r['pose']['tilt_about_handle_deg'] for r in valid)))
    s['stability_pass'] = bool(contiguous and len(results) >= 30
        and s['success_rate'] >= config['min_success_rate']
        and max(std) <= config['max_position_std_m']
        and max(angles) <= config['max_direction_deviation_deg']
        and max(deviation) <= config['max_direction_deviation_deg'])
    return s


def render_panel(depth, meta, mask, result, title, contacts=None, candidates=None, status_note=None):
    z = depth.astype(float)*meta['depth_scale_m_per_unit']
    valid = depth > 0
    image = np.zeros(depth.shape, np.uint8)
    if valid.any():
        lo, hi = np.percentile(z[valid], [2, 98])
        image[valid] = np.clip((z[valid]-lo)/max(.001, hi-lo)*255, 0, 255).astype(np.uint8)
    image = cv2.applyColorMap(image, cv2.COLORMAP_TURBO)
    image[~valid] = 0
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(image, contours, -1, (0, 255, 255), 1)
    k = meta['intrinsics']
    if candidates:
        for c in candidates:
            if c['accepted']:
                cv2.circle(image, project(np.asarray(c['surface_center_m']), k), 2, (160, 160, 160), -1)
    if result.get('pose'):
        pose = result['pose']
        center = np.asarray(pose['grasp_point_m'])
        rotation = np.asarray(pose['rotation_matrix'])
        cv2.circle(image, project(center, k), 4, (0, 0, 255), -1)
        cv2.arrowedLine(image, project(center-rotation[:, 0]*.025, k),
                       project(center+rotation[:, 0]*.025, k), (255, 255, 255), 1)
        cv2.arrowedLine(image, project(center-rotation[:, 2]*.035, k), project(center, k), (255, 255, 0), 1)
    if contacts is not None:
        contacts = [project(np.asarray(c), k) for c in contacts]
        cv2.line(image, contacts[0], contacts[1], (255, 0, 255), 2)
        for c in contacts:
            cv2.circle(image, c, 3, (255, 0, 255), -1)
    image = cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    band = np.full((65, image.shape[1], 3), 25, np.uint8)
    cv2.putText(band, title, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    label = status_note or ('geometric candidate OK' if result['success'] else 'FAIL: '+result['reason'][:65])
    cv2.putText(band, label, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, .45,
                (0, 255, 0) if result['success'] and not status_note else (0, 140, 255), 1)
    return np.vstack((band, image))


def compare(paths, config, geo_config, output):
    output = Path(output)
    details = detail_directory(output)/'geograsp_comparison'
    details.mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    baseline, proposed, records, rows = [], [], [], []
    identities, frame_numbers, timestamps, calibration = set(), [], [], None
    last = None
    for path in paths:
        b = dict(success=False, reason='unprocessed', pose=None)
        g = dict(success=False, reason='unprocessed', pose=None)
        row = dict(frame=path.name, baseline_success=False, geograsp_success=False,
                   baseline_reason='', geograsp_reason='', shared_detection_and_baseline_ms='',
                   geograsp_incremental_ms='', selected_fraction='', score='',
                   grasp_point_difference_m='', rotation_difference_deg='', patch_rmse_m='')
        try:
            d, m = load_frame(path)
            signature = (m['intrinsics'], m['depth_scale_m_per_unit'], m.get('serial'), m.get('origin'))
            if calibration is not None and signature != calibration:
                raise ValueError('calibration or source changed')
            calibration = signature
            identity = (m.get('frame_number'), m.get('timestamp_ms'))
            if identity in identities or None in identity:
                raise ValueError('duplicate or missing frame identity')
            identities.add(identity)
            frame_numbers.append(identity[0]); timestamps.append(identity[1])
            b, mask, fit = detect(d, m, config)
            row['shared_detection_and_baseline_ms'] = b['processing_ms']
            if b.get('detection_success'):
                pc = dict(config)
                if config.get('support_plane', {}).get('enabled', False):
                    if 'support_plane' not in b:
                        raise ValueError('shared tabletop reference invalid')
                    pc.update(approach_camera=b['support_plane']['approach_camera'], approach_verified=True)
                start = time.perf_counter()
                # Same segmented RAW points, independent local normal/curvature planner.
                g = plan_grasp(points_from_pixels(d, m, mask > 0), pc, geo_config)
                row['geograsp_incremental_ms'] = (time.perf_counter()-start)*1000
            else:
                g.update(reason='shared handle detection rejected')
            if b['success']:
                b['axis'] = b['measurement']['axis_direction']
            if g['success']:
                selected = g['selected_candidate']
                g['axis'] = selected['local_axis']
                row.update(selected_fraction=selected['axial_fraction'], score=selected['score'],
                           patch_rmse_m=selected['patch_rmse_m'])
            if b['success'] and g['success']:
                row['grasp_point_difference_m'] = float(np.linalg.norm(
                    np.asarray(b['pose']['grasp_point_m'])-g['pose']['grasp_point_m']))
                rb, rg = np.array(b['pose']['rotation_matrix']), np.array(g['pose']['rotation_matrix'])
                alternatives = [rg, rg @ np.diag([-1., -1., 1.])]
                row['rotation_difference_deg'] = min(float(np.degrees(
                    Rotation.from_matrix(rb.T @ candidate).magnitude())) for candidate in alternatives)
            last = (d, m, mask, b, g)
        except (ValueError, KeyError, OSError, RuntimeError, ImportError, np.linalg.LinAlgError) as exc:
            b = dict(success=False, reason=str(exc), pose=None)
            g = dict(success=False, reason=str(exc), pose=None)
            # Invalid last input cannot leave a previous successful image.
            last = None
        row.update(baseline_success=b['success'], geograsp_success=g['success'],
                   baseline_reason=b['reason'], geograsp_reason=g['reason'])
        for label, result in [('baseline', b), ('geograsp', g)]:
            for key, value in zip(('x_m', 'y_m', 'z_m'), result['pose']['grasp_point_m'] if result['success'] else ['', '', '']):
                row[label+'_'+key] = value
        baseline.append(b); proposed.append(g); rows.append(row)
        records.append(dict(frame=path.name, baseline=b, geograsp=g))
    contiguous = bool(len(frame_numbers) == len(paths) and len(paths) >= 2
                      and np.all(np.diff(frame_numbers) == 1) and np.all(np.diff(timestamps) > 0))
    summary = dict(frame_count=len(paths), sequence_contiguous=contiguous,
        baseline=method_stats(baseline, config, contiguous),
        simplified_geograsp=method_stats(proposed, config, contiguous),
        comparison_scope='grasp planners on the same detected raw-depth handle mask and tabletop reference; not independent object detectors',
        timing_scope='shared detection includes baseline fit/pose; GeoGrasp timing is incremental planning and deprojection after that shared stage',
        physical_grasp_experiments_performed=False, absolute_accuracy_ground_truth_available=False,
        observed_antipodal_verified=False, clearance_checked=False,
        contact_evidence='top edges plus rectangular-section prior; opposite sidewall normals are not observed',
        baseline_config=config, geograsp_config=geo_config)
    fractions = [r['selected_fraction'] for r in rows if r['geograsp_success']]
    summary['selected_fraction_counts'] = {str(x): fractions.count(x) for x in sorted(set(fractions))}
    shared = [r['shared_detection_and_baseline_ms'] for r in rows if r['shared_detection_and_baseline_ms'] != '']
    extra = [r['geograsp_incremental_ms'] for r in rows if r['geograsp_incremental_ms'] != '']
    differences = [r['grasp_point_difference_m'] for r in rows if r['grasp_point_difference_m'] != '']
    if shared:
        summary['shared_detection_and_baseline_mean_ms'] = float(np.mean(shared))
    if extra:
        summary.update(geograsp_incremental_mean_ms=float(np.mean(extra)), geograsp_incremental_p95_ms=float(np.percentile(extra, 95)))
    if differences:
        summary['mean_between_method_grasp_distance_m'] = float(np.mean(differences))
    write_json(details/'comparison_summary.json', summary)
    write_json(details/'frame_results.json', records)
    if rows:
        with (details/'comparison.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    elif (details/'comparison.csv').exists():
        (details/'comparison.csv').unlink()
    if last:
        d, m, mask, b, g = last
        bp = render_panel(d, m, mask, b, 'Existing curved-ribbon fit')
        selected = g.get('selected_candidate', {})
        gp = render_panel(d, m, mask, g, 'Simplified GeoGrasp: local surface ranking',
                          selected.get('inferred_contacts_m'), g.get('candidates'),
                          'UNSTABLE sequence; last candidate shown' if len(paths) >= 30 and
                          not summary['simplified_geograsp']['stability_pass'] else None)
        cv2.imwrite(str(output/'geograsp_detection.png'), gp)
        cv2.imwrite(str(output/'grasp_comparison.png'), np.hstack((bp, gp)))
        final = dict(g, sequence_stability_pass=summary['simplified_geograsp']['stability_pass']
                     if len(paths) >= 30 else None)
        if len(paths) >= 30 and not summary['simplified_geograsp']['stability_pass']:
            final = dict(method=g.get('method'), success=False, pose=None,
                         reason='GeoGrasp sequence failed stability gates; see frame_results.json for geometric candidates',
                         last_frame_candidate_generated=g['success'], sequence_stability_pass=False,
                         clearance_checked=False, observed_antipodal_verified=False)
        write_json(details/'geograsp_pose.json', final)
    else:
        for name in ('geograsp_detection.png', 'grasp_comparison.png'):
            (output/name).unlink(missing_ok=True)
        write_json(details/'geograsp_pose.json', dict(success=False, reason='no valid final input', pose=None))
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'data/captures/live_vertical_first_final')
    p.add_argument('--config', type=Path, default=ROOT/'configs/real_handle.json')
    p.add_argument('--geograsp-config', type=Path, default=ROOT/'configs/geograsp.json')
    p.add_argument('--output', type=Path, default=ROOT/'outputs')
    a = p.parse_args()
    cv2.setNumThreads(1)
    config = json.loads(a.config.read_text()); gc = json.loads(a.geograsp_config.read_text())
    paths = [a.input] if (a.input/'depth_raw.npy').exists() else sorted(a.input.glob('frame_*'))
    s = compare(paths, config, gc, a.output)
    print(json.dumps({k:v for k,v in s.items() if not k.endswith('_config')}, indent=2))
    return 0 if paths and s['simplified_geograsp']['successful_frames'] else 2

if __name__ == '__main__':
    sys.exit(main())
