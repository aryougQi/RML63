import csv
from pathlib import Path
import numpy as np
from .data import load_frame, write_json
from .detection import detect
from .visualization import save_outputs
from .paths import detail_directory


def evaluate(paths, cfg, output, expected=30):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    details = detail_directory(output)
    details.mkdir(parents=True, exist_ok=True)
    rows, results, centers, axes = [], [], [], []
    frame_numbers, timestamps = [], []
    reference_metadata = None
    identities = set()
    for path in paths:
        try:
            depth, meta = load_frame(path)
            # A stability sequence must share calibration and contain distinct frames.
            calibration = (meta['intrinsics'], meta['depth_scale_m_per_unit'], meta.get('serial'), meta.get('origin'))
            if reference_metadata is not None and calibration != reference_metadata:
                raise ValueError('calibration/device/origin changed in sequence')
            reference_metadata = calibration
            identity = (meta.get('frame_number'), meta.get('timestamp_ms'))
            if identity == (None, None) or identity in identities:
                raise ValueError('duplicate or missing frame identity')
            identities.add(identity)
            frame_numbers.append(meta.get('frame_number'))
            timestamps.append(meta.get('timestamp_ms'))
            result, mask, fit = detect(depth, meta, cfg)
            save_outputs(output, depth, meta, result, mask, fit)
        except (ValueError, OSError, KeyError, RuntimeError, ImportError) as exc:
            result = dict(success=False, reason=str(exc), pose=None, processing_ms=None)
        results.append(result)
        row = dict(frame=Path(path).name, success=result['success'], reason=result['reason'],
                   processing_ms=result.get('processing_ms'), x_m='', y_m='', z_m='',
                   axis_x='', axis_y='', axis_z='', plane_rmse_m='', direction_deviation_deg='')
        row['detection_success'] = result.get('detection_success', result['success'])
        if row['detection_success']:
            c = result['measurement'].get('estimated_section_center_m', result['measurement']['surface_center_m'])
            a = result['measurement']['axis_direction']
            centers.append(c)
            axes.append(a)
            row.update(dict(zip(('x_m', 'y_m', 'z_m'), c)))
            row.update(dict(zip(('axis_x', 'axis_y', 'axis_z'), a)))
            row['plane_rmse_m'] = result['measurement']['plane_rmse_m']
        rows.append(row)
    summary = dict(frame_count=len(rows), expected_frames=expected, successful_frames=len(centers),
                   success_rate=len(centers)/max(len(rows), 1), position_std_m=None,
                   max_direction_deviation_deg=None, stability_pass=False,
                   note='Repeated offline frames are not accepted as independent observations. Real accuracy requires measured ground truth.')
    summary['grasp_pose_ready_frames'] = sum(r['success'] for r in results)
    summary['position_definition'] = 'estimated section center from physical thickness; not a calibrated robot grasp pose'
    has_timing = (len(frame_numbers) == len(rows) and len(rows) >= 2
                  and all(isinstance(n, (int, float)) for n in frame_numbers + timestamps))
    contiguous = bool(has_timing and np.all(np.diff(frame_numbers) == 1)
                      and np.all(np.diff(timestamps) > 0))
    summary['sequence_contiguous'] = contiguous
    if has_timing:
        summary['max_frame_number_step'] = int(max(np.diff(frame_numbers)))
        summary['timestamp_interval_median_ms'] = float(np.median(np.diff(timestamps)))
    if len(centers) >= 2:
        a = np.asarray(axes)
        # Axes are undirected: use outer products to avoid +/-180 degree jumps.
        _, vectors = np.linalg.eigh(a.T @ a)
        mean_axis = vectors[:, -1]
        angles = np.degrees(np.arccos(np.clip(np.abs(a @ mean_axis), 0, 1)))
        summary.update(position_std_m=np.std(centers, axis=0, ddof=1).tolist(),
                       max_direction_deviation_deg=float(angles.max()))
        for row, angle in zip([r for r in rows if r['detection_success']], angles):
            row['direction_deviation_deg'] = float(angle)
        summary['stability_pass'] = bool(contiguous and len(rows) == expected and summary['success_rate'] >= cfg['min_success_rate']
            and max(summary['position_std_m']) <= cfg['max_position_std_m']
            and angles.max() <= cfg['max_direction_deviation_deg'])
    timings = [r['processing_ms'] for r in results if r.get('processing_ms') is not None]
    rotations = [r['pose']['rotation_matrix'] for r in results if r.get('pose')]
    summary['max_tool_rotation_deviation_deg'] = None
    summary['grasp_pose_stability_pass'] = False
    if len(rotations) >= 2:
        from scipy.spatial.transform import Rotation
        orientations = Rotation.from_matrix(np.asarray(rotations))
        deviation = np.degrees((orientations.mean().inv()*orientations).magnitude())
        summary['max_tool_rotation_deviation_deg'] = float(max(deviation))
        summary['grasp_pose_stability_pass'] = bool(summary['stability_pass']
            and len(rotations)/max(len(rows), 1) >= cfg['min_success_rate']
            and max(deviation) <= cfg['max_direction_deviation_deg'])
    if timings:
        summary.update(mean_processing_ms=float(np.mean(timings)), p95_processing_ms=float(np.percentile(timings, 95)),
                       estimated_compute_fps=1000/float(np.mean(timings)), timing_scope='detect only, excludes acquisition and file IO')
    with (details/'test_results.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else
                               ['frame', 'success', 'reason', 'processing_ms'])
        writer.writeheader()
        if rows:
            writer.writerows(rows)
    write_json(details/'stability_summary.json', summary)
    # Last-frame outputs must never retain a pose from an earlier successful frame.
    if not results or ('measurement' not in results[-1] and results[-1].get('processing_ms') is None):
        write_json(details/'handle_pose.json', results[-1] if results else
                   dict(success=False, reason='no input frames', pose=None))
        (output/'handle_detection.png').unlink(missing_ok=True)
        for name in ('handle_mask.png', 'depth_visualization.png', 'handle_points.ply'):
            (details/name).unlink(missing_ok=True)
    return summary
