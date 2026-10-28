#!/usr/bin/env python3
"""Report actual raw coverage and timestamps; never estimate a grasp pose."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
from handle_detect.data import load_frame, points_from_pixels, write_json
from handle_detect.visualization import save_outputs
from handle_detect.paths import detail_directory


def inspect(directory, config, output):
    paths = sorted(Path(directory).glob('frame_*'))
    if not paths:
        raise ValueError('no captured frames')
    rows, stamps, numbers, coverages = [], [], [], []
    result = None
    for path in paths:
        d, m = load_frame(path)
        x, y, w, h = config['roi_xywh']
        if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > d.shape[1] or y+h > d.shape[0]:
            raise ValueError('invalid diagnostic ROI')
        z = d.astype(float)*m['depth_scale_m_per_unit']
        mask = np.zeros(d.shape, bool)
        mask[y:y+h, x:x+w] = True
        mask &= (d > 0) & (z >= config['depth_range_m'][0]) & (z <= config['depth_range_m'][1])
        selected = z[mask]
        coverage = float(np.mean(d > 0))
        coverages.append(coverage)
        stamps.append(m['timestamp_ms'])
        numbers.append(m['frame_number'])
        rows.append(dict(frame=path.name, raw_valid_fraction=coverage,
                         foreground_points=int(mask.sum()),
                         foreground_depth_median_m=float(np.median(selected)) if len(selected) else None))
    times = np.diff(stamps)
    gaps = np.diff(numbers)
    report = dict(frame_count=len(paths), unique_depth_frame_numbers=len(set(numbers)),
        timestamp_domain=m.get('timestamp_domain'), min_frame_step=int(min(gaps)) if len(gaps) else None,
        max_frame_step=int(max(gaps)) if len(gaps) else None,
        timestamp_interval_median_ms=float(np.median(times)) if len(times) else None,
        timestamp_interval_max_ms=float(max(times)) if len(times) else None,
        observed_depth_fps=float(1000/np.median(times)) if len(times) and min(times) > 0 else None,
        raw_valid_fraction_mean=float(np.mean(coverages)),
        mean_foreground_points=float(np.mean([r['foreground_points'] for r in rows])),
        usb_type=m.get('usb_type'), sensor_options=m.get('sensor_options'),
        selection=dict(roi_xywh=config['roi_xywh'], depth_range_m=config['depth_range_m']),
        meaning='Depth coverage diagnostic only. Foreground mask is NOT a geometrically accepted handle. No pose or stability claim.',
        frames=rows)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    details = detail_directory(output)
    write_json(details/'capture_quality.json', report)
    # Preview last frame using exactly the raw depth grid used for inspection.
    result = dict(success=False, reason='UNVERIFIED depth foreground; geometry not accepted',
                  pose=None, mask_semantics='diagnostic_depth_foreground_only')
    save_outputs(output, d, m, result, mask.astype(np.uint8)*255, None)
    pts = points_from_pixels(d, m, mask)
    with (details/'candidate_points_raw.ply').open('w') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n' % len(pts))
        np.savetxt(f, pts, fmt='%.6f')
    rgb = paths[-1]/'rgb.png'
    if rgb.exists():
        import shutil
        shutil.copy2(str(rgb), str(details/'rgb_reference.png'))
    print(json.dumps({k:v for k,v in report.items() if k != 'frames'}, indent=2))
    return report

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    inspect(a.input, json.loads(a.config.read_text()), a.output)
