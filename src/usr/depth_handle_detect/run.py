#!/usr/bin/env python3
import os
# Bound CPU use before importing NumPy/SciPy.
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import argparse
import json
from pathlib import Path
import sys
import cv2
from handle_detect.data import capture, load_frame, save_frame, write_json
from handle_detect.detection import detect
from handle_detect.visualization import save_outputs
from handle_detect.stability import evaluate
from handle_detect.synthetic import generate
from handle_detect.paths import detail_directory, raw_directory

ROOT = Path(__file__).resolve().parent

def main():
    p = argparse.ArgumentParser(description='L515 raw-depth handle detection, no robot control')
    p.add_argument('command', choices=['capture', 'detect', 'stability', 'live', 'synthetic', 'import-pnp', 'roi'])
    p.add_argument('--input', type=Path)
    p.add_argument('--output', type=Path, default=ROOT/'outputs')
    p.add_argument('--config', type=Path, default=ROOT/'configs/default.json')
    p.add_argument('--frames', type=int, default=30)
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--serial')
    p.add_argument('--color-preview', action='store_true', help='save native RGB for visual reference only')
    p.add_argument('--min-distance-mm', type=float, help='temporary SDK distance cutoff (>=250 mm); restored on exit')
    a = p.parse_args()
    if a.frames <= 0:
        p.error('--frames must be positive')
    cfg = json.loads(a.config.read_text())
    cv2.setNumThreads(1)
    if a.command in ('capture', 'live'):
        raw = raw_directory(a.output)
        if a.command == 'live' and raw == a.output:
            raw = a.output/'raw'
        def on_frame(d, m):
            result, mask, fit = detect(d, m, cfg)
            save_outputs(a.output, d, m, result, mask, fit)
            print('frame %s: %s (%.1f ms)' % (m['frame_number'], result['reason'], result['processing_ms']), flush=True)
        capture(raw, a.frames, a.width, a.height, a.fps, a.serial,
                on_frame if a.command == 'live' else None, a.color_preview, a.min_distance_mm,
                (lambda d, m: detect(d, m, cfg)) if a.command == 'live' else None)
        if a.command == 'capture':
            print('Saved raw frames to', raw)
            return 0
        paths = sorted(raw.glob('frame_*'))
        summary = evaluate(paths, cfg, a.output, a.frames)
        print(json.dumps(summary, indent=2))
        return 0 if summary['stability_pass'] else 2
    if a.command == 'synthetic':
        destination = raw_directory(a.output)
        generate(destination, a.frames, a.config)
        print('SYNTHETIC data:', destination)
        return 0
    if a.input is None:
        p.error('--input is required')
    if a.command == 'import-pnp':
        # Read original project only; copy native depth and exact matching metadata.
        manifest = json.loads(a.input.read_text())
        source_root = a.input.parents[3]
        for i, frame in enumerate(manifest['frames']):
            raw = frame['metadata']['depth_raw']
            metadata = dict(intrinsics=raw['intrinsics'],
                depth_scale_m_per_unit=raw['depth_scale_m_per_unit'], origin='real_l515_import',
                frame_number=raw['frame_number'], timestamp_ms=raw['timestamp_ms'],
                source_manifest=str(a.input.resolve()), source_frame=frame['frame_id'])
            import numpy as np
            data = np.load(str(source_root/frame['files']['depth_raw']['path']), allow_pickle=False)
            save_frame(raw_directory(a.output)/('frame_%06d' % i), data, metadata)
        print('Imported', len(manifest['frames']), 'native raw depth frames')
        return 0
    if a.command == 'stability':
        summary = evaluate(sorted(a.input.glob('frame_*')), cfg, a.output, a.frames)
        print(json.dumps(summary, indent=2))
        return 0 if summary['stability_pass'] else 2
    d, m = load_frame(a.input)
    if a.command == 'roi':
        # Headless-friendly manual ROI: inspect saved PNG then edit roi_xywh in JSON.
        result = dict(success=False, reason='ROI preview only', pose=None)
        import numpy as np
        save_outputs(a.output, d, m, result, np.zeros(d.shape, np.uint8), None)
        print('Inspect %s; set roi_xywh=[x,y,w,h] in config, depth pixel coordinates.' % (detail_directory(a.output)/'depth_visualization.png'))
        return 0
    result, mask, fit = detect(d, m, cfg)
    save_outputs(a.output, d, m, result, mask, fit)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['success'] else 2

if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, KeyError, OSError, RuntimeError, ImportError) as e:
        # Invalid input must also clear any old grasp artifacts.
        output = Path(sys.argv[sys.argv.index('--output')+1]) if '--output' in sys.argv else ROOT/'outputs'
        output.mkdir(parents=True, exist_ok=True)
        details = detail_directory(output)
        write_json(details/'handle_pose.json', dict(success=False, reason=str(e), pose=None))
        (output/'handle_detection.png').unlink(missing_ok=True)
        for name in ('handle_mask.png', 'depth_visualization.png', 'handle_points.ply'):
            (details/name).unlink(missing_ok=True)
        print('FAIL:', e, file=sys.stderr)
        sys.exit(2)
