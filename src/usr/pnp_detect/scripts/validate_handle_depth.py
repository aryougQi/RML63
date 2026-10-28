#!/usr/bin/env python3
"""Offline manual handle-region quality metrics; never infer a grasp/6D pose."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import cv2
import numpy as np
import yaml
from pnp_detect.rgbd_io import read_session, read_pair, sha256, write_json_new, write_png_new
from pnp_detect.handle_depth_quality import validate_annotation, evaluate_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--annotation', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/handle_depth_quality.yaml')
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New final annotated PNG under outputs/')
    args = parser.parse_args()
    if ROOT / 'reports' not in args.report.resolve().parents or ROOT / 'outputs' not in args.output.resolve().parents or args.output.suffix.lower() != '.png':
        parser.error('JSON must be under reports/; final PNG must be under outputs/')
    if args.output.exists() or args.report.exists():
        raise FileExistsError('Refusing overwrite; choose new report and PNG names')
    manifest = read_session(args.manifest, ROOT)
    annotation = json.loads(args.annotation.read_text(encoding='utf-8'))
    masks = validate_annotation(annotation, args.manifest, manifest)
    raw_config = args.config.read_bytes()
    config = yaml.safe_load(raw_config)
    result = evaluate_session(manifest, annotation, masks, ROOT, config)
    result.update(schema='pnp_detect.handle_depth_quality.v1',
        manifest_sha256=sha256(args.manifest), annotation_sha256=sha256(args.annotation),
        configuration=config, configuration_sha256=hashlib.sha256(raw_config).hexdigest(),
        result_kind='measured_depth_diagnostics_with_manual_2d_regions',
        pose=None, verified_grasp_success_rate=None)
    reference = next(f for f in manifest['frames'] if f['frame_id'] == annotation['reference_frame_id'])
    color, _, _ = read_pair(reference, ROOT)
    final = color[:, :, ::-1].copy()
    for key, color_bgr in [('handle_polygon', (0, 0, 255)), ('background_polygon', (255, 100, 0))]:
        if annotation.get(key):
            cv2.polylines(final, [np.rint(annotation[key]).astype(np.int32)], True, color_bgr, 2)
    text = 'MANUAL ROI / DEPTH DIAGNOSTICS / NOT GRASP VALIDATED'
    cv2.rectangle(final, (0, 0), (final.shape[1]-1, 22), (0, 0, 0), -1)
    cv2.putText(final, text, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, .38, (255, 255, 255), 1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_png_new(args.output, final)
    result['final_annotation_png'] = str(args.output.resolve().relative_to(ROOT))
    write_json_new(args.report, result)
    print('Manual depth diagnostics:', args.report)
    print('Final annotated PNG:', args.output)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
