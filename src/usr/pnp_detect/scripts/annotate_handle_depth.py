#!/usr/bin/env python3
"""Export an offline manual polygon annotation page. Does not detect handles."""
import argparse
import base64
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from pnp_detect.rgbd_io import read_session, sha256


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--frame', type=int, default=0)
    parser.add_argument('--html', type=Path, required=True, help='New temporary HTML under /tmp')
    args = parser.parse_args()
    target = args.html.resolve()
    if Path('/tmp') not in target.parents:
        parser.error('Annotation HTML is a debug artifact; save it under /tmp')
    manifest = read_session(args.manifest, ROOT)
    if not 0 <= args.frame < len(manifest['frames']):
        parser.error('Invalid frame index')
    frame = manifest['frames'][args.frame]
    metadata = dict(schema='pnp_detect.manual_handle_regions.v1', source='manual',
                    session_manifest_sha256=sha256(args.manifest), reference_frame_id=frame['frame_id'],
                    reference_rgb_sha256=frame['files']['color']['sha256'],
                    image_resolution_wh=frame['metadata']['color']['resolution_wh'],
                    coordinate_frame='native_color_pixels', scene_static_confirmed=False,
                    reference_handle_axial_depth_m=None, reference_depth_source=None)
    image = base64.b64encode((ROOT / frame['files']['color']['path']).read_bytes()).decode('ascii')
    template = (ROOT / 'scripts/handle_depth_annotation.html').read_text(encoding='utf-8')
    html = template.replace('__METADATA__', json.dumps(metadata)).replace('__IMAGE__', image)
    with target.open('x', encoding='utf-8') as stream:
        stream.write(html)
    print('MANUAL annotation page:', target)
    print('Export JSON into reports/rgbd/%s/manual_regions.json (do not overwrite an existing annotation).' % manifest['session_id'])
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
