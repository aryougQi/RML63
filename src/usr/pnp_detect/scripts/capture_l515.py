#!/usr/bin/env python3
"""Standalone real L515 SDK probe/capture. No ROS, no robot, no fake fallback."""
import argparse
from collections import Counter
import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import yaml
from pnp_detect.rgbd_io import SCHEMA, pair_gate, save_pair, write_json_new


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', action='store_true', help='Enumerate SDK/device/profiles without starting streams')
    parser.add_argument('--report', type=Path, help='New JSON under reports/ (probe only)')
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/rgbd_capture.yaml')
    parser.add_argument('--session', help='New session directory name (not a path)')
    parser.add_argument('--frames', type=int, default=30)
    parser.add_argument('--serial')
    parser.add_argument('--sdk-library', type=Path,
                        default=Path.home() / '.local/opt/realsense-l515/lib/librealsense2.so.2.50.0')
    args = parser.parse_args()
    if args.frames < 1 or args.frames > 10000:
        parser.error('--frames must be 1..10000')
    library = args.sdk_library.expanduser().resolve()
    # Avoid a conflicting third-party libusb (e.g. MVS). Relaunch before CDLL loading.
    if library.is_file() and os.environ.get('PNP_RGBD_LIBRARY_ENV_READY') != str(library):
        env = dict(os.environ, PNP_RGBD_LIBRARY_ENV_READY=str(library))
        env['LD_LIBRARY_PATH'] = str(library.parent) + ':/usr/lib/x86_64-linux-gnu'
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve())] + sys.argv[1:], env)
    if args.report and ROOT / 'reports' not in args.report.resolve().parents:
        parser.error('--report must be a new file under the project reports/')
    if args.report and args.report.exists():
        raise FileExistsError(args.report)
    from pnp_detect.realsense_c import RealSense
    reader = None
    if args.probe:
        result = dict(probe_only=True, streaming_tested=False, sdk_usable=False, robot_ready=False)
        try:
            reader = RealSense(library)
            result.update(reader.probe(), sdk_usable=True)
            result['l515_present'] = any('L515' in d.get('name', '') for d in result['devices'])
        except (ValueError, OSError, RuntimeError) as error:
            result['error'] = str(error)
        finally:
            if reader:
                reader.close()
        if args.report:
            write_json_new(args.report, result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get('l515_present') else 2
    if args.report:
        parser.error('--report is only for --probe; capture metadata goes to reports/rgbd/SESSION/manifest.json')
    session = args.session or datetime.datetime.now(datetime.timezone.utc).strftime('l515_%Y%m%dT%H%M%S_%fZ')
    if not session or session in ('.', '..') or '/' in session or '\\' in session:
        parser.error('--session must be a directory name, not a path')
    data_dir, report_dir = ROOT / 'data/rgbd' / session, ROOT / 'reports/rgbd' / session
    if data_dir.exists() or report_dir.exists():
        raise FileExistsError('Session already exists; refusing overwrite: ' + session)
    config_bytes = args.config.read_bytes()
    config = yaml.safe_load(config_bytes)
    for name in ['fps', 'frame_timeout_ms', 'capture_timeout_seconds', 'max_pair_skew_ms']:
        if not isinstance(config.get(name), (int, float)) or not 0 < config[name] < float('inf'):
            raise ValueError('Invalid capture config: ' + name)
    for name in ['depth_resolution_wh', 'color_resolution_wh']:
        if len(config[name]) != 2 or any(type(n) is not int or n < 1 for n in config[name]):
            raise ValueError('Invalid resolution: ' + name)
    if type(config['fps']) is not int or type(config['warmup_frames']) is not int or config['warmup_frames'] < 0:
        raise ValueError('fps/warmup_frames must be integer and nonnegative')
    data_dir.mkdir(parents=True)
    report_dir.mkdir(parents=True)
    result = dict(schema=SCHEMA, data_origin='real_l515_sdk_capture', session_id=session,
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), status='incomplete',
        requested_config=config, config_sha256=hashlib.sha256(config_bytes).hexdigest(),
        synchronization=dict(method='SDK composite frameset + comparable-domain timestamp gate',
                             hardware_trigger_synchronization_verified=False, max_skew_ms=config['max_pair_skew_ms']),
        alignment=dict(method='librealsense align depth to color', target_grid='native color pixels',
                       stored_values='original depth-sensor axial depth units reprojected to color grid',
                       hole_filling=False, smoothing=False, raw_depth_preserved=True),
        frames=[], rejected_pairs={}, robot_ready=False, real_object_pose=None,
        physical_scene='UNVERIFIED: no assertion that a basket or handle is present')
    seen, rejected = {'color': set(), 'depth': set()}, Counter()
    code = 2
    try:
        reader = RealSense(library)
        result.update(reader.start(config, args.serial))
        for _ in range(config['warmup_frames']):
            reader.next_pair(config['frame_timeout_ms'])
        deadline = time.monotonic() + config['capture_timeout_seconds']
        while len(result['frames']) < args.frames and time.monotonic() < deadline:
            color, depth, aligned, metadata = reader.next_pair(config['frame_timeout_ms'])
            accepted, reason, skew = pair_gate(metadata, config['max_pair_skew_ms'], seen)
            if not accepted:
                rejected[reason] += 1
                continue
            metadata['pair_timestamp_skew_ms'] = skew
            record = save_pair(ROOT, data_dir, len(result['frames']), color, depth, aligned, metadata)
            result['frames'].append(record)
            seen['color'].add(metadata['color']['frame_number'])
            seen['depth'].add(metadata['depth_raw']['frame_number'])
        if len(result['frames']) != args.frames:
            raise RuntimeError('Capture timeout: only %d/%d accepted pairs' % (len(result['frames']), args.frames))
        result['status'] = 'captured'
        code = 0
    except (Exception, KeyboardInterrupt) as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        if reader:
            reader.close()
        result['rejected_pairs'] = dict(rejected)
        result['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        manifest = report_dir / 'manifest.json'
        write_json_new(manifest, result)
        print('Capture status: %s; accepted pairs: %d; report: %s' % (result['status'], len(result['frames']), manifest))
        if 'error' in result:
            print(result['error'], file=sys.stderr)
    return code


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
