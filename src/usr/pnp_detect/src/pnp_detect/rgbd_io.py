"""Lossless RGB-D storage and provenance; exclusively creates new files."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

SCHEMA = 'pnp_detect.rgbd.v1'


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json_new(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    with path.open('x', encoding='utf-8') as stream:
        stream.write(encoded)


def write_png_new(path, array):
    ok, data = cv2.imencode('.png', array)
    if not ok:
        raise ValueError('PNG encoding failed')
    with Path(path).open('xb') as stream:
        stream.write(data.tobytes())


def write_npy_new(path, array):
    with Path(path).open('xb') as stream:
        np.save(stream, array, allow_pickle=False)


def validate_stream(metadata):
    width, height = metadata['resolution_wh']
    intr = metadata['intrinsics']
    if type(width) is not int or type(height) is not int or min(width, height) < 1:
        raise ValueError('Invalid measured stream resolution')
    if [intr['width'], intr['height']] != [width, height] or intr.get('source') != 'active_sdk_stream_profile':
        raise ValueError('Stream intrinsics do not match the actual profile')
    numbers = [intr[k] for k in ['fx', 'fy', 'ppx', 'ppy']] + intr['coefficients']
    if len(intr['coefficients']) != 5 or not np.isfinite(numbers).all() or min(intr['fx'], intr['fy']) <= 0:
        raise ValueError('Missing/nonfinite measured intrinsics')
    if not intr.get('distortion_model') or type(intr.get('distortion_model_id')) is not int:
        raise ValueError('Explicit SDK distortion model required')
    if type(metadata['frame_number']) is not int or metadata['frame_number'] < 0 or not np.isfinite(metadata['timestamp_ms']):
        raise ValueError('Invalid frame counter/timestamp')


def validate_arrays(color, raw, aligned, metadata):
    for name in ['color', 'depth_raw', 'depth_aligned_to_color']:
        validate_stream(metadata[name])
    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3:
        raise ValueError('Native color must be uint8 HxWx3 RGB8')
    for array, name in [(raw, 'depth_raw'), (aligned, 'depth_aligned_to_color')]:
        if array.dtype != np.uint16 or array.ndim != 2:
            raise ValueError(name + ' must be a raw uint16 2D array; color previews are invalid')
        if metadata[name]['resolution_wh'] != list(array.shape[::-1]):
            raise ValueError(name + ' resolution mismatch')
        scale = metadata[name]['depth_scale_m_per_unit']
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError('A positive measured depth scale is required')
    if metadata['color']['resolution_wh'] != [color.shape[1], color.shape[0]] or aligned.shape != color.shape[:2]:
        raise ValueError('Color/aligned grid mismatch')
    if metadata['color']['format'] != 'RGB8' or any(metadata[n]['format'] != 'Z16' for n in ['depth_raw', 'depth_aligned_to_color']):
        raise ValueError('Unexpected source format')
    depth, aligned_metadata = metadata['depth_raw'], metadata['depth_aligned_to_color']
    if not np.isclose(depth['depth_scale_m_per_unit'], aligned_metadata['depth_scale_m_per_unit'], rtol=1e-7):
        raise ValueError('Aligned and original depth scales differ')
    if depth['frame_number'] != aligned_metadata['frame_number'] or depth['timestamp_domain_id'] != aligned_metadata['timestamp_domain_id'] or abs(depth['timestamp_ms']-aligned_metadata['timestamp_ms']) > 1e-5:
        raise ValueError('Aligned depth is not derived from the recorded raw depth frame')
    extr = metadata['depth_to_color']
    r, t = np.asarray(extr['rotation_matrix']), np.asarray(extr['translation_m'])
    if r.shape != (3, 3) or t.shape != (3,) or not np.isfinite(r).all() or not np.isfinite(t).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-4) or abs(np.linalg.det(r)-1) > 1e-4:
        raise ValueError('Invalid measured depth-to-color extrinsics')
    column_major = np.asarray(extr['rotation_column_major'])
    if column_major.shape != (9,) or not np.allclose(column_major.reshape(3, 3, order='F'), r):
        raise ValueError('Extrinsic matrix storage convention mismatch')


def pair_gate(metadata, max_skew_ms, seen):
    color, depth = metadata['color'], metadata['depth_raw']
    if color['timestamp_domain_id'] != depth['timestamp_domain_id']:
        return False, 'incomparable_timestamp_domains', None
    times = np.array([color['timestamp_ms'], depth['timestamp_ms']], dtype=float)
    if not np.isfinite(times).all():
        return False, 'nonfinite_timestamp', None
    skew = abs(float(times[0] - times[1]))
    if skew > max_skew_ms:
        return False, 'timestamp_skew_exceeded', skew
    if color['frame_number'] in seen['color'] or depth['frame_number'] in seen['depth']:
        return False, 'repeated_color_or_depth_frame', skew
    return True, None, skew


def save_pair(project, directory, index, color, raw, aligned, metadata):
    validate_arrays(color, raw, aligned, metadata)
    directory = Path(directory) / ('frame_%06d' % index)
    directory.mkdir(exist_ok=False)
    entries = {}
    for name, array, filename in [('color', color[:, :, ::-1], 'rgb.png'),
                                   ('depth_raw', raw, 'depth_raw.npy'),
                                   ('depth_aligned_to_color', aligned, 'depth_aligned_to_color.npy')]:
        path = directory / filename
        if path.suffix == '.png':
            write_png_new(path, array)
        else:
            write_npy_new(path, array)
        entries[name] = dict(path=path.relative_to(project).as_posix(), sha256=sha256(path))
    return dict(frame_id='frame_%06d' % index, files=entries, metadata=metadata)


def read_session(manifest_path, project):
    manifest = json.loads(Path(manifest_path).read_text(encoding='utf-8'))
    if manifest.get('schema') != SCHEMA or manifest.get('data_origin') != 'real_l515_sdk_capture':
        raise ValueError('Not a recorded L515 RGB-D manifest')
    if not manifest.get('frames'):
        raise ValueError('No accepted RGB-D frames in session')
    if 'L515' not in manifest.get('device', {}).get('name', ''):
        raise ValueError('Session does not identify an L515')
    root = Path(project).resolve()
    seen = {'color': set(), 'depth': set()}
    identifiers = set()
    for frame in manifest['frames']:
        if frame['frame_id'] in identifiers or set(frame['files']) != {'color', 'depth_raw', 'depth_aligned_to_color'}:
            raise ValueError('Duplicate frame or missing raw/color/aligned artifact')
        identifiers.add(frame['frame_id'])
        for entry in frame['files'].values():
            path = (root / entry['path']).resolve()
            if root not in path.parents or sha256(path) != entry['sha256']:
                raise ValueError('Capture path escapes project or file hash mismatch')
        accepted, reason, _ = pair_gate(frame['metadata'], manifest['synchronization']['max_skew_ms'], seen)
        if not accepted:
            raise ValueError('Recorded pair fails synchronization policy: ' + reason)
        seen['color'].add(frame['metadata']['color']['frame_number'])
        seen['depth'].add(frame['metadata']['depth_raw']['frame_number'])
    return manifest


def read_pair(frame, project):
    files = frame['files']
    color_bgr = cv2.imread(str(Path(project) / files['color']['path']), cv2.IMREAD_UNCHANGED)
    if color_bgr is None:
        raise ValueError('Cannot decode recorded RGB PNG')
    color = color_bgr[:, :, ::-1].copy()
    raw = np.load(str(Path(project) / files['depth_raw']['path']), allow_pickle=False)
    aligned = np.load(str(Path(project) / files['depth_aligned_to_color']['path']), allow_pickle=False)
    validate_arrays(color, raw, aligned, frame['metadata'])
    return color, raw, aligned
