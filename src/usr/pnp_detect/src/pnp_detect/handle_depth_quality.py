"""Manual-region depth diagnostics, not automatic handle detection or grasp validation."""
import numpy as np
import cv2

from .rgbd_io import sha256, read_pair


def polygon_mask(points, resolution):
    width, height = resolution
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1:] != (2,) or len(p) < 3 or not np.isfinite(p).all():
        raise ValueError('Manual polygon requires >=3 finite [x,y] vertices')
    if len(np.unique(p, axis=0)) != len(p) or (p < 0).any() or (p[:, 0] >= width).any() or (p[:, 1] >= height).any():
        raise ValueError('Duplicate or out-of-image polygon vertices')
    def cross(a, b, c):
        u, v = b-a, c-a
        return u[0]*v[1]-u[1]*v[0]
    def intersects(a, b, c, d):
        if max(min(a[0], b[0]), min(c[0], d[0])) > min(max(a[0], b[0]), max(c[0], d[0])):
            return False
        if max(min(a[1], b[1]), min(c[1], d[1])) > min(max(a[1], b[1]), max(c[1], d[1])):
            return False
        return cross(a,b,c)*cross(a,b,d) <= 0 and cross(c,d,a)*cross(c,d,b) <= 0
    for i in range(len(p)):
        for j in range(i+1, len(p)):
            if j == i+1 or (i == 0 and j == len(p)-1):
                continue
            if intersects(p[i], p[(i+1)%len(p)], p[j], p[(j+1)%len(p)]):
                raise ValueError('Self-intersecting manual polygon')
    if abs(cv2.contourArea(p.astype(np.float32))) < 1:
        raise ValueError('Degenerate manual polygon')
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(p).astype(np.int32)], 1)
    return mask.astype(bool)


def validate_annotation(annotation, manifest_path, manifest):
    if annotation.get('schema') != 'pnp_detect.manual_handle_regions.v1' or annotation.get('source') != 'manual':
        raise ValueError('Explicit source=manual annotation required; no automatic ROI fallback')
    if annotation.get('session_manifest_sha256') != sha256(manifest_path):
        raise ValueError('Annotation is bound to a different session manifest')
    frames = {f['frame_id']: f for f in manifest['frames']}
    reference = frames.get(annotation.get('reference_frame_id'))
    if reference is None or annotation.get('reference_rgb_sha256') != reference['files']['color']['sha256']:
        raise ValueError('Annotation reference frame/hash mismatch')
    resolution = reference['metadata']['color']['resolution_wh']
    if annotation.get('image_resolution_wh') != resolution or annotation.get('coordinate_frame') != 'native_color_pixels':
        raise ValueError('Annotation requires original color resolution and coordinate frame')
    if type(annotation.get('scene_static_confirmed')) is not bool:
        raise ValueError('scene_static_confirmed must explicitly be true or false')
    handle = polygon_mask(annotation['handle_polygon'], resolution)
    background = None
    if annotation.get('background_polygon'):
        background = polygon_mask(annotation['background_polygon'], resolution)
        if np.any(handle & background):
            raise ValueError('Handle/background manual regions overlap')
    truth = annotation.get('reference_handle_axial_depth_m')
    if truth is not None:
        if not np.isfinite(truth) or truth <= 0 or not str(annotation.get('reference_depth_source', '')).strip():
            raise ValueError('An independent axial depth reference needs a positive value and source')
    return handle, background


def depth_statistics(depth, mask, scale, config, reference=None):
    if depth.dtype != np.uint16 or depth.ndim != 2 or depth.shape != mask.shape:
        raise ValueError('Expected unmodified uint16 aligned depth, not an 8-bit/color preview')
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('Missing real depth units')
    raw = depth[mask]
    if raw.size == 0:
        raise ValueError('Empty manual region')
    z = raw.astype(float) * scale
    valid = (raw > 0) & (z >= config['min_depth_m']) & (z <= config['max_depth_m'])
    values = z[valid]
    result = dict(region_pixel_count=int(raw.size), zero_missing_count=int((raw == 0).sum()),
                  valid_count=int(valid.sum()), valid_fraction=float(valid.mean()),
                  zero_missing_fraction=float((raw == 0).mean()),
                  nonzero_out_of_range_count=int(((raw > 0) & ~valid).sum()),
                  median_depth_m=None, spatial_std_mm=None, p10_p90_depth_m=None,
                  reference_comparison=None)
    if values.size:
        result.update(median_depth_m=float(np.median(values)), spatial_std_mm=float(values.std() * 1000),
                      p10_p90_depth_m=np.percentile(values, [10, 90]).tolist())
        if reference is not None:
            result['reference_comparison'] = dict(reference_axial_depth_m=reference,
                median_absolute_error_mm=float(abs(np.median(values)-reference)*1000),
                fraction_within_reference_tolerance=float((np.abs(values-reference) <= config['reference_tolerance_m']).mean()),
                tolerance_m=config['reference_tolerance_m'])
    return result


def temporal_statistics(depths_m, min_observations):
    values = np.asarray(depths_m, dtype=float)
    observations = np.isfinite(values).sum(axis=0)
    eligible = observations >= min_observations
    if not eligible.any():
        return dict(eligible_pixel_count=0, per_pixel_std_median_mm=None, per_pixel_std_p95_mm=None)
    deviations = np.nanstd(values[:, eligible], axis=0) * 1000
    return dict(eligible_pixel_count=int(eligible.sum()), per_pixel_std_median_mm=float(np.median(deviations)),
                per_pixel_std_p95_mm=float(np.percentile(deviations, 95)))


def evaluate_session(manifest, annotation, masks, project, config):
    handle, background = masks
    if not 0 <= config['min_depth_m'] < config['max_depth_m'] < float('inf'):
        raise ValueError('Invalid common depth range')
    if not 0 < config['reference_tolerance_m'] < float('inf') or type(config['min_temporal_observations']) is not int or config['min_temporal_observations'] < 2:
        raise ValueError('Invalid common quality policy')
    static = annotation['scene_static_confirmed']
    selected = manifest['frames'] if static else [f for f in manifest['frames'] if f['frame_id'] == annotation['reference_frame_id']]
    result = dict(annotation_source='manual', evaluated_frame_count=len(selected),
        scene_static_confirmed_by_operator=static, frames=[], temporal=None,
        automatic_handle_detection=False, handle_surface_depth_identity_verified=False,
        absolute_depth_accuracy_verified=False, depth_validated_for_grasp=False, robot_ready=False,
        limitations=['A manual RGB region is 2D region truth, not ground-truth 3D handle depth.',
                     'SDK alignment may duplicate samples and leaves holes; aligned pixels are not independent rays.',
                     'Spatial variation is not sensor noise. Temporal statistics require a static scene.',
                     'Depth on the basket bottom cannot be excluded without independent surface evidence.'])
    stack = []
    for frame in selected:
        color, raw, aligned = read_pair(frame, project)
        scale = frame['metadata']['depth_raw']['depth_scale_m_per_unit']
        if frame['metadata']['color']['resolution_wh'] != annotation['image_resolution_wh']:
            raise ValueError('Stream resolution changed; do not reuse a manual region')
        stats = depth_statistics(aligned, handle, scale, config, annotation.get('reference_handle_axial_depth_m'))
        background_stats = depth_statistics(aligned, background, scale, config) if background is not None else None
        raw_nonzero = float((raw > 0).mean())
        item = dict(frame_id=frame['frame_id'], manual_handle=stats, manual_background=background_stats,
                    native_depth_whole_frame_nonzero_fraction=raw_nonzero,
                    possible_background_contamination=None)
        z = aligned[handle].astype(float) * scale
        z[(aligned[handle] == 0) | (z < config['min_depth_m']) | (z > config['max_depth_m'])] = np.nan
        stack.append(z)
        if background_stats and background_stats['median_depth_m'] is not None and stats['median_depth_m'] is not None:
            bg = background_stats['median_depth_m']
            valid = z[np.isfinite(z)]
            item['possible_background_contamination'] = dict(
                handle_minus_background_median_mm=(stats['median_depth_m']-bg)*1000,
                fraction_near_background_depth=float((np.abs(valid-bg) <= config['reference_tolerance_m']).mean()),
                interpretation='Diagnostic only: similar depth does not prove which surface was measured.')
        result['frames'].append(item)
    if static:
        result['temporal'] = temporal_statistics(stack, config['min_temporal_observations'])
    else:
        result['limitations'].append('Only the annotated reference frame evaluated: static scene was not confirmed.')
    return result
