"""Estimate vertical approach from a user-confirmed horizontal tabletop ROI."""
import numpy as np
from .data import points_from_pixels
from .geometry import Reject


def fit_support_plane(depth, metadata, config):
    if not config.get('horizontal_confirmed', False):
        raise Reject('horizontal tabletop reference not confirmed')
    x, y, w, h = config['roi_xywh']
    if min(x, y) < 0 or min(w, h) < 3 or x+w > depth.shape[1] or y+h > depth.shape[0]:
        raise Reject('support-plane ROI outside depth grid')
    z = depth.astype(float)*metadata['depth_scale_m_per_unit']
    mask = np.zeros(depth.shape, bool)
    mask[y:y+h, x:x+w] = True
    lo, hi = config.get('depth_range_m', [.25, 2.])
    mask &= (depth > 0) & (z >= lo) & (z <= hi)
    points = points_from_pixels(depth, metadata, mask)
    if len(points) < config.get('min_points', 300):
        raise Reject('insufficient support-plane depth')
    rng = np.random.default_rng(71)
    sample = points if len(points) < 1000 else points[rng.choice(len(points), 1000, replace=False)]
    threshold = config.get('inlier_m', .003)
    best = np.zeros(len(sample), bool)
    for _ in range(60):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        norm = np.linalg.norm(normal)
        if norm < 1e-10:
            continue
        normal /= norm
        inside = abs((sample-a) @ normal) < threshold
        if inside.sum() > best.sum():
            best = inside
    minimum = config.get('min_inlier_ratio', .9)
    if best.mean() < minimum:
        raise Reject('tabletop plane support too low')
    center = sample[best].mean(0)
    _, singular, vt = np.linalg.svd(sample[best]-center, full_matrices=False)
    if singular[1] < .005:
        raise Reject('degenerate tabletop patch')
    normal = vt[-1]
    inside = abs((points-center) @ normal) < threshold
    if inside.mean() < minimum:
        raise Reject('tabletop full-cloud support too low')
    p = points[inside]
    center = p.mean(0)
    _, _, vt = np.linalg.svd(p-center, full_matrices=False)
    normal = vt[-1]
    if normal @ center > 0:
        normal = -normal
    rmse = float(np.sqrt(np.mean(((p-center) @ normal)**2)))
    if rmse > config.get('max_rmse_m', .0015):
        raise Reject('tabletop plane residual too large')
    return dict(approach_camera=(-normal).tolist(), outward_normal=normal.tolist(),
                plane_center_m=center.tolist(), rmse_m=rmse,
                inlier_ratio=float(inside.mean()), point_count=len(p),
                source='depth-fitted normal of user-confirmed horizontal tabletop ROI')
