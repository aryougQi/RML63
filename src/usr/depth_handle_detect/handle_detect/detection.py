import time
import cv2
import numpy as np
from .data import points_from_pixels
from .geometry import Reject, fit_handle, fit_curved_handle, make_pose, points_in_handle_section


def detect(depth, meta, cfg):
    start = time.perf_counter()
    empty = np.zeros(depth.shape, np.uint8)
    result = dict(success=False, reason='no valid candidate', coordinate_frame='depth_optical',
                  units='metres', source=meta.get('origin', 'unknown'), pose=None,
                  candidate_count=0, diagnostics=[], detection_success=False)
    result.update(frame_number=meta.get('frame_number'), timestamp_ms=meta.get('timestamp_ms'),
                  timestamp_domain=meta.get('timestamp_domain'), device_serial=meta.get('serial'))
    candidates = []
    z = depth.astype(np.float32)*meta['depth_scale_m_per_unit']
    valid = (depth > 0) & (z >= cfg['depth_range_m'][0]) & (z <= cfg['depth_range_m'][1])
    roi = cfg['roi_xywh'] or [0, 0, depth.shape[1], depth.shape[0]]
    x, y, w, h = map(int, roi)
    if x < 0 or y < 0 or w < 3 or h < 3 or x+w > depth.shape[1] or y+h > depth.shape[0]:
        raise ValueError('ROI outside depth image')
    window = np.zeros(depth.shape, bool)
    window[y:y+h, x:x+w] = True
    valid &= window
    raw_valid = valid.copy()
    raw_labels,raw_stats=None,None
    if cfg.get('recover_target_foreground_component',False):
        _,raw_labels,raw_stats,_=cv2.connectedComponentsWithStats(raw_valid.astype(np.uint8),8)
    # Exclude depth discontinuities from segmentation. L515 boundary samples
    # can lie between the handle and its background and broaden the footprint.
    # All fitted 3D points still come from unmodified native depth measurements.
    local_range_limit = cfg.get('segmentation_max_local_depth_range_m')
    if local_range_limit is not None:
        if not np.isfinite(local_range_limit) or local_range_limit <= 0:
            raise ValueError('invalid segmentation local depth range')
        kernel = np.ones((3, 3), np.uint8)
        local_range = cv2.dilate(z, kernel) - cv2.erode(z, kernel)
        valid &= local_range <= local_range_limit
    result['raw_valid_fraction'] = float(np.count_nonzero(depth)/depth.size)
    result['usable_roi_points'] = int(valid.sum())
    # Median is used ONLY to segment, never to fabricate 3D surface measurements.
    filtered = cv2.medianBlur(z, 3)
    step = cfg['depth_band_m'] / 2
    if step <= 0 or not np.isfinite(step):
        raise ValueError('invalid depth band')
    levels = np.unique(np.floor(filtered[valid]/step).astype(int)) if valid.any() else []
    seen = []
    bands = [valid] if cfg.get('include_full_roi_component', False) else []
    bands.extend(valid & (filtered >= level*step) & (filtered <= (level+2)*step) for level in levels)
    for band in bands:
        n, labels, stats, _ = cv2.connectedComponentsWithStats(band.astype(np.uint8), 8)
        for label in range(1, n):
            bx, by, bw, bh, area = stats[label]
            if area < cfg['min_points']:
                continue
            # Clipped ends invalidate physical length/center estimates.
            if bx <= x or by <= y or bx+bw >= x+w or by+bh >= y+h:
                continue
            mask = labels == label
            # Discard duplicate proposals from overlapping depth bands.
            if any(np.count_nonzero(mask & old)/min(area, old.sum()) > 0.75 for old in seen):
                continue
            seen.append(mask)
            v, u = np.nonzero(mask)
            rect = cv2.minAreaRect(np.column_stack((u, v)).astype(np.float32))
            a, b = sorted(rect[1])
            if a < 2 or b/a < 3 or area/max(a*b, 1) < cfg['min_rect_fill']:
                continue
            try:
                # Edge rejection is for segmentation, not for shrinking the
                # physical width. Recover adjacent native samples for the
                # metric fit; its unchanged surface-support tests reject
                # mixed/background depths. No depth values are interpolated.
                measurement_mask = mask
                if local_range_limit is not None:
                    measurement_mask = cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool) & raw_valid
                raw_points = points_from_pixels(depth, meta, measurement_mask)
                if cfg['shape'] == 'curved_rectangular_top_face':
                    core = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
                    fit = fit_curved_handle(points_from_pixels(depth, meta, core), cfg, raw_points)
                else:
                    fit = fit_handle(raw_points, cfg)
                # Rectangular footprint and known metric dimensions reject many rims.
                relation = cfg.get('basket_relation')
                if relation:
                    rx, ry, rw, rh = relation['bbox_xywh']
                    if rw <= 0 or rh <= 0:
                        raise Reject('invalid basket reference box')
                    uv = np.array([u.mean(), v.mean()])
                    offset = (uv-np.array([rx, ry]))/np.array([rw, rh])
                    if np.linalg.norm(offset-relation['expected_center_uv_normalized']) > relation['tolerance_normalized']:
                        raise Reject('basket relative position mismatch')
                    pixel_axis = np.linalg.eigh(np.cov(np.column_stack((u, v)).T))[1][:, -1]
                    theta = np.deg2rad(relation['expected_axis_deg'])
                    delta = np.degrees(np.arccos(np.clip(abs(pixel_axis @ [np.cos(theta), np.sin(theta)]), 0, 1)))
                    if delta > relation['axis_tolerance_deg']:
                        raise Reject('basket relative axis mismatch')
                candidate_mask = mask
                if local_range_limit is not None:
                    # The scene must classify the recovered, fitted surface
                    # as target too; otherwise its edges become obstacles to
                    # the fingers that are supposed to grasp that surface.
                    surface = {tuple(point) for point in fit['points']}
                    measured_inliers = np.fromiter((tuple(point) in surface for point in raw_points), bool, count=len(raw_points))
                    mv, mu = np.nonzero(measurement_mask)
                    candidate_mask = mask.copy()
                    candidate_mask[mv[measured_inliers], mu[measured_inliers]] = True
                    if cfg.get('dimensions_verified') and cfg.get('thickness_m',0)>0:
                        # Include the measured side/boundary of the known wood
                        # section, not just its fitted top-face interior.
                        # Points beyond this bounded 3D section stay obstacles.
                        section_points=points_from_pixels(depth,meta,raw_valid)
                        pixel_size=float(np.median(fit['points'][:,2]))/min(meta['intrinsics']['fx'],meta['intrinsics']['fy'])
                        section=points_in_handle_section(section_points,fit,cfg,pixel_size)
                        sv,su=np.nonzero(raw_valid)
                        before=int(candidate_mask.sum())
                        candidate_mask[sv[section],su[section]]=True
                        fit['section_boundary_points_added']=int(candidate_mask.sum())-before
                    if raw_labels is not None and cfg.get('roi_confirms_handle_identity'):
                        # Use the foreground component for object identity,
                        # while retaining only fitted inliers for metrology.
                        # Mixed boundary returns connected to the confirmed
                        # wood are part of its image mask, not basket obstacles.
                        labels_in_core=np.unique(raw_labels[mask])
                        if len(labels_in_core)==1 and labels_in_core[0]>0:
                            label=int(labels_in_core[0]);rx,ry,rw,rh,_=raw_stats[label]
                            if rx>x and ry>y and rx+rw<x+w and ry+rh<y+h:
                                before=int(candidate_mask.sum())
                                candidate_mask|=(raw_labels==label)
                                fit['foreground_component_points_added']=int(candidate_mask.sum())-before
                candidates.append((candidate_mask, fit))
            except Reject as exc:
                if len(result['diagnostics']) < 12:
                    result['diagnostics'].append(str(exc))
    result['candidate_count'] = len(candidates)
    selected_mask, fit = empty, None
    if len(candidates) > 1:
        result['reason'] = 'ambiguous: multiple geometrically valid candidates'
    elif len(candidates) == 1:
        selected_mask, fit = candidates[0]
        result['measurement'] = dict(surface_center_m=fit['surface_center'].tolist(),
            axis_direction=fit['axis'].tolist(), outward_normal=fit['normal'].tolist(),
            length_m=fit['length'], width_m=fit['width'], plane_rmse_m=fit['rmse'],
            plane_inlier_ratio=fit['inlier_ratio'], point_count=len(fit['points']))
        if 'bow_sag_m' in fit:
            result['measurement'].update(bow_sag_m=fit['bow_sag_m'], fit_model=fit['fit_model'],
                core_inlier_ratio=fit['core_inlier_ratio'],
                residual_definition='normal distance to fitted curved upper face',
                axis_definition='tangent at surface arc midpoint')
            result['measurement'].update(
                surface_centerline_m=fit['centerline'].tolist(),
                centerline_outward_normals=fit['centerline_normals'].tolist(),
                centerline_source='existing quadratic upper-face fit; normals are fitted, not measured side contacts')
        result['measurement']['section_boundary_points_added']=fit.get('section_boundary_points_added',0)
        result['measurement']['foreground_component_points_added']=fit.get('foreground_component_points_added',0)
        try:
            if not ((cfg['roi_xywh'] and cfg['roi_confirms_handle_identity']) or cfg.get('basket_relation')):
                raise Reject('handle identity unverified: supply confirmed ROI or basket relation')
            if cfg['dimensions_verified'] and cfg['thickness_m'] > 0 and cfg['shape'] in (
                    'rectangular_top_face', 'curved_rectangular_top_face'):
                result['detection_success'] = True
                result['measurement']['estimated_section_center_m'] = (
                    fit['surface_center']-fit['normal']*cfg['thickness_m']/2).tolist()
                result['measurement']['dimension_source'] = cfg.get('dimension_source', 'configured physical dimensions')
                result['measurement']['dimension_uncertainty_m'] = cfg.get('dimension_uncertainty_m')
            pose_config = dict(cfg)
            if cfg.get('support_plane', {}).get('enabled', False):
                from .support import fit_support_plane
                support = fit_support_plane(depth, meta, cfg['support_plane'])
                result['support_plane'] = support
                pose_config.update(approach_camera=support['approach_camera'], approach_verified=True)
            result['pose'] = make_pose(fit, pose_config)
            result.update(success=True, reason='ok')
        except Reject as exc:
            result['reason'] = str(exc)
    result['processing_ms'] = (time.perf_counter()-start)*1000
    return result, selected_mask.astype(np.uint8)*255, fit
