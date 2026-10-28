"""Original ribbon-specific planner inspired by GeoGrasp's geometric ideas.

Only local PCA normals, curvature and contact-pair ranking are borrowed as
concepts. No upstream code, PCL/ROS dependency or neural model is used.
Single-view top edges do not establish measured sidewall antipodality.
"""
import time
import numpy as np
from scipy.spatial import cKDTree
from .geometry import Reject, unit, make_pose


def local_features(points, config):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise Reject('invalid GeoGrasp input cloud')
    minimum = config['min_patch_points']
    if len(points) < minimum:
        raise Reject('insufficient GeoGrasp points')
    tree = cKDTree(points)
    k = min(config['neighbor_outlier_k']+1, len(points))
    distances, _ = tree.query(points, k=k)
    points = points[distances[:, -1] <= config['max_neighbor_distance_m']]
    if len(points) < minimum:
        raise Reject('insufficient supported points after outlier rejection')
    k = min(config['normal_neighbors'], len(points))
    distances, indices = cKDTree(points).query(points, k=k)
    neighborhood = points[indices]
    supported = distances <= config['normal_radius_m']
    counts = supported.sum(axis=1)
    means = (neighborhood*supported[..., None]).sum(axis=1)/counts[:, None]
    centered = (neighborhood-means[:, None])*supported[..., None]
    covariance = np.einsum('nki,nkj->nij', centered, centered)/counts[:, None, None]
    values, vectors = np.linalg.eigh(covariance)
    values = np.maximum(values, 0.)
    normals = vectors[:, :, 0]
    normals[np.sum(normals*points, axis=1) > 0] *= -1
    curvature = values[:, 0]/np.maximum(values.sum(axis=1), 1e-15)
    valid = ((counts >= config['min_normal_neighbors']) & (values[:, 1] > 1e-7)
             & (values[:, 1]/np.maximum(values[:, 2], 1e-15) > .03))
    if valid.sum() < minimum:
        raise Reject('degenerate or insufficient normal neighborhoods')
    return points[valid], normals[valid], curvature[valid]


def plan_grasp(points, pose_config, config):
    start = time.perf_counter()
    result = dict(method='simplified_geograsp_ribbon', success=False, reason='no valid local contact pair',
                  pose=None, candidates=[], candidate_only=True, clearance_checked=False,
                  observed_antipodal_verified=False, force_closure_verified=False,
                  contact_model='rectangular-section prior, not measured opposite-sidewall normals')
    try:
        if pose_config['shape'] not in ('rectangular_top_face', 'curved_rectangular_top_face'):
            raise Reject('GeoGrasp ribbon planner requires a rectangular section')
        if not pose_config['dimensions_verified']:
            raise Reject('physical section dimensions unverified')
        weights = config['score_weights']
        if any(not np.isfinite(v) or v < 0 for v in weights.values()) or sum(weights.values()) <= 0:
            raise Reject('invalid candidate score weights')
        if config['normal_neighbors'] < max(3, config['min_normal_neighbors']) or config['normal_radius_m'] <= 0:
            raise Reject('invalid normal neighborhood configuration')
        if any(not np.isfinite(v) or abs(v) > .5 for v in config['candidate_fractions']):
            raise Reject('invalid axial candidate fractions')
        maximum = config.get('gripper_max_opening_m')
        if maximum is not None and (not np.isfinite(maximum) or maximum <= 0):
            raise Reject('invalid gripper opening bound')
        points, normals, curvatures = local_features(points, config)
        origin = np.median(points, axis=0)
        _, _, vt = np.linalg.svd(points-origin, full_matrices=False)
        axis = vt[0]
        if axis[np.argmax(abs(axis))] < 0:
            axis = -axis
        coordinate = (points-origin) @ axis
        lo, hi = np.percentile(coordinate, [1, 99])
        length = float(hi-lo)
        if abs(length-pose_config['length_m']) > pose_config['length_tolerance_m']:
            raise Reject('GeoGrasp cloud length incompatible with physical prior')
        for fraction in config['candidate_fractions']:
            target = (lo+hi)/2+float(fraction)*length
            record = dict(axial_fraction=float(fraction), accepted=False, reason='unprocessed', score=None)
            result['candidates'].append(record)
            try:
                patch_half = config['patch_half_length_m']
                if min(target-lo, hi-target) < config['min_end_margin_m']+patch_half:
                    raise Reject('too close to handle attachment/end')
                chosen = abs(coordinate-target) <= patch_half
                p, n, curvature = points[chosen], normals[chosen], curvatures[chosen]
                if len(p) < config['min_patch_points']:
                    raise Reject('insufficient local surface support')
                median_curvature = float(np.median(curvature))
                if median_curvature > config['max_curvature']:
                    raise Reject('local surface curvature/noise too high')
                normal = unit(np.median(n, axis=0))
                angles = np.degrees(np.arccos(np.clip(n @ normal, -1, 1)))
                deviation = float(np.percentile(angles, 80))
                if deviation > config['max_normal_deviation_deg']:
                    raise Reject('local normals inconsistent')
                tangent = unit(axis-normal*np.dot(axis, normal))
                closing = unit(np.cross(normal, tangent))
                if closing[np.argmax(abs(closing))] < 0:
                    closing = -closing
                # Local coordinates: observed top surface only. Estimate opposing
                # lateral top edges, then infer sidewall midpoints from thickness.
                local_origin = np.median(p, axis=0)
                along = (p-local_origin) @ tangent
                across = (p-local_origin) @ closing
                height = (p-local_origin) @ normal
                # Robust plane height; trim extreme residuals, but require full patch support.
                plane_height = float(np.median(height))
                residual = abs(height-plane_height)
                surface_inliers = residual <= config['patch_inlier_distance_m']
                inlier_ratio = float(surface_inliers.mean())
                if inlier_ratio < config['min_patch_inlier_ratio']:
                    raise Reject('insufficient local plane inliers')
                if surface_inliers.sum() < config['min_patch_points']:
                    raise Reject('insufficient local plane points')
                p, n = p[surface_inliers], n[surface_inliers]
                along, across = along[surface_inliers], across[surface_inliers]
                rmse = float(np.sqrt(np.mean(residual[surface_inliers]**2)))
                if rmse > config['max_patch_rmse_m']:
                    raise Reject('local plane residual too large')
                left, right = np.percentile(across, [5, 95])
                width = float(right-left)
                if abs(width-pose_config['width_m']) > config['width_tolerance_m']:
                    raise Reject('contact span incompatible with known width')
                edge_band = max(.001, width*.2)
                left_support = across <= left+edge_band
                right_support = across >= right-edge_band
                if min(left_support.sum(), right_support.sum()) < config['min_edge_points']:
                    raise Reject('one lateral edge has insufficient support')
                # Both edges need observations along the patch, not isolated tips.
                for side in (left_support, right_support):
                    if np.ptp(along[side]) < patch_half:
                        raise Reject('lateral edge lacks axial coverage')
                at = target-float(np.dot(local_origin-origin, axis))
                projection = float(tangent @ axis)
                if projection < .5:
                    raise Reject('local tangent incompatible with global rod direction')
                at /= projection
                center_support = abs(along-at) <= patch_half/3
                if center_support.sum() < config['min_center_points']:
                    raise Reject('grasp center lacks observed surface support')
                for side in (left_support, right_support):
                    if np.count_nonzero(side & center_support) < config['min_edge_center_points']:
                        raise Reject('contact location lacks observed top-edge support')
                surface = local_origin+tangent*at+closing*(left+right)/2+normal*plane_height
                top_edges = np.vstack((surface-closing*width/2, surface+closing*width/2))
                contacts = top_edges-normal*pose_config['thickness_m']/2
                local_fit = dict(surface_center=surface, axis=tangent, normal=normal)
                pose = make_pose(local_fit, pose_config)
                tool_y = np.asarray(pose['rotation_matrix'])[:, 1]
                if abs(tool_y @ closing) < .9:
                    raise Reject('tool closing direction incompatible with inferred contacts')
                opening = width+2*config['opening_clearance_per_side_m']
                if maximum is not None and opening > maximum:
                    raise Reject('requested opening exceeds configured gripper range')
                components = dict(centrality=max(0., 1-abs(fraction)/.5),
                    flatness=max(0., 1-median_curvature/config['max_curvature']),
                    normal_consistency=max(0., 1-deviation/config['max_normal_deviation_deg']),
                    width_match=max(0., 1-abs(width-pose_config['width_m'])/config['width_tolerance_m']),
                    edge_balance=min(left_support.sum(), right_support.sum())/max(left_support.sum(), right_support.sum()),
                    residual=max(0., 1-rmse/config['max_patch_rmse_m']))
                score = float(sum(weights[k]*components[k] for k in weights)/sum(weights.values()))
                record.update(accepted=True, reason='model-assisted local contact candidate', score=score,
                    score_components=components, pose=pose, surface_center_m=surface.tolist(),
                    local_axis=tangent.tolist(), surface_normal=normal.tolist(),
                    inferred_contacts_m=contacts.tolist(), observed_top_edge_estimates_m=top_edges.tolist(),
                    contacts_source='inferred sidewall midpoints from top edges and configured thickness',
                    width_m=width, recommended_opening_m=float(opening),
                    gripper_opening_verified=maximum is not None, patch_point_count=len(p),
                    median_curvature=median_curvature, normal_deviation_p80_deg=deviation,
                    patch_rmse_m=rmse,
                    patch_inlier_ratio=inlier_ratio,
                    observed_normal_alignment_to_closing=float(np.median(abs(n @ closing))))
            except Reject as exc:
                record['reason'] = str(exc)
        accepted = [c for c in result['candidates'] if c['accepted']]
        result.update(input_point_count=len(points), valid_candidate_count=len(accepted))
        if not accepted:
            raise Reject('all GeoGrasp candidates rejected')
        best = max(accepted, key=lambda c: (c['score'], -abs(c['axial_fraction'])))
        result.update(success=True, reason='ok: model-assisted geometric candidate',
                      selected_candidate=best, pose=best['pose'])
    except (Reject, KeyError, np.linalg.LinAlgError) as exc:
        result['reason'] = str(exc)
    result['planning_ms'] = (time.perf_counter()-start)*1000
    return result
