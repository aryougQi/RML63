"""Contact evidence is independent of IK. Missing sides never become observations."""
from collections import Counter
import numpy as np
from grasp_candidates import unit
from grasp_geometry import raised_pad_grasp


def antipodal(q_left, q_right, inward_left, inward_right, friction):
    """Normals point INTO the object, so ideal left normal points left->right."""
    if not np.isfinite(friction) or friction < 0:
        raise ValueError('invalid friction coefficient')
    v = unit(np.asarray(q_right)-q_left)
    quality = min(float(unit(inward_left) @ v), float(-unit(inward_right) @ v))
    return quality, quality >= 1/np.sqrt(1+friction**2)


def evaluate(candidate, cfg, gripper, obstacles=None, observed_contacts=None, perturb=None):
    c = candidate
    t = c.tcp.copy()
    if perturb is not None:
        t[:3, 3] += perturb[:3]
        from scipy.spatial.transform import Rotation
        from grasp_geometry import rotation_matrix
        t[:3, :3] = t[:3, :3] @ rotation_matrix(Rotation.from_rotvec(perturb[3:]))
    reasons = []
    open_width = cfg['open_width_m']
    closing = t[:3, 1]
    across = unit(np.cross(c.normal, c.tangent))
    # Project the rectangular section onto the closure direction.
    opening = c.width_m*abs(closing @ across)+c.thickness_m*abs(closing @ c.normal)
    try:
        geometry = gripper.at_width(opening)
        if opening+2*cfg['min_contact_margin_m'] > open_width:
            reasons.append('insufficient_opening')
        gripper.at_width(open_width)
        raised_pad_grasp(t, c.tcp[:3, 3], c.normal, c.thickness_m,
                         geometry['pad_bounds_tcp_m'], 0, cfg['min_contact_margin_m'])
    except ValueError:
        reasons.append('CAD_pad_coverage_or_opening')
    displacement = t[:3, 3]-c.tcp[:3, 3]
    if abs(displacement @ closing) > cfg['min_contact_margin_m']:
        reasons.append('lateral_contact_alignment')
    end_clearance = c.end_clearance_m-abs(displacement @ c.tangent)
    if end_clearance < cfg['end_margin_m']+cfg['min_contact_margin_m']:
        reasons.append('end_margin')
    if abs(closing @ c.tangent) > cfg['max_closure_axis_dot']:
        reasons.append('closure_axis')
    if (t[2, 2] > -np.cos(np.radians(cfg['max_downward_angle_deg']))
            or -t[:3, 2] @ c.normal < np.cos(np.radians(cfg['max_face_angle_deg']))):
        reasons.append('approach_angle')
    if c.rmse_m > cfg['max_rmse_m']:
        reasons.append('fit_quality')
    source = 'missing'; confidence = 0.; anti = None
    if observed_contacts is not None:
        if observed_contacts.get('source') != 'measured' or len(observed_contacts.get('points', [])) != 2:
            reasons.append('invalid_measured_contact_evidence')
        else:
            anti, feasible = antipodal(*observed_contacts['points'], *observed_contacts['inward_normals'], cfg['friction_coefficient'])
            source = 'measured'; confidence = float(observed_contacts['confidence'])
            if not np.isfinite(confidence) or not 0 <= confidence <= 1:
                raise ValueError('invalid measured contact confidence')
            points = np.asarray(observed_contacts['points'], float)
            local = (points-t[:3, 3]) @ t[:3, :3]
            if (abs(unit(points[1]-points[0]) @ closing) < .95
                    or np.max(abs(local[:, 0])) > c.end_clearance_m
                    or np.linalg.norm(points.mean(0)-c.tcp[:3, 3]) > c.thickness_m):
                reasons.append('contact_points_outside_expected_region')
            if not feasible: reasons.append('outside_friction_cones')
    elif cfg['allow_section_inference']:
        # Rectangular side faces are model estimates, never measured back-side normals.
        ql = c.tcp[:3, 3]-across*c.width_m/2
        qr = c.tcp[:3, 3]+across*c.width_m/2
        anti, feasible = antipodal(ql, qr, across, -across, cfg['friction_coefficient'])
        # A tilted jaw must also be compatible with those model side-face normals.
        anti = min(anti, abs(float(closing @ across)))
        if anti < 1/np.sqrt(1+cfg['friction_coefficient']**2): reasons.append('outside_friction_cones')
        source = 'inferred_rectangular_section'; confidence = cfg['section_model_confidence']
    else:
        reasons.append('contact_geometry_missing')
    clearance = None
    if obstacles is not None:
        points = np.asarray(obstacles, float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('invalid obstacle points')
        if len(points):
            # CAD triangle vertices, conservative nearest-sample clearance. Full swept
            # mesh/scene collision remains an independent MTC planning constraint.
            from grasp_geometry import flange_target
            distances = []
            for width in (opening, open_width):
                g = gripper.at_width(width)
                flange = flange_target(t, g['flange_tcp'])
                for name, triangles in gripper.meshes.items():
                    link = flange @ gripper.link(name, g['linkage_angle_rad'])
                    local = (points-link[:3, 3]) @ link[:3, :3]
                    # AABB broad phase deliberately rejects possible mesh collisions.
                    mesh = triangles['vertices'].reshape(-1, 3)
                    delta = np.maximum(np.maximum(mesh.min(0)-local, local-mesh.max(0)), 0)
                    distances.append(float(np.linalg.norm(delta, axis=1).min()))
            clearance = min(distances)
            if clearance < cfg['scene_margin_m']: reasons.append('observed_obstacle_near_CAD')
    # Unknown clearance receives no reward, not a fabricated free-space measurement.
    qc = 0. if clearance is None else float(np.clip(clearance/cfg['clearance_scale_m'], 0, 1))
    qg = float(np.clip((open_width-opening)/(open_width), 0, 1))*float(np.clip(1-c.rmse_m/cfg['max_rmse_m'], 0, 1))
    return dict(id=c.id, accepted=not reasons, reasons=reasons, contact_source=source,
                contact_confidence=confidence, antipodal=anti, clearance_m=clearance,
                strict_antipodal_verified=source == 'measured' and not reasons,
                components=dict(anti=0. if anti is None else max(0., anti)*confidence,
                                clearance=qc, geometry=qg, robust=0.))


def rank(candidates, cfg, gripper, obstacles=None):
    if not 0 <= cfg['section_model_confidence'] <= 1:
        raise ValueError('invalid section-model confidence')
    if cfg['robust_trials'] < 0 or min(cfg['robust_position_m'], cfg['robust_angle_deg']) < 0:
        raise ValueError('invalid robustness configuration')
    weights = cfg['score_weights']
    if set(weights) != {'anti', 'clearance', 'geometry', 'robust'} or any(not np.isfinite(w) or w < 0 for w in weights.values()) or sum(weights.values()) <= 0:
        raise ValueError('invalid normalized score weights')
    rng = np.random.RandomState(cfg['robust_seed'])
    perturbations = rng.uniform(-1, 1, (cfg['robust_trials'], 6))
    perturbations[:, :3] *= cfg['robust_position_m']
    perturbations[:, 3:] *= np.radians(cfg['robust_angle_deg'])
    results = []
    for c in candidates:
        result = evaluate(c, cfg, gripper, obstacles)
        failures = Counter(); passed = 0
        if result['accepted']:
            for p in perturbations:
                trial = evaluate(c, cfg, gripper, obstacles, perturb=p)
                passed += int(trial['accepted']); failures.update(trial['reasons'])
        trials = len(perturbations) if result['accepted'] else 0
        result['robustness'] = dict(trials=trials, passed=passed, pass_rate=passed/trials if trials else None,
                                    failure_reasons=dict(failures), real_success_probability=False)
        result['components']['robust'] = passed/trials if trials else 0.
        result['score'] = sum(weights[k]*v for k, v in result['components'].items())/sum(weights.values())
        results.append(result)
    return sorted(results, key=lambda r: (not r['accepted'], -r['score'], r['id']))
