"""Handle-only integration into the existing calibrated grasp workflow.

No basket pose, basket geometry, place target, MTC model or hardware commands.
"""
from pathlib import Path
import numpy as np
import yaml
from grasp_candidates import generate
from grasp_scoring import rank


def prepare_candidates(capture, visual, config, gripper, pad_bounds):
    path = Path(config.get('handle_candidates_config',
        Path(__file__).resolve().parents[1]/'config/multigrasp.yaml'))
    settings = yaml.safe_load(path.read_text())
    # The original planner retains its existing tilt/azimuth/180-degree tests.
    # This integration adds positions along the fitted handle, without multiplying
    # those existing orientation trials a second time.
    settings['orientation_signs'] = [1]
    settings['approach_angles_deg'] = [0]
    settings['open_width_m'] = config['open_width_m']
    settings['scene_margin_m'] = config['scene_margin_m']
    candidates, rejected = generate(capture['visual_result'], np.asarray(capture['base_depth']),
        visual, settings, pad_bounds)
    scores = rank(candidates, settings, gripper)
    by_id = {c.id: c for c in candidates}
    top_k=int(config.get('handle_planning_top_k',settings['planning_top_k']))
    if top_k<1:raise ValueError('handle_planning_top_k must be positive')
    selected = [by_id[s['id']] for s in scores if s['accepted']][:top_k]
    report = dict(scope='handle_only', requires_basket_pose=False, requires_place_target=False,
        candidates=[c.to_dict() for c in candidates], scores=scores, rejected=rejected,
        planning_candidate_ids=[c.id for c in selected],
        clearance_note='unknown here; original calibrated planner checks observed scene and paths',
        planner='existing calibrated RobotSession; not MTC')
    return selected, report


def path_cost(plans):
    """Compare only complete, already validated paths in the original pipeline."""
    length = duration = 0.
    for plan in plans:
        points = plan.joint_trajectory.points
        if not points:
            raise ValueError('empty planned trajectory')
        q = np.asarray([p.positions for p in points], float)
        times = np.asarray([p.time_from_start.to_sec() for p in points])
        if not np.isfinite(q).all() or not np.isfinite(times).all() or np.any(np.diff(times) < 0):
            raise ValueError('invalid planned trajectory')
        length += float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
        duration += float(times[-1])
    return length+.1*duration


def side_place_targets(grasp, config):
    """Relative placement uses the measured handle pose; no basket pose required."""
    from grasp_geometry import validate
    validate(grasp)
    offset = np.asarray(config.get('side_place_offset_base_m', [0., -.05, 0.]), float)
    clearance = float(config.get('side_place_lift_m', .03))
    retreat_distance = float(config.get('side_place_retreat_m', .05))
    if (offset.shape != (3,) or not np.isfinite(offset).all() or abs(offset[2]) > 1e-9
            or not .01 <= np.linalg.norm(offset) <= .10):
        raise ValueError('side placement must be a horizontal offset of 1..10 cm')
    if not .01 <= clearance <= .10 or not .01 <= retreat_distance <= .15:
        raise ValueError('invalid side-placement lift/retreat distance')
    lifted = grasp.copy(); lifted[2, 3] += clearance
    place = grasp.copy(); place[:3, 3] += offset
    above = place.copy(); above[2, 3] += clearance
    retreat = place.copy(); retreat[2, 3] += retreat_distance
    return dict(lift=lifted, transfer=above, place=place, retreat=retreat)


def precheck_side_place(session, tcp, grasp, seed, config):
    """Reject colliding/unreachable later endpoints before planning the prefix."""
    from grasp_geometry import flange_target
    targets = side_place_targets(grasp, config)
    try:
        for name in ('lift', 'transfer', 'place', 'retreat'):
            session.set_tool_width(config['open_width_m'] if name == 'retreat' else config['contact_width_m'])
            try:
                seed = session.solve(flange_target(targets[name], tcp), seed)
            except RuntimeError as error:
                wrapped=type(error)('side-place endpoint '+name+': '+str(error))
                for key in ('joint_state_rad','contacts','joint_state_basis','reported_joint_state_rad'):
                    if hasattr(error,key):setattr(wrapped,key,getattr(error,key))
                raise wrapped from error
    finally:
        session.set_tool_width(config['open_width_m'])


def coarse_precheck_task(session,tcp,pre,grasp,pre_seed,config,place_aside=True):
    """Experimental fast rejection only; all accepted tasks still run fine paths."""
    session.set_tool_width(config['open_width_m'])
    from planner_metrics import PlannerMetrics
    metrics=getattr(session,'metrics',None)
    if isinstance(metrics,PlannerMetrics):metrics.current_stage='coarse_descent'
    seed=session.coarse_check(tcp,pre,grasp,pre_seed)
    try:
        if place_aside:
            targets=side_place_targets(grasp,config)
        else:
            from grasp_geometry import offset_target
            targets={'lift':offset_target(grasp,grasp[:3,2],config['lift_distance_m'])}
        start=grasp
        for name,target in targets.items():
            if isinstance(metrics,PlannerMetrics):metrics.current_stage='coarse_'+name
            session.set_tool_width(config['open_width_m'] if name=='retreat' else config['contact_width_m'])
            seed=session.coarse_check(tcp,start,target,seed);start=target
    finally:session.set_tool_width(config['open_width_m'])


def plan_side_place(session, tcp, grasp, after_descent, config, progress=None):
    """Original calibrated Cartesian solver; all paths planned before any motion."""
    from grasp_ros import state_after
    targets = side_place_targets(grasp, config)
    plans = []; state = after_descent; start = grasp
    try:
        for name in ('lift', 'transfer', 'place', 'retreat'):
            if progress is not None: progress(name)
            session.set_tool_width(config['open_width_m'] if name == 'retreat' else config['contact_width_m'])
            plan = session.plan_descent(tcp, start, targets[name], state)
            plans.append(plan); state = state_after(state, plan); start = targets[name]
    finally:
        session.set_tool_width(config['open_width_m'])
    return targets, plans
