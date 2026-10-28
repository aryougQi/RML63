"""True ROS1 MTC stage graph. No action clients, driver commands or Task.execute.

The articulated planning model is isolated from the calibrated fixed-aperture
hardware model. Execution is blocked until their state correspondence is validated.
"""
import numpy as np
from grasp_geometry import validate, offset_target, flange_target


def validate_place(cfg):
    for key in ('basket_pose_base', 'place_pose_base'):
        if cfg.get(key) is None:
            raise ValueError('full MTC task requires '+key)
        validate(np.asarray(cfg[key], float))
    if not cfg.get('basket_relation_verified') or not cfg.get('basket_geometry_verified'):
        raise ValueError('basket geometry and handle/basket relation require explicit verification')
    if not cfg.get('basket_boxes'):
        raise ValueError('basket collision geometry missing; partial depth cloud is insufficient')
    for box in cfg['basket_boxes']:
        size = np.asarray(box['size_m'], float)
        if size.shape != (3,) or not np.isfinite(size).all() or min(size) <= 0:
            raise ValueError('invalid basket box size')
        validate(np.asarray(box['pose_basket'], float))
    for key in ('place_approach_m', 'retreat_m', 'upright_tolerance_rad', 'planning_timeout_sec'):
        if not np.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('invalid '+key)
    start, end = (np.asarray(cfg[k], float) for k in ('basket_pose_base', 'place_pose_base'))
    if min(start[2, 2], end[2, 2]) < np.cos(cfg['upright_tolerance_rad']):
        raise ValueError('configured basket Z axis is not upright')
    from scipy.spatial.transform import Rotation
    if np.linalg.norm((Rotation.from_matrix if hasattr(Rotation, "from_matrix") else Rotation.from_dcm)(start[:3, :3].T @ end[:3, :3]).as_rotvec()) > cfg['upright_tolerance_rad']:
        raise ValueError('place orientation exceeds transport orientation constraint')


def build_task(candidate, cfg, arm_cfg, gripper, initial_scene=None, world_objects=()):
    """initial_scene is used only for explicit offline tests; live uses CurrentState."""
    validate_place(cfg)
    from moveit.task_constructor import core, stages
    from moveit_msgs.msg import CollisionObject, Constraints, OrientationConstraint
    from shape_msgs.msg import SolidPrimitive
    from grasp_ros import stamped, pose_msg
    from geometry_msgs.msg import PoseStamped
    task = core.Task('candidate_%d' % candidate.id)
    task.name = 'RML63 basket candidate %d' % candidate.id
    task.loadRobotModel('robot_description')
    # No silent fallback to the old fixed-finger model.
    import rospy
    import xml.etree.ElementTree as ET
    semantic = ET.fromstring(rospy.get_param('robot_description_semantic'))
    if semantic.find("group[@name='eg2_planning']") is None:
        raise RuntimeError('MTC requires isolated articulated eg2_planning model; fixed real model unsupported')
    if initial_scene is None:
        task.add(stages.CurrentState('CurrentState'))
    else:
        task.add(stages.CurrentState('CurrentState (recorded scene service, offline)'))
    frame = arm_cfg['base_frame']; link = arm_cfg['flange_frame']
    setup = stages.ModifyPlanningScene('BasketAndObservedScene')
    for obj in world_objects: setup.addObject(obj)
    basket = CollisionObject(); basket.id = 'depth_grasp_basket'; basket.header.frame_id = frame
    basket.operation = CollisionObject.ADD
    base_basket = np.asarray(cfg['basket_pose_base'], float)
    for box in cfg['basket_boxes']:
        basket.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=box['size_m']))
        basket.primitive_poses.append(pose_msg(base_basket @ np.asarray(box['pose_basket'])))
    setup.addObject(basket)
    # Contact permission is limited to actual finger pads, never the entire arm/tool.
    pads = ['4C2_Link5', '4C2_Link6']
    setup.allowCollisions(basket.id, pads, True)
    task.add(setup)
    pipeline = core.PipelinePlanner()
    pipeline.max_velocity_scaling_factor = arm_cfg['velocity_scaling']
    pipeline.max_acceleration_scaling_factor = arm_cfg['acceleration_scaling']
    pipeline.goal_position_tolerance = .001
    pipeline.goal_orientation_tolerance = .01
    cart = core.CartesianPath(); cart.step_size = .002; cart.min_fraction = 1.
    cart.max_velocity_scaling_factor = arm_cfg['velocity_scaling']
    cart.max_acceleration_scaling_factor = arm_cfg['acceleration_scaling']
    joint = core.JointInterpolationPlanner()
    joint.max_velocity_scaling_factor = arm_cfg['velocity_scaling']
    joint.max_acceleration_scaling_factor = arm_cfg['acceleration_scaling']
    opened = gripper.at_width(cfg['open_width_m'])
    closed = gripper.at_width(candidate.width_m)
    tcp = closed['flange_tcp']
    pre = offset_target(candidate.tcp, candidate.tcp[:3, 2], arm_cfg['pregrasp_distance_m'])

    def hand(name, angle):
        stage = stages.MoveTo(name, joint); stage.group = 'eg2_planning'
        stage.setGoal({'4C2_Joint1': float(angle)}); task.add(stage)

    def move(name, target, planner, constraints=None):
        stage = stages.MoveTo(name, planner); stage.group = arm_cfg['planning_group']
        stage.timeout = cfg['planning_timeout_sec']
        ik = PoseStamped(); ik.header.frame_id = link; ik.pose.orientation.w = 1.
        stage.ik_frame = ik
        stage.setGoal(stamped(flange_target(target, tcp), frame))
        stage.restrictDirection(stages.MoveTo.Direction.FORWARD)
        if constraints is not None: stage.path_constraints = constraints
        task.add(stage)

    hand('OpenGripper (scene only)', opened['linkage_angle_rad'])
    move('MoveToPreGrasp', pre, pipeline)
    move('Approach', candidate.tcp, cart)
    hand('CloseGripper (scene only)', closed['linkage_angle_rad'])
    attach = stages.ModifyPlanningScene('AttachBasket')
    attach.attachObject(basket.id, link); task.add(attach)
    # MTC computes the current link->basket shape transforms, preserving the
    # configured basket origin. link6->TCP is included in flange_target above.
    relative = np.linalg.inv(candidate.tcp) @ base_basket
    place_tcp = np.asarray(cfg['place_pose_base']) @ np.linalg.inv(relative)
    constraint = Constraints()
    orientation = OrientationConstraint(); orientation.header.frame_id = frame
    orientation.link_name = link
    orientation.orientation = pose_msg(flange_target(candidate.tcp, tcp)).orientation
    orientation.absolute_x_axis_tolerance = cfg['upright_tolerance_rad']
    orientation.absolute_y_axis_tolerance = cfg['upright_tolerance_rad']
    orientation.absolute_z_axis_tolerance = cfg['upright_tolerance_rad']
    orientation.weight = 1.; constraint.orientation_constraints.append(orientation)
    lifted = candidate.tcp.copy(); lifted[2, 3] += arm_cfg['lift_distance_m']
    move('Lift', lifted, cart, constraint)
    above = place_tcp.copy(); above[2, 3] += cfg['place_approach_m']
    move('Transfer', above, pipeline, constraint)
    near = place_tcp.copy(); near[2, 3] += cfg['place_approach_m']/2
    move('PlaceApproach', near, cart, constraint)
    move('Place', place_tcp, cart, constraint)
    hand('OpenGripperAtPlace (scene only)', opened['linkage_angle_rad'])
    detach = stages.ModifyPlanningScene('DetachBasket')
    detach.detachObject(basket.id, link); task.add(detach)
    retreat = place_tcp.copy(); retreat[2, 3] += cfg['retreat_m']
    move('Retreat', retreat, cart)
    return task


def trajectory_cost(message, weights, clearance_m, cfg):
    """Joint path length in radians; duration from actual timed sub-trajectories.

    Clearance is an observed/model proxy, not swept-volume collision probability.
    Full collision feasibility is supplied by MTC before this cost is evaluated.
    """
    required = {'length', 'duration', 'clearance', 'joint_motion'}
    if set(weights) != required or any(not np.isfinite(v) or v < 0 for v in weights.values()):
        raise ValueError('invalid planning cost weights')
    length = duration = motion = 0.; points = 0
    for sub in message.sub_trajectory:
        trajectory = sub.trajectory.joint_trajectory
        if not trajectory.points: continue  # scene-only stages have no arm trajectory
        q = np.asarray([p.positions for p in trajectory.points], float)
        times = np.asarray([p.time_from_start.to_sec() for p in trajectory.points])
        if not np.isfinite(q).all() or not np.isfinite(times).all() or np.any(np.diff(times) < 0):
            raise ValueError('incomplete/nonfinite trajectory')
        if len(q) > 1:
            if times[-1] <= 0: raise ValueError('trajectory lacks time parameterization')
            delta = np.diff(q, axis=0)
            length += float(np.linalg.norm(delta, axis=1).sum())
            motion += float(abs(delta).sum())
        duration += float(times[-1]); points += len(q)
    if points < 2: raise ValueError('no complete motion trajectory')
    clearance = 1. if clearance_m is None else 1-float(np.clip(clearance_m/cfg['clearance_scale_m'], 0, 1))
    terms = dict(length=length, duration=duration, clearance=clearance, joint_motion=motion)
    return sum(weights[k]*v for k, v in terms.items()), terms


def _plan_ranked(candidates, scores, cfg, arm_cfg, gripper, initial_scene=None, world_objects=()):
    validate_place(cfg)
    tasks = []; feasible = []; failures = []
    lookup = {c.id: c for c in candidates}
    accepted = [s for s in scores if s['accepted']][:cfg['planning_top_k']]
    for score in accepted:
        task = None
        try:
            task = build_task(lookup[score['id']], cfg, arm_cfg, gripper, initial_scene, world_objects)
            tasks.append(task)  # keep introspection and solution objects alive
            task.plan(1)
            if not len(task.solutions):
                diagnostics = []
                for stage in task:
                    for failure in stage.failures:
                        diagnostics.append(dict(stage=stage.name, reason=failure.comment or 'MTC solver rejected this stage; inspect OMPL/Cartesian log'))
                failures.append(dict(id=score['id'], reason='no complete MTC solution', stages=diagnostics))
                continue
            for solution in task.solutions:
                msg = solution.toMsg()
                cost, terms = trajectory_cost(msg, cfg['cost_weights'], score['clearance_m'], cfg)
                feasible.append(dict(id=score['id'], cost=cost, terms=terms,
                                     task=task, solution=solution, message=msg))
        except Exception as e:
            failures.append(dict(id=score['id'], reason=str(e)))
    feasible.sort(key=lambda p: (p['cost'], p['id']))
    return feasible, failures, tasks


def require_execution_disabled(execute):
    if execute:
        raise RuntimeError('MTC execution blocked: articulated planning model/calibrated hardware state and gripper phase synchronization not yet validated. Existing grasp_demo.py --execute remains the small-lift workflow.')


def recorded_scene_service(state_blob, namespace, ready):
    """Separate process avoids the 0.1.3 plan() binding holding Python's GIL.

    This is explicitly recorded feedback, not a simulated live robot. The only
    service it supplies is GetPlanningScene; no motion interfaces are created.
    """
    import rospy
    from moveit_msgs.msg import RobotState
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneResponse
    import xml.etree.ElementTree as ET
    state = RobotState(); state.deserialize(state_blob)
    rospy.init_node('recorded_scene', argv=['recorded_scene', '__ns:='+namespace], anonymous=True)
    urdf = ET.fromstring(rospy.get_param(namespace+'robot_description'))
    semantic = ET.fromstring(rospy.get_param(namespace+'robot_description_semantic'))
    names = [link.get('name') for link in urdf.findall('link')]
    matrix = AllowedCollisionMatrix(); matrix.entry_names = names
    allowed = {frozenset((pair.get('link1'), pair.get('link2')))
               for pair in semantic.findall('disable_collisions')}
    matrix.entry_values = [AllowedCollisionEntry(enabled=[
        i == j or frozenset((a, b)) in allowed for j, b in enumerate(names)])
        for i, a in enumerate(names)]
    def respond(request):
        response = GetPlanningSceneResponse()
        response.scene.name = 'recorded capture for offline planning'
        response.scene.robot_state = state
        response.scene.robot_model_name = 'rml_63_description'
        response.scene.allowed_collision_matrix = matrix
        return response
    service = rospy.Service('get_planning_scene', GetPlanningScene, respond)
    ready.set()
    rospy.spin()


def plan_ranked(candidates, scores, cfg, arm_cfg, gripper, initial_scene=None, world_objects=()):
    if initial_scene is None:
        return _plan_ranked(candidates, scores, cfg, arm_cfg, gripper, None, world_objects)
    import rospy
    import multiprocessing
    import io
    if rospy.get_namespace() == '/':
        raise RuntimeError('recorded-scene MTC requires isolated namespace')
    import rosgraph
    _, _, services = rosgraph.Master(rospy.get_name()).getSystemState()
    if rospy.resolve_name('get_planning_scene') in dict(services):
        raise RuntimeError('refusing to replace an existing planning-scene service')
    data = io.BytesIO(); initial_scene.serialize(data)
    ctx = multiprocessing.get_context('spawn'); ready = ctx.Event()
    process = ctx.Process(target=recorded_scene_service, args=(data.getvalue(), rospy.get_namespace(), ready))
    process.start()
    try:
        if not ready.wait(10): raise RuntimeError('recorded planning-scene service failed to start')
        return _plan_ranked(candidates, scores, cfg, arm_cfg, gripper, initial_scene, world_objects)
    finally:
        process.terminate(); process.join(5)
