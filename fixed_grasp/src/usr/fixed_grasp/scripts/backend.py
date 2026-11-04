"""Adapter for the existing local calibrated planner; no vision pipeline imports."""
import hashlib
from copy import copy, deepcopy
from pathlib import Path
import sys
import time
import numpy as np
import yaml
import rospkg
import rospy

PACK = rospkg.RosPack()
# Runtime helpers ship alongside fixed_grasp.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from grasp_ros import RobotSession, GripperIO, pose_msg, state_after
from rm_msgs.msg import Gripper_Pick, Gripper_Set
from grasp_geometry import GripperGeometry, validate, flange_target
from capture_geometry import environment_geometry
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, DisplayTrajectory, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive
from fixed_grasp.srv import VerifySupport, VerifySupportRequest
from waypoint_offsets import offset_pose
from planner_metrics import profiled, PlannerMetrics, CollisionFailure
from tool_envelope import build_mesh_union
from grasp_profiles import load_task_waypoints


def state_after_shared_geometry(state, trajectory):
    """Reuse immutable CAD while keeping original joint-state update semantics."""
    light = copy(state)
    light.attached_collision_objects = []
    result = state_after(light, trajectory)
    result.attached_collision_objects = list(state.attached_collision_objects)
    result.is_diff = state.is_diff
    return result


def load_settings(path):
    root = Path(PACK.get_path('fixed_grasp'))
    config = yaml.safe_load((root / 'config/runtime_defaults.yaml').read_text())
    config['calibration_file'] = str(root / 'config/runtime_kinematics.yaml')
    config['mount_file'] = str(Path(PACK.get_path('rml63_tools_description')) / 'config/measured_mount.yaml')
    extra = yaml.safe_load(Path(path).read_text())
    allowed = {'scene_mode', 'waypoints_file', 'basket', 'support_service', 'support_source', 'release_mode',
               'taught_place_verification', 'environment_geometry', 'open_width_m',
               'velocity_scaling', 'acceleration_scaling', 'gripper_min_hold_force', 'gripper_force',
               'fast_ik_enabled', 'planning_attempts', 'joint_plan_timeout_sec',
               'trajectory_start_delay_sec', 'trajectory_settle_timeout_sec', 'trajectory_settle_poll_sec',
               'motion_result_poll_sec', 'gripper_poll_interval_sec'}
    if set(extra) - allowed:
        raise ValueError('fixed config cannot override existing safety thresholds')
    config.update(extra)
    if not isinstance(config.get('fast_ik_enabled', True), bool):
        raise ValueError('fast_ik_enabled must be boolean')
    attempts = config.get('planning_attempts', 3)
    if isinstance(attempts, bool) or not isinstance(attempts, int) or not 1 <= attempts <= 10:
        raise ValueError('planning_attempts must be an integer in [1, 10]')
    timeout = float(config.get('joint_plan_timeout_sec', 8.))
    if not np.isfinite(timeout) or timeout <= 0:
        raise ValueError('joint_plan_timeout_sec must be finite and positive')
    config['planning_attempts'] = attempts
    config['joint_plan_timeout_sec'] = timeout
    for key, default, allow_zero in (
            ('trajectory_start_delay_sec', .05, True),
            ('trajectory_settle_timeout_sec', .6, True),
            ('trajectory_settle_poll_sec', .02, False),
            ('motion_result_poll_sec', .05, False),
            ('gripper_poll_interval_sec', .05, False)):
        value = float(config.get(key, default))
        if not np.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
            raise ValueError(key+' must be finite and '+('nonnegative' if allow_zero else 'positive'))
        config[key] = value
    grip_force = config['gripper_force']
    if isinstance(grip_force, bool) or not isinstance(grip_force, int) or not 1 <= grip_force <= 1000:
        raise ValueError('gripper_force must be an integer in [1, 1000]')
    force = float(config.get('gripper_min_hold_force', 50))
    if not np.isfinite(force) or force <= 0:
        raise ValueError('gripper_min_hold_force must be finite and positive')
    config['gripper_min_hold_force'] = force
    for key in ('velocity_scaling', 'acceleration_scaling'):
        value = float(config[key])
        if not np.isfinite(value) or not 0 < value <= 1:
            raise ValueError(key + ' must be finite and in (0, 1]')
        config[key] = value
    width = float(config['open_width_m'])
    if not np.isfinite(width) or not 0 < width <= .070:
        raise ValueError('open_width_m must be finite and in (0, 0.070] metres')
    config['open_width_m'] = width
    if config.get('scene_mode', 'measured') not in ('measured', 'robot_only', 'existing'):
        raise ValueError('scene_mode must be measured, robot_only or existing')
    if config.get('release_mode', 'support_service') not in ('support_service', 'taught_place'):
        raise ValueError('release_mode must be support_service or taught_place')
    if not isinstance(config.get('taught_place_verification', {}), dict):
        raise ValueError('taught_place_verification must be a mapping')
    return config


def geometry(config):
    root = Path(PACK.get_path('rml63_tools_description'))
    mount = yaml.safe_load(Path(config['mount_file']).read_text())['launch_defaults']
    return GripperGeometry(root/'upstream/EG2-4C2.urdf', root/'meshes/eg2_4c2',
                           mount['mount_xyz'], mount['mount_rpy'])


def identity(session, tcp):
    def digest(value):
        return hashlib.sha256(value).hexdigest()
    return dict(calibration_sha256=session.calibration_hash,
                mount_sha256=digest(Path(session.config['mount_file']).read_bytes()),
                urdf_sha256=digest(session.xml.encode()),
                srdf_sha256=digest(rospy.get_param('/robot_description_semantic').encode()),
                base_frame=session.config['base_frame'], flange_frame=session.config['flange_frame'],
                joint_names=session.active, flange_tcp=tcp.tolist())


class Session(RobotSession):
    def __init__(self, config, check):
        self.cancel_check = check
        super().__init__(config)

    def check_planning_deadline(self):
        self.cancel_check()
        super().check_planning_deadline()

    def measured(self):
        self.cancel_check()
        result = super().measured()
        if self.moving and getattr(self, 'hold_monitor', None):
            self.hold_monitor()
        return result

    def verify_hardware(self, require_execution=True):
        if self.config.get('scene_mode', 'measured') not in ('robot_only', 'existing'):
            return super().verify_hardware(require_execution)
        super().verify_hardware(require_execution=False)
        if require_execution and not rospy.get_param('/move_group/allow_trajectory_execution', False):
            raise RuntimeError('real trajectory execution disabled')

    def require_environment_verified(self):
        if self.config.get('scene_mode', 'measured') == 'existing':
            # Explicit operator choice: rely on MoveIt's existing scene only.
            # This is not an environment certification by fixed_grasp.
            return
        if self.config.get('scene_mode', 'measured') != 'robot_only':
            return super().require_environment_verified()
        if not getattr(self, 'robot_scene_ready', False):
            raise RuntimeError('robot/tool collision scene has not been built')

    @profiled('fixed_scene_apply')
    def apply(self, world=None, attached=None, collision_matrix=None):
        try:
            return super().apply(world, attached, collision_matrix)
        except Exception as error:
            world_ops = [(o.id, o.operation) for o in world or []]
            attached_ops = [(a.object.id, a.object.operation) for a in attached or []]
            raise RuntimeError('scene update failed: world=%s attached=%s; %s' %
                               (world_ops, attached_ops, error)) from error

    @profiled('fixed_state_for')
    def state_for(self, q, reference=None):
        state = super().state_for(q, reference)
        if self.config.get('scene_mode') == 'existing':
            # A joint-state diff inherits move_group's current attached bodies;
            # an empty non-diff state would incorrectly clear them for planning.
            state.attached_collision_objects = []
            state.is_diff = True
        return state

    @profiled('fixed_retime')
    def dense_timed(self, trajectory, start):
        return super().dense_timed(trajectory, start)

    @profiled('fixed_ik')
    def solve(self, desired_flange, seed):
        self.check_planning_deadline()
        if self.config.get('fast_ik_enabled', True):
            if not hasattr(self, '_fast_ik'):
                try:
                    from fast_ik import CalibratedIK
                except ImportError:
                    self._fast_ik = None
                else:
                    try:
                        self._fast_ik = CalibratedIK(self.chain, self.offsets,
                            [self.robot.get_joint(name).bounds() for name in self.active])
                    except ValueError as error:
                        rospy.logwarn('compiled IK unavailable, using numerical IK: %s', error)
                        self._fast_ik = None
            if self._fast_ik is not None:
                for q in self._fast_ik.solutions(desired_flange, seed):
                    self.check_planning_deadline()
                    if (getattr(self, '_local_ik', False) and
                            np.max(abs(q-seed)) > np.radians(self.config['max_joint_jump_deg'])):
                        continue
                    try:
                        self.check_state(q)
                    except CollisionFailure:
                        continue
                    if isinstance(getattr(self, 'metrics', None), PlannerMetrics):
                        self.metrics.counts['fast_ik_successes'] += 1
                    return q
        if isinstance(getattr(self, 'metrics', None), PlannerMetrics):
            self.metrics.counts['numeric_ik_fallbacks'] += 1
        return super().solve(desired_flange, seed)

    def plan_descent(self, flange_tcp, start_tcp, end_tcp, start_state):
        previous = getattr(self, '_local_ik', False)
        self._local_ik = True
        try:
            return super().plan_descent(flange_tcp, start_tcp, end_tcp, start_state)
        finally:
            self._local_ik = previous

    @profiled('fixed_joint_path')
    def plan_joint(self, q, start):
        self.check_planning_deadline()
        budget = self.config.get('joint_plan_timeout_sec', 8.)
        deadline = time.monotonic()+budget
        if getattr(self, 'planning_deadline', None) is not None:
            deadline = min(deadline, self.planning_deadline)
        attempts = self.config.get('planning_attempts', 3)
        failure = RuntimeError('MoveIt joint plan failed')
        self.group.set_num_planning_attempts(1)
        for attempt in range(attempts):
            self.check_planning_deadline()
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                break
            # Short attempts first, full remaining budget on the last attempt.
            allocation = remaining if attempt == attempts-1 else min(remaining, budget/(attempts-attempt))
            self.group.set_planning_time(allocation)
            self.group.set_start_state(start)
            self.group.set_joint_value_target(dict(zip(self.active, q)))
            response = self.group.plan()
            self.check_planning_deadline()
            success, trajectory = (response[0], response[1]) if isinstance(response, tuple) else (True, response)
            if not success:
                continue
            trajectory = self.dense_timed(trajectory, start)
            try:
                self.check_trajectory(trajectory, start)
            except CollisionFailure as error:
                # A nominal MoveIt route may fail the calibrated collision check.
                failure = error
                continue
            return trajectory
        raise failure

    @profiled('fixed_trajectory_validation')
    def check_trajectory(self, trajectory, start):
        # Fixed tasks use the robot's actual joint bounds, without the visual
        # approach planner's arbitrary 90-degree total travel cap.
        jt = trajectory.joint_trajectory
        if (len(jt.joint_names) != len(self.active) or
                set(jt.joint_names) != set(self.active) or not jt.points):
            raise RuntimeError('invalid/empty joint trajectory')
        initial = dict(zip(start.joint_state.name, start.joint_state.position))
        q0 = np.array([initial[name] for name in jt.joint_names])
        positions = np.asarray([point.positions for point in jt.points], float)
        if positions.shape != (len(jt.points), len(self.active)) or not np.isfinite(positions).all():
            raise RuntimeError('invalid joint trajectory positions')
        if not np.isfinite(q0).all() or np.max(abs(positions[0]-q0)) > .02:
            raise RuntimeError('trajectory start mismatch')
        if len(positions) > 1 and np.max(abs(np.diff(positions, axis=0))) > np.radians(self.config['max_joint_jump_deg']):
            raise RuntimeError('trajectory joint jump')
        times = [point.time_from_start.to_sec() for point in jt.points]
        if not np.isfinite(times).all() or times[0] < 0 or np.any(np.diff(times) <= 0):
            raise RuntimeError('trajectory timing invalid')
        for point in positions:
            self.check_planning_deadline()
            values = dict(zip(jt.joint_names, point))
            self.check_state(np.array([values[name] for name in self.active]))
        return positions

    def check_state(self, q):
        # Existing validity service does not itself guarantee joint bounds.
        for values in (np.asarray(q), np.asarray(q)+self.offsets):
            if not np.isfinite(values).all():
                raise ValueError('nonfinite joints')
            for name, value in zip(self.active, values):
                low, high = self.robot.get_joint(name).bounds()
                if not low <= value <= high:
                    raise RuntimeError('joint limit: '+name)
        super().check_state(q)


def force_contact(state, minimum, allow_regrip=False):
    """Require force-stop initially; allow loaded reclosure after contact is verified."""
    force = state.get('current_force')
    # Pick_On can alternate force-stop (6) and closing (4) while maintaining
    # contact. Closing alone must never establish the initial grasp.
    modes = (4, 6) if allow_regrip else (6,)
    return (state.get('status') == 1 and state.get('enable') == 1 and
            state.get('error', 0) == 0 and state.get('mode') in modes and
            isinstance(force, (int, float)) and np.isfinite(force) and
            abs(force) >= minimum)


class Gripper(GripperIO):
    def __init__(self, config, check):
        self.config = config
        self.position_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=1)
        # Continuous force control maintains the grip if the object settles.
        self.pick_pub = rospy.Publisher('/rm_driver/Gripper_Pick_On', Gripper_Pick, queue_size=1)
        self.check = check

    def query(self, command):
        self.check()
        return super().query(command)

    def _connected(self, publisher):
        super()._connected(publisher)
        self.check()

    def close(self):
        self._connected(self.pick_pub)
        self.pick_pub.publish(Gripper_Pick(speed=self.config['gripper_speed'],
                                           force=self.config['gripper_force']))
        deadline = time.monotonic() + self.config['gripper_timeout_sec']
        history = []
        consecutive = 0
        while time.monotonic() < deadline:
            state = self.query('get_gripper_state')
            history.append(dict(state))
            if state.get('error', 0):
                raise RuntimeError('gripper error during force-controlled closure')
            consecutive = consecutive + 1 if force_contact(state, self.config['gripper_min_hold_force']) else 0
            if consecutive >= 3:
                return dict(state, stable_feedback_samples=history[-3:])
            rospy.sleep(self.config.get('gripper_poll_interval_sec', .15))
        error = RuntimeError('force-controlled contact not verified; minimum_force={}; last feedback: {}'.format(
            self.config['gripper_min_hold_force'], history[-1] if history else None))
        error.gripper_feedback = history
        raise error


class Backend:
    def __init__(self, config, waypoints, allow, check, task_id, grasp_waypoints=None):
        self.config, self.allow, self.check, self.task_id = config, allow, check, task_id
        self.data, self.waypoints_sha256 = load_task_waypoints(waypoints, grasp_waypoints)
        self.s = Session(config, check)
        self.metrics = self.s.metrics
        self.geom = geometry(config)
        self.tcp = self.geom.at_width(config['contact_width_m'])['flange_tcp']
        if self.data.get('schema') != 1 or self.data.get('identity') != identity(self.s, self.tcp):
            raise RuntimeError('teaching model/calibration/TCP mismatch; teach again')
        self.poses = {}
        for name, record in self.data['waypoints'].items():
            if 'derived_from' in record:
                continue
            q = np.asarray(record['joints_rad'], float)
            if q.shape != (len(self.s.active),) or not np.isfinite(q).all():
                raise ValueError('invalid taught joints: '+name)
            pose = validate(np.asarray(record['tcp_base'], float))
            if not np.allclose(self.s.chain.fk(q, self.s.offsets)@self.tcp, pose, atol=1e-7):
                raise ValueError('taught joints/TCP disagree: '+name)
            self.poses[name] = pose
        for name, record in self.data['waypoints'].items():
            if 'derived_from' not in record:
                continue
            if name not in ('Pregrasp', 'Lift') or record['derived_from'] != 'Grasp' or 'Grasp' not in self.poses:
                raise ValueError('only Pregrasp/Lift may derive from measured Grasp')
            pose = offset_pose(self.poses['Grasp'], record['offset_base_m'])
            if not np.allclose(pose, np.asarray(record['tcp_base'], float), atol=1e-9):
                raise ValueError('Grasp changed; rerun derive_grasp.py before planning')
            self.poses[name] = validate(pose)
        self.gripper = Gripper(config, check)
        self.s.hold_monitor = lambda: self.quick_hold() if self.attached else None
        self.attached = False
        self.placed = False
        self.scene_ready = False
        self.scene_touched = False
        self.original = None if config.get('scene_mode') == 'existing' else self.current_scene()
        if self.original and any(a.object.id == 'fixed_grasp_basket' for a in self.original.robot_state.attached_collision_objects):
            raise RuntimeError('basket still attached from previous task; manual recovery required')

    @profiled('fixed_scene_read')
    def current_scene(self):
        request = GetPlanningSceneRequest()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        return rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)(request).scene

    def preflight(self, execute, full, home_only=False, confirm_empty=False):
        self.check()
        if confirm_empty and (not home_only or self.config.get('scene_mode') != 'existing'):
            raise RuntimeError('confirmed-empty recovery is only available for Home in existing scene mode')
        required = ['Home', 'Grasp'] if home_only else ['Home', 'Pregrasp', 'Grasp', 'Lift'] + (['Preplace', 'Place', 'Retreat'] if full else [])
        if any(n not in self.poses for n in required):
            raise ValueError('missing taught poses: '+str(required))
        if not home_only:
            delta = self.poses['Lift'][:3, 3]-self.poses['Grasp'][:3, 3]
            if not np.isfinite(delta).all() or delta[2] <= 0 or np.linalg.norm(delta[:2]) > 1e-6:
                raise ValueError('Lift must be finite, vertical along base +Z, and above Grasp')
            for a, b in [('Pregrasp', 'Grasp'), ('Grasp', 'Lift')] + ([('Preplace', 'Place'), ('Place', 'Retreat')] if full else []):
                from scipy.spatial.transform import Rotation
                if np.linalg.norm(Rotation.from_dcm(self.poses[a][:3,:3].T@self.poses[b][:3,:3]).as_rotvec()) > np.radians(.5):
                    raise ValueError('Cartesian poses must share orientation: '+a+' -> '+b)
        scene_mode = self.config.get('scene_mode', 'measured')
        external_models_optional = scene_mode in ('robot_only', 'existing')
        basket = self.config['basket']
        if not external_models_optional and not basket['boxes']:
            raise RuntimeError('complete basket collision boxes required, including for planning')
        for box in ([] if external_models_optional else basket['boxes']):
            validate(np.asarray(box['pose_tcp'], float))
            sizes = np.asarray(box['size_m'], float)
            if sizes.shape != (3,) or not np.isfinite(sizes).all() or np.any(sizes <= 0):
                raise ValueError('invalid basket box')
        if execute:
            if not self.allow:
                raise RuntimeError('EXECUTE disabled; launch allow_execution:=true explicitly')
            if not external_models_optional:
                if basket.get('verified') is not True or not basket.get('source'):
                    raise RuntimeError('full carried basket geometry not verified')
                environment_geometry(self.config, require_verified=True)
            if full:
                self.require_release_config()
            if external_models_optional:
                rospy.logwarn('%s: fixed_grasp does not certify external environment or carried basket collisions', scene_mode)
            self.s.verify_hardware()
            if home_only:
                self.require_home_empty(confirm_empty)
            else:
                self.require_open()
        if not self.scene_ready:
            if scene_mode == 'existing':
                self.scene_ready = True
            else:
                self.build_scene()
        if execute:
            self.s.require_environment_verified()

    def basket(self, attached=False):
        obj = CollisionObject()
        obj.id = 'fixed_grasp_basket'
        obj.operation = CollisionObject.ADD
        obj.header.frame_id = self.config['flange_frame'] if attached else self.config['base_frame']
        origin = self.tcp if attached else self.poses['Place' if self.placed else 'Grasp']
        for box in self.config['basket']['boxes']:
            # Same existing margin; no contact exemption or ACM relaxation.
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,
                dimensions=(np.asarray(box['size_m'])+2*self.config['scene_margin_m']).tolist()))
            obj.primitive_poses.append(pose_msg(origin@np.asarray(box['pose_tcp'])))
        return obj

    def set_basket(self, attached):
        if self.config.get('scene_mode', 'measured') in ('robot_only', 'existing'):
            # Logical holding state still drives gripper monitoring; no invented basket geometry.
            self.attached = attached
            return
        obj = self.basket(attached)
        current = self.current_scene()
        world_ids = {o.id for o in current.world.collision_objects}
        attached_ids = {a.object.id for a in current.robot_state.attached_collision_objects}
        a = AttachedCollisionObject()
        a.link_name = self.config['flange_frame']
        a.touch_links = []
        self.scene_touched = True
        if attached:
            a.object = obj
            removed = [CollisionObject(id=obj.id, operation=CollisionObject.REMOVE)] if obj.id in world_ids else []
            self.s.apply(world=removed, attached=[a])
        else:
            a.object = CollisionObject(id=obj.id, operation=CollisionObject.REMOVE)
            # MoveIt rejects removing an attached object that does not exist.
            removed = [a] if obj.id in attached_ids else []
            self.s.apply(world=[obj], attached=removed)
        self.attached = attached

    @profiled('fixed_scene_build')
    def build_scene(self):
        if self.config.get('scene_mode') == 'existing':
            self.scene_ready = True
            return
        robot_only = self.config.get('scene_mode', 'measured') == 'robot_only'
        env = dict(boxes=[]) if robot_only else environment_geometry(self.config)
        if not robot_only and (env.get('mode') != 'measured_boxes' or not env['boxes']):
            raise RuntimeError('fixed grasp requires nonempty measured environment boxes')
        world = []
        for i, box in enumerate(env['boxes']):
            obj = CollisionObject(id='fixed_grasp_environment_%d'%i, operation=CollisionObject.ADD)
            obj.header.frame_id = self.config['base_frame']
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=box['size_m'])]
            obj.primitive_poses = [pose_msg(np.asarray(box['pose_base']))]
            world.append(obj)
        # Conservative union of CAD at all openings sampled <=1 mm. Retain
        # original URDF collision geometry and ACM; may reject tight grasps.
        tool = AttachedCollisionObject()
        tool.link_name = self.config['flange_frame']
        tool.touch_links = [self.config['flange_frame'], 'fixed_plate_link'] + list(self.geom.meshes)
        tool.object.id = 'fixed_grasp_tool_envelope'
        tool.object.header.frame_id = self.config['flange_frame']
        tool.object.operation = CollisionObject.ADD
        meshes, transforms, counts = build_mesh_union(self.geom, self.config['open_width_m'], self.check)
        tool.object.meshes = meshes
        tool.object.mesh_poses = [pose_msg(t) for t in transforms]
        self.metrics.counts.update(counts)
        center = self.s.handeye @ np.array([0.,0.,-.013,1.])
        pose = np.eye(4); pose[:3,3] = center[:3]
        tool.object.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[self.config['camera_radius_m']])]
        tool.object.primitive_poses = [pose_msg(pose)]
        self.scene_touched = True
        self.s.apply(world=world, attached=[tool])
        self.s.open_tool_attachment = tool
        self.s.scene_environment = deepcopy(env)
        self.s.robot_scene_ready = True
        self.set_basket(False)
        self.scene_ready = True

    @profiled('fixed_plan')
    def plan(self, names, check):
        q, _ = self.s.measured()
        self.plan_start_joints_rad = q.tolist()
        state = self.s.state_for(q)
        plans = {}
        initial = state
        try:
            for name in names:
                check()
                # Keep the two Home trajectories separate: their start states differ.
                pose_name = 'Home' if name == 'ReturnHome' else name
                q = np.array([dict(zip(state.joint_state.name, state.joint_state.position))[n] for n in self.s.active])
                # Explicitly refresh attached bodies after each scene transition.
                state = self.planning_state(q, reference=state)
                if name in ('Grasp', 'Lift', 'Place', 'Retreat'):
                    start_tcp = self.s.chain.fk(q, self.s.offsets)@self.tcp
                    # Numerical IK endpoint residuals must not change taught orientation.
                    target = self.poses[name].copy()
                    from scipy.spatial.transform import Rotation
                    error = np.linalg.norm(Rotation.from_dcm(start_tcp[:3,:3].T@target[:3,:3]).as_rotvec())
                    if error > np.radians(.5):
                        raise RuntimeError('Cartesian start orientation mismatch')
                    target[:3,:3] = start_tcp[:3,:3]
                    trajectory = self.s.plan_descent(self.tcp, start_tcp, target, state)
                else:
                    record = self.data['waypoints'].get(pose_name, {})
                    if 'joints_rad' in record:
                        # Constructor already verified this measured joint/TCP pair.
                        target_q = np.asarray(record['joints_rad'], float)
                    else:
                        seed = (np.asarray(self.data['waypoints']['Grasp']['joints_rad'], float)
                                if pose_name == 'Pregrasp' else q)
                        target_q = self.s.solve(flange_target(self.poses[pose_name], self.tcp), seed)
                    self.s.check_state(target_q)
                    trajectory = self.s.plan_joint(target_q, state)
                plans[name] = trajectory
                state = state_after_shared_geometry(state, trajectory)
                if name == 'Grasp': self.set_basket(True)
                if name == 'Place':
                    self.placed = True
                    self.set_basket(False)
            self.s.display.publish(DisplayTrajectory(trajectory_start=initial, trajectory=list(plans.values())))
            return plans
        finally:
            self.placed = False
            self.set_basket(False)

    def planning_state(self, q, reference=None):
        state = self.s.state_for(q, reference)
        if self.config.get('scene_mode') == 'existing':
            return state
        state.attached_collision_objects = [a for a in state.attached_collision_objects
                                            if a.object.id != 'fixed_grasp_basket']
        if self.attached and self.config.get('scene_mode', 'measured') != 'robot_only':
            a = AttachedCollisionObject()
            a.link_name = self.config['flange_frame']
            a.object = self.basket(True)
            state.attached_collision_objects.append(a)
        return state

    @profiled('fixed_scene_restore')
    def restore_preview(self):
        # No scene was modified when preflight failed; do not create cleanup errors.
        if self.config.get('scene_mode') == 'existing' or not self.scene_touched:
            return
        ids = {'fixed_grasp_tool_envelope'}
        if self.config.get('scene_mode', 'measured') != 'robot_only':
            ids.add('fixed_grasp_basket')
            ids.update('fixed_grasp_environment_%d'%i for i in range(len(self.config['environment_geometry']['boxes'])))
        current = self.current_scene()
        attached = []
        for original in current.robot_state.attached_collision_objects:
            if original.object.id not in ids:
                continue
            a = AttachedCollisionObject()
            a.link_name = original.link_name
            a.object = CollisionObject(id=original.object.id, operation=CollisionObject.REMOVE)
            attached.append(a)
        if attached:
            self.s.apply(attached=attached)
            # MoveIt detaches bodies back into the world. Query after detaching
            # so those newly-created world objects are removed as well.
            current = self.current_scene()
        world = [CollisionObject(id=o.id, operation=CollisionObject.REMOVE)
                 for o in current.world.collision_objects if o.id in ids]
        if world:
            self.s.apply(world=world)
        original_world = [o for o in self.original.world.collision_objects if o.id in ids]
        original_attached = [a for a in self.original.robot_state.attached_collision_objects if a.object.id in ids]
        if original_world or original_attached:
            self.s.apply(world=original_world, attached=original_attached)
        self.scene_touched = False
        self.scene_ready = False
        self.s.robot_scene_ready = False

    def move(self, name, trajectory):
        self.check()
        if name == 'ReturnHome':
            self.require_open()
        if self.attached:
            self.verify_hold()
        self.s.check_trajectory(trajectory, self.planning_state(self.s.measured()[0]))
        self.check()
        self.s.execute_one(trajectory, self.tcp, self.poses['Home' if name == 'ReturnHome' else name])

    def require_home_empty(self, confirm_empty=False):
        if not confirm_empty:
            self.require_open()
            return
        if self.config.get('scene_mode') != 'existing':
            raise RuntimeError('confirmed-empty Home requires existing scene mode')
        # An operator can confirm an empty, unpowered/uninitialized gripper.
        # This does not initialize, enable, open or close it. Positive evidence
        # of load, movement or a fault must still stop recovery.
        unready = False
        for sample in range(3):
            self.check()
            state = self.gripper.query('get_gripper_state')
            force = state.get('current_force')
            if (state.get('status') not in (0, 1) or state.get('enable') not in (0, 1) or
                    state.get('error') != 0 or state.get('mode') not in (0, 1, 2, 3) or
                    not isinstance(force, (int, float)) or not np.isfinite(force) or abs(force) >= 50):
                raise RuntimeError('confirmed-empty Home blocked by gripper load/motion/fault or invalid feedback: '+str(state))
            unready = unready or state.get('status') != 1 or state.get('enable') != 1 or state.get('mode') == 0
            if sample < 2:
                time.sleep(self.config.get('gripper_poll_interval_sec', .15))
        if unready:
            rospy.logwarn('Home only: operator confirmed empty; gripper is offline/uninitialized; no gripper commands will be sent')

    def require_open(self):
        # Existing-scene mode does not build an aperture-dependent tool envelope.
        # Keep feedback checks, but do not require the configured release width.
        match_width = self.config.get('scene_mode', 'measured') != 'existing'
        target = int(round(self.config['open_width_m']/.070*1000))
        for sample in range(3):
            s = self.gripper.query('get_gripper_state')
            if not (s.get('status') == 1 and s.get('enable') == 1 and s.get('error', 0) == 0 and
                    s.get('mode') in (1,2,3) and
                    (not match_width or abs(s.get('actpos', -1000)-target) < 35) and
                    abs(s.get('current_force', 10000)) < 50):
                raise RuntimeError('gripper preflight failed (status={}, enable={}, error={}, mode={}, '
                                   'actpos={}, force={}, required_position={}); no automatic opening'.format(
                                       s.get('status'), s.get('enable'), s.get('error', 0),
                                       s.get('mode'), s.get('actpos'), s.get('current_force'),
                                       target if match_width else 'unrestricted') +
                                   ('; gripper not ready: after robot power-up, check gripper power, '
                                    'communication and initialization/enable on the teach pendant'
                                    if s.get('status') == 0 or s.get('enable') == 0 else ''))
            if sample < 2:
                time.sleep(self.config.get('gripper_poll_interval_sec', .15))

    def prepare_gripper(self):
        self.check()
        self.require_open()
        self.check()
        # Reuse the existing command and stable-position/force feedback checks.
        self.gripper.open(self.config['open_width_m'])
        self.check()

    def close(self):
        self.check()
        self.gripper.close()
        # close() already requires three consecutive force-contact samples.
        # Lift validates holding again before dispatch and while moving.

    def quick_hold(self):
        s = self.gripper.query('get_gripper_state')
        # Used after Gripper.close() has verified three force-stop samples.
        if not force_contact(s, self.config['gripper_min_hold_force'], allow_regrip=True):
            raise RuntimeError('gripper force contact lost/unverified: minimum_force={}, feedback={}'.format(
                self.config['gripper_min_hold_force'], s))

    def verify_hold(self):
        for sample in range(3):
            self.quick_hold()
            if sample < 2:
                time.sleep(self.config.get('gripper_poll_interval_sec', .15))

    def attach(self):
        self.set_basket(True)

    def require_release_config(self):
        mode = self.config.get('release_mode', 'support_service')
        if mode == 'support_service':
            if not self.config['support_service'] or not self.config['support_source']:
                raise RuntimeError('reliable support verification adapter not configured')
        elif mode == 'taught_place':
            proof = self.config.get('taught_place_verification', {})
            if (proof.get('verified') is not True or not proof.get('source') or
                    proof.get('waypoints_sha256') != self.waypoints_sha256):
                raise RuntimeError('taught Place release not verified for this waypoint file; '
                                   'verify placement with the current object and update taught_place_verification')
        else:
            raise ValueError('unknown release_mode: '+str(mode))

    def verify_taught_place(self):
        """Check actual, stationary TCP against the operator-verified placement."""
        from scipy.spatial.transform import Rotation
        self.s.verify_hardware()
        samples = []
        last_stamp = None
        target = self.poses['Place']
        for _ in range(3):
            self.check()
            q, stamp = self.s.measured()
            if last_stamp is not None and stamp <= last_stamp:
                raise RuntimeError('taught Place release requires distinct fresh joint samples')
            last_stamp = stamp
            achieved = validate(self.s.chain.fk(q, self.s.offsets) @ self.tcp)
            distance = np.linalg.norm(achieved[:3, 3]-target[:3, 3])
            angle = np.degrees(np.linalg.norm(Rotation.from_dcm(
                target[:3, :3].T @ achieved[:3, :3]).as_rotvec()))
            if distance > self.config['position_tolerance_m'] or angle > self.config['orientation_tolerance_deg']:
                raise RuntimeError('taught Place release TCP mismatch: %.2f mm / %.2f deg' %
                                   (distance*1000, angle))
            samples.append(q.copy())
            if np.max(np.ptp(np.asarray(samples), axis=0)) > np.radians(self.config['max_capture_motion_deg']):
                raise RuntimeError('robot not stationary at taught Place; no release')
            self.quick_hold()
            if len(samples) < 3:
                rospy.sleep(self.config.get('gripper_poll_interval_sec', .15))
        self.check()

    def verify_support(self):
        self.require_release_config()
        if self.config.get('release_mode', 'support_service') == 'taught_place':
            self.verify_taught_place()
            return
        self.verify_hold()
        service = self.config['support_service']
        rospy.wait_for_service(service, timeout=2.)
        requested = rospy.Time.now()
        # A stalled external verifier must not prevent cancellation indefinitely.
        import queue
        import threading
        replies = queue.Queue(maxsize=1)
        def query_support():
            try:
                replies.put(rospy.ServiceProxy(service, VerifySupport)(VerifySupportRequest(self.task_id, requested)))
            except Exception as error:
                replies.put(error)
        threading.Thread(target=query_support, daemon=True).start()
        deadline = time.monotonic()+2.
        while True:
            self.check()
            if time.monotonic() > deadline:
                raise RuntimeError('support verification timed out')
            try:
                response = replies.get(timeout=.02)
                break
            except queue.Empty:
                continue
        if isinstance(response, Exception):
            raise response
        self.check()
        age = (rospy.Time.now()-response.measured_at).to_sec()
        if not response.supported or response.source != self.config['support_source'] or not 0 <= age <= .3 or response.measured_at < requested:
            raise RuntimeError('support not independently verified: '+response.reason)

    def open(self):
        self.check()
        self.gripper.open(self.config['open_width_m'])

    def detach(self):
        self.placed = True
        self.set_basket(False)

    def stop(self):
        self.s.stop()
