"""ROS/MoveIt bridge. Constructing RobotSession reads state only; execute is explicit."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
from planner_metrics import (PlannerMetrics,profiled,PlanningTimeout,IKFailure,CollisionFailure,JointConstraintFailure)
import rospy
import moveit_commander
import actionlib
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, DisplayTrajectory, PlanningScene, CollisionObject, AttachedCollisionObject, PlanningSceneComponents, AllowedCollisionEntry
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, ApplyPlanningScene, GetPlanningScene, GetPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from visualization_msgs.msg import Marker, MarkerArray
from rm_msgs.msg import Gripper_Set, Gripper_Pick, Manual_Set_Force_Pose, Joint_En_Flag
from std_msgs.msg import Empty
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from tf.transformations import quaternion_from_matrix
from grasp_geometry import (transform, flange_target, rotation_matrix,
                            stl_triangles, observed_obstacle_points, collision_samples,sample_bounds,bounded_sample_clearance)
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'usr/calibration/rml63_handeye/scripts'))
from publish_joint_calibration import load_config


def pose_msg(t):
    m = Pose()
    m.position.x, m.position.y, m.position.z = t[:3, 3]
    q = quaternion_from_matrix(t)
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
    return m


def stamped(t, frame, stamp=None):
    m = PoseStamped()
    m.header.frame_id, m.header.stamp = frame, stamp or rospy.Time.now()
    m.pose = pose_msg(t)
    return m


def state_after(state, trajectory):
    out = deepcopy(state)
    updates = dict(zip(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[-1].positions))
    out.joint_state.position = [updates.get(n, p) for n, p in zip(out.joint_state.name, out.joint_state.position)]
    out.joint_state.velocity = []; out.joint_state.effort = []
    out.joint_state.header.stamp = rospy.Time(0)
    out.is_diff = False
    return out


def open_finger_collision_matrix(matrix, finger_links, world_id, tool_id):
    """Replace frozen finger/world tests while retaining the full open CAD test."""
    if tool_id == world_id or tool_id in finger_links:
        raise ValueError('open tool must have a distinct collision object ID')
    out=deepcopy(matrix)
    if len(out.entry_values)!=len(out.entry_names) or any(len(row.enabled)!=len(out.entry_names) for row in out.entry_values):
        raise ValueError('invalid collision matrix')
    # Explicitly keep open-tool/world collision checking enabled, regardless
    # of a matrix left behind by an earlier application.
    for name in list(finger_links)+[world_id,tool_id]:
        if name not in out.entry_names:
            for row in out.entry_values:row.enabled.append(False)
            out.entry_names.append(name)
            out.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(out.entry_names)))
    world=out.entry_names.index(world_id);tool=out.entry_names.index(tool_id)
    out.entry_values[world].enabled[tool]=False
    out.entry_values[tool].enabled[world]=False
    for name in finger_links:
        finger=out.entry_names.index(name)
        out.entry_values[finger].enabled[world]=True
        out.entry_values[world].enabled[finger]=True
    return out


class RobotSession:
    def __init__(self, config):
        self.metrics=PlannerMetrics()
        self.config = config
        self.xml = rospy.get_param('/robot_description')
        self.cal, self.chain, self.offsets, self.handeye = load_config(config['calibration_file'], self.xml)
        self.calibration_hash = hashlib.sha256(Path(config['calibration_file']).read_bytes()).hexdigest()
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(config['planning_group'])
        self.group.set_end_effector_link(config['flange_frame'])
        self.group.set_pose_reference_frame(config['base_frame'])
        self.group.set_max_velocity_scaling_factor(config['velocity_scaling'])
        self.group.set_max_acceleration_scaling_factor(config['acceleration_scaling'])
        self.group.set_goal_position_tolerance(.001)
        self.group.set_goal_orientation_tolerance(math.radians(.5))
        self.group.set_planning_time(8.)
        self.active = self.group.get_active_joints()
        if self.active != self.cal['joint_names']:
            raise RuntimeError('unexpected planning joint order')
        self.moving = False
        self.motion_client = actionlib.SimpleActionClient('/rm_group/follow_joint_trajectory',FollowJointTrajectoryAction)
        self.stop_pub = rospy.Publisher('/rm_driver/Emergency_Stop',Empty,queue_size=1)
        self.fault_message=None
        self.fault_sub=rospy.Subscriber('/rm_driver/JointErrorCode',Manual_Set_Force_Pose,
            lambda msg:setattr(self,'fault_message',msg),queue_size=1)
        self.minimum_observed_clearance_m = float('inf')
        self.names = []
        # Dense waypoint checks use the same service many times. Reuse its TCP
        # connection rather than negotiating a fresh ROS connection per state.
        # A server disconnect still raises and stops planning; no motion/retry.
        self.scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene,persistent=True)
        self.validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity,persistent=True)
        self.display = rospy.Publisher('/depth_handle_grasp/display_planned_path', DisplayTrajectory, queue_size=1, latch=True)
        self.camera_pub = rospy.Publisher('/depth_handle_grasp/pose_camera', PoseStamped, queue_size=1, latch=True)
        self.base_pub = rospy.Publisher('/depth_handle_grasp/pose_base', PoseStamped, queue_size=1, latch=True)
        self.pre_pub = rospy.Publisher('/depth_handle_grasp/pregrasp_base', PoseStamped, queue_size=1, latch=True)
        self.markers = rospy.Publisher('/depth_handle_grasp/markers', MarkerArray, queue_size=1, latch=True)
        self.capture_frame = 'depth_handle_grasp/captured_depth_optical'
        self.capture_tf = tf2_ros.StaticTransformBroadcaster()
        rospy.on_shutdown(self.stop)

    def stop(self):
        if self.moving:
            self.motion_client.cancel_goal()
            self.stop_pub.publish(Empty())
            self.moving = False

    def measured(self):
        msg = rospy.wait_for_message('/joint_states', JointState, timeout=2.)
        age = (rospy.Time.now()-msg.header.stamp).to_sec()
        if not -.05 <= age <= self.config['joint_feedback_max_age_sec']:
            raise RuntimeError('joint feedback stale')
        values = dict(zip(msg.name, msg.position))
        q = np.array([values[n] for n in self.active])
        if not np.isfinite(q).all():
            raise RuntimeError('invalid joint feedback')
        return q, msg.header.stamp

    def verify_hardware(self,require_execution=True):
        import rosgraph
        pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        if dict(pubs).get('/joint_states') != ['/rm_driver']:
            raise RuntimeError('joint_states must come exclusively from real rm_driver')
        faults = rospy.wait_for_message('/rm_driver/JointErrorCode', Manual_Set_Force_Pose, timeout=3)
        enabled = rospy.wait_for_message('/rm_driver/Udp_Joint_En_Flag', Joint_En_Flag, timeout=3)
        if list(faults.joint) != [0]*6 or list(enabled.joint_en_flag) != [True]*6:
            raise RuntimeError('joint fault or disabled joint')
        if require_execution and not rospy.get_param('/move_group/allow_trajectory_execution', False):
            raise RuntimeError('real trajectory execution disabled')
        self.measured()

    def apply(self, world=None, attached=None, collision_matrix=None):
        scene = PlanningScene(); scene.is_diff = True; scene.robot_state.is_diff = True
        scene.world.collision_objects = world or []
        scene.robot_state.attached_collision_objects = attached or []
        if collision_matrix is not None:scene.allowed_collision_matrix=collision_matrix
        if not self.scene(scene).success:
            raise RuntimeError('planning scene update rejected')

    def self_filter(self, points, q):
        import rospkg
        root=ET.fromstring(self.xml)
        transforms={self.config['base_frame']:np.eye(4)}
        values=dict(zip(self.active,q+self.offsets))
        remaining=list(root.findall('joint'))
        while remaining:
            progress=False
            for joint in remaining[:]:
                parent,child=joint.find('parent').get('link'),joint.find('child').get('link')
                node=joint.find('origin')
                xyz=np.fromstring(node.get('xyz','0 0 0') if node is not None else '0 0 0',sep=' ')
                rpy=np.fromstring(node.get('rpy','0 0 0') if node is not None else '0 0 0',sep=' ')
                relative=transform(xyz,rotation_matrix(Rotation.from_euler('xyz',rpy)))
                if joint.get('type')!='fixed':
                    axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                    relative[:3,:3]=relative[:3,:3]@rotation_matrix(Rotation.from_rotvec(axis*values.get(joint.get('name'),0.)))
                if parent in transforms:
                    transforms[child]=transforms[parent]@relative
                elif child in transforms:
                    transforms[parent]=transforms[child]@np.linalg.inv(relative)
                else:continue
                remaining.remove(joint);progress=True
            if not progress:raise RuntimeError('cannot build full robot collision-frame tree')
        keep=np.ones(len(points),bool)
        package=rospkg.RosPack()
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                name=link.get('name');frame=transforms[name]
                origin=collision.find('origin')
                if origin is not None:
                    frame=frame@transform(np.fromstring(origin.get('xyz','0 0 0'),sep=' '),
                        rotation_matrix(Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' '))))
                geometry=collision.find('geometry');mesh=geometry.find('mesh');box=geometry.find('box')
                if mesh is not None:
                    path=mesh.get('filename')
                    if path.startswith('package://'):
                        pkg,relative=path[len('package://'):].split('/',1);path=Path(package.get_path(pkg))/relative
                    vertices=stl_triangles(path)['vertices'].reshape(-1,3)
                    scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ');vertices=vertices*scale
                    lower,upper=vertices.min(0),vertices.max(0)
                elif box is not None:
                    half=np.fromstring(box.get('size'),sep=' ')/2;lower,upper=-half,half
                else:raise RuntimeError('unsupported self-filter primitive')
                local=(points-frame[:3,3])@frame[:3,:3]
                inside=np.all((local>=lower-.010)&(local<=upper+.010),axis=1)
                keep &= ~inside
        return keep

    @profiled('scene_build')
    def add_scene(self, base_depth, depth, meta, handle_mask, support, gripper, open_angle,capture_q=None):
        # Partial observed obstacle geometry. No claim about unobserved surfaces/cables.
        from scipy.spatial import cKDTree
        z = depth.astype(float)*meta['depth_scale_m_per_unit']
        v, u = np.nonzero((depth > 0) & (z >= .25) & (z <= 1.))
        k = meta['intrinsics']; zz = z[v, u]
        points = np.column_stack(((u-k['ppx'])*zz/k['fx'], (v-k['ppy'])*zz/k['fy'], zz))
        points = points @ base_depth[:3, :3].T+base_depth[:3, 3]
        self.full_observed = points
        if capture_q is None:q,_=self.measured()
        else:
            q=np.asarray(capture_q,float)
            if q.shape!=(len(self.active),) or not np.isfinite(q).all():
                raise ValueError('invalid captured joint state for scene reconstruction')
        outside_robot=self.self_filter(points,q)
        target_mask = handle_mask[v, u].astype(bool)
        if np.any(~outside_robot & target_mask):
            raise RuntimeError('target observations overlap the captured robot model')
        # Target region excludes the robot/camera around the observation pose.
        target = self.target[:3, 3]
        nearby = (np.linalg.norm(points-target, axis=1) < .24) & outside_robot
        table_n = base_depth[:3, :3] @ np.asarray(support['outward_normal'])
        table_c = base_depth[:3, :3] @ np.asarray(support['plane_center_m'])+base_depth[:3, 3]
        if table_n[2] < .95:
            raise RuntimeError('table normal inconsistent with confirmed horizontal base plane')
        table = CollisionObject(); table.header.frame_id = self.config['base_frame']; table.id = 'depth_grasp_table'; table.operation = CollisionObject.ADD
        normal = table_n/np.linalg.norm(table_n)
        ex = np.array([1., 0., 0.]); ex -= normal*(ex @ normal); ex /= np.linalg.norm(ex)
        rot = np.column_stack((ex, np.cross(normal, ex), normal))
        table_center = target-normal*np.dot(target-table_c,normal)
        primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.48, .65, .10])
        table.primitives=[primitive]; table.primitive_poses=[pose_msg(transform(table_center-normal*.05, rot))]
        # Exclude the target from world voxels because the running URDF freezes
        # fingers closed. Actual OPEN-finger target clearance is checked separately.
        not_handle = ~target_mask
        above = ((points-table_c) @ table_n) > .010
        obstacle = points[nearby & not_handle & above]
        voxel = self.config['scene_voxel_m']
        centers,detail_points=collision_samples(obstacle,target,voxel,
            self.config.get('scene_grasp_detail_radius_m',.06))
        body = CollisionObject(); body.header.frame_id=self.config['base_frame']; body.id='depth_grasp_basket'; body.operation=CollisionObject.ADD
        size = voxel+2*self.config['scene_margin_m']
        for c in centers:
            body.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size]*3))
            body.primitive_poses.append(pose_msg(transform(c, np.eye(3))))
        # Near the contact use exact native sample positions with the SAME
        # safety margin. Coarse padded cubes can reach well beyond that margin
        # and falsely fill the space between an open finger and the handle.
        # Every observed non-target point remains represented, including thin
        # obstacles and edge returns; none are dropped as depth outliers.
        for point in detail_points:
            body.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE,
                dimensions=[self.config['scene_margin_m']]))
            body.primitive_poses.append(pose_msg(transform(point,np.eye(3))))
        attachment=AttachedCollisionObject(); attachment.link_name=self.config['flange_frame']
        attachment.touch_links=[self.config['flange_frame'], 'fixed_plate_link']+list(gripper.meshes)
        obj=attachment.object; obj.header.frame_id=self.config['flange_frame']; obj.id='depth_grasp_open_tool'; obj.operation=CollisionObject.ADD
        tool_angles=[open_angle]
        for name, triangles in gripper.meshes.items():
            vertices, indices=np.unique(triangles['vertices'].reshape(-1,3),axis=0,return_inverse=True)
            mesh=Mesh()
            from geometry_msgs.msg import Point
            mesh.vertices=[Point(*map(float,p)) for p in vertices]
            mesh.triangles=[MeshTriangle(vertex_indices=list(map(int,idx))) for idx in indices.reshape(-1,3)]
            for angle in tool_angles:
                obj.meshes.append(mesh); obj.mesh_poses.append(pose_msg(gripper.link(name,angle)))
        # Conservative camera body around the optical centre, not an invented exact CAD.
        camera_center=(self.handeye @ np.array([0.,0.,-.013,1.]))[:3]
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[self.config['camera_radius_m']]))
        obj.primitive_poses.append(pose_msg(transform(camera_center,np.eye(3))))
        finger_links=['4C2_Link5','4C2_Link6']
        if not all(name in gripper.meshes for name in finger_links):
            raise RuntimeError('exact open finger CAD missing; cannot replace frozen collision geometry')
        request=GetPlanningSceneRequest()
        request.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)(request).scene.allowed_collision_matrix
        matrix=open_finger_collision_matrix(matrix,finger_links,body.id,obj.id)
        self.apply([table,body],[attachment],matrix);self.names=[table.id,body.id,obj.id]
        self.open_tool_attachment=deepcopy(attachment)
        # The target is intentionally inside the open gripper at the grasp
        # pose. Check the open tool only against non-target observed geometry.
        self.collision_points=observed_obstacle_points(points, nearby, above, target_mask)
        self.collision_tree=cKDTree(self.collision_points)
        # Dense triangle samples for observed target/obstacle clearance. Each tool
        # mesh is sampled at <=3 mm pitch; this supplements the frozen URDF model.
        samples=[];self.tool_local_samples={}
        for name, triangles in gripper.meshes.items():
            patches=[]
            for tri in triangles['vertices'].astype(float):
                count=max(1,int(np.ceil(max(np.linalg.norm(tri[1]-tri[0]),np.linalg.norm(tri[2]-tri[0]),np.linalg.norm(tri[2]-tri[1]))/.003)))
                ii,jj=np.meshgrid(np.arange(count+1),np.arange(count+1));valid=ii+jj<=count
                aa,bb=ii[valid]/count,jj[valid]/count
                patches.append(tri[0]+aa[:,None]*(tri[1]-tri[0])+bb[:,None]*(tri[2]-tri[0]))
            tri=np.vstack(patches)
            _,unique=np.unique(np.floor(tri/.002).astype(int),axis=0,return_index=True)
            tri=tri[unique]
            self.tool_local_samples[name]=tri
            for angle in tool_angles:
                t=gripper.link(name,angle)
                samples.append(tri @ t[:3,:3].T+t[:3,3])
        self.tool_samples=np.vstack(samples)
        self.tool_sample_groups=sample_bounds(samples)
        if isinstance(getattr(self,'metrics',None),PlannerMetrics):self.metrics.counts['cad_tool_samples']=len(self.tool_samples)
        self.tool_geometry=gripper
        self.current_tool_width_m=self.config['open_width_m']
        return dict(table_normal_base=table_n.tolist(),table_center_base=table_c.tolist(),
                    obstacle_object_semantics='depth_grasp_basket contains local observed non-target points, including possible cables; not semantic basket segmentation',
                    basket_obstacle_voxels=len(centers),observed_cloud_points=len(self.collision_points),
                    grasp_detail_native_point_spheres=len(detail_points),
                    grasp_detail_sphere_radius_m=self.config['scene_margin_m'],
                    observed_obstacle_points=len(self.collision_points),
                    observed_robot_points_filtered=int(np.count_nonzero(~outside_robot)),
                    finger_world_collision_model='phase-specific attached CAD replaces duplicate frozen finger/world tests',
                    approach_tool_width_m=self.config['open_width_m'],
                    lift_tool_width_m=self.config['contact_width_m'],
                    camera_collision_model='45 mm conservative sphere',unobserved_geometry_verified=False)

    def set_tool_width(self,width):
        """Update planning geometry only; this sends no hardware command."""
        if np.isclose(width,self.current_tool_width_m,atol=1e-8):return
        angle=self.tool_geometry.at_width(width)['linkage_angle_rad']
        attachment=deepcopy(self.open_tool_attachment)
        names=list(self.tool_geometry.meshes)
        if len(attachment.object.meshes)!=len(names):
            raise RuntimeError('phase-specific gripper mesh mapping invalid')
        attachment.object.mesh_poses=[pose_msg(self.tool_geometry.link(name,angle)) for name in names]
        samples=[]
        for name in names:
            t=self.tool_geometry.link(name,angle)
            samples.append(self.tool_local_samples[name]@t[:3,:3].T+t[:3,3])
        self.apply(attached=[attachment])
        self.open_tool_attachment=attachment
        self.tool_samples=np.vstack(samples)
        self.tool_sample_groups=sample_bounds(samples)
        self.current_tool_width_m=width

    def state_for(self,q,reference=None):
        if reference is None:
            state=self.robot.get_current_state()
        else:
            state=RobotState()
            state.joint_state=deepcopy(reference.joint_state)
            state.multi_dof_joint_state=deepcopy(reference.multi_dof_joint_state)
            state.attached_collision_objects=list(reference.attached_collision_objects)
        if hasattr(self,'open_tool_attachment'):
            # A requested RobotState must carry the same replacement geometry
            # as the world scene, even before the commander's cache updates.
            state.attached_collision_objects=[a for a in state.attached_collision_objects
                if a.object.id!=self.open_tool_attachment.object.id]+[self.open_tool_attachment]
        values=dict(zip(self.active,q))
        state.joint_state.position=[values.get(n,p) for n,p in zip(state.joint_state.name,state.joint_state.position)]
        state.joint_state.header.stamp=rospy.Time(0);state.is_diff=False
        return state

    def check_planning_deadline(self):
        deadline=getattr(self,'planning_deadline',None)
        if deadline is not None and time.monotonic()>=deadline:
            raise PlanningTimeout('planning time budget exceeded; candidate rejected')

    @profiled('collision_state')
    def check_state(self,q):
        self.check_planning_deadline()
        # Also check the corrected FK branch against the same model/scene, since
        # camera calibration offsets are NOT installed into the MoveIt URDF.
        for basis,values in (('nominal',q),('calibrated_offset',q+self.offsets)):
            request=GetStateValidityRequest()
            # The synchronous apply()/set_tool_width() already installs the CAD
            # attachment in move_group's scene. A RobotState diff preserves it
            # (MoveIt robotStateMsgToRobotState), while replacing only arm joints.
            # Avoid copying/serializing every STL triangle twice per waypoint.
            request.robot_state.is_diff=True
            request.robot_state.joint_state.name=list(self.active)
            request.robot_state.joint_state.position=np.asarray(values,float).tolist()
            request.group_name=self.config['planning_group']
            started=time.monotonic()
            result=self.validity(request)
            metrics=getattr(self,'metrics',None)
            if isinstance(metrics,PlannerMetrics):
                metrics.counts['collision_service_calls']+=1
                metrics.samples['collision_service'].append(time.monotonic()-started)
            self.check_planning_deadline()
            if not result.valid:
                contacts=[(c.contact_body_1,c.contact_body_2) for c in result.contacts[:5]]
                error=CollisionFailure('state collision: '+str(contacts))
                error.joint_state_rad=np.asarray(values).tolist();error.contacts=contacts
                error.joint_state_basis=basis
                error.reported_joint_state_rad=np.asarray(q).tolist()
                raise error
        if hasattr(self,'collision_tree'):
            flange=self.chain.fk(q,self.offsets)
            started=time.monotonic()
            if self.config.get('cad_clearance_broadphase',True) and hasattr(self,'tool_sample_groups'):
                clearance,pruned,queried=bounded_sample_clearance(self.collision_tree,self.tool_sample_groups,flange,self.config['scene_margin_m'])
                if isinstance(metrics,PlannerMetrics):
                    metrics.counts['cad_groups_proven_clear']+=pruned
                    metrics.counts['cad_dense_points_queried']+=queried
            else:
                points=self.tool_samples @ flange[:3,:3].T+flange[:3,3]
                distances,_=self.collision_tree.query(points,k=1)
                clearance=float(min(distances)) if len(distances) else float('inf')
            if isinstance(metrics,PlannerMetrics):metrics.samples['observed_cad_clearance'].append(time.monotonic()-started)
            self.minimum_observed_clearance_m=min(self.minimum_observed_clearance_m,clearance)
            if clearance<self.config['scene_margin_m']:
                raise CollisionFailure('open-tool to observed depth clearance below margin: %.2f mm'%(clearance*1000))
            camera_center=(flange@self.handeye@np.array([0.,0.,-.013,1.]))[:3]
            camera_distance=float(self.collision_tree.query(camera_center,k=1)[0])-self.config['camera_radius_m']
            if camera_distance<self.config['scene_margin_m']:
                raise CollisionFailure('camera bounding sphere too close to observed object')

    @profiled('ik')
    def solve(self,desired_flange,seed):
        self.check_planning_deadline()
        from scipy.optimize import least_squares
        # Reported joint coordinates are solved against the corrected kinematic
        # chain, rather than mixing corrected camera FK with nominal flange IK.
        lower=[];upper=[]
        for name in self.active:
            low,high=self.robot.get_joint(name).bounds()
            lower.append(low+.005);upper.append(high-.005)
        def residual(q):
            self.check_planning_deadline()
            current=self.chain.fk(q,self.offsets)
            rot=Rotation.from_dcm(desired_flange[:3,:3].T @ current[:3,:3]).as_rotvec()
            return np.r_[(current[:3,3]-desired_flange[:3,3])/.001,rot/.01]
        started=time.monotonic()
        fitted=least_squares(residual,np.clip(seed,np.array(lower)+.001,np.array(upper)-.001),
                             bounds=(lower,upper),max_nfev=250,xtol=1e-10,ftol=1e-10,gtol=1e-10)
        q=fitted.x;current=self.chain.fk(q,self.offsets)
        position=np.linalg.norm(current[:3,3]-desired_flange[:3,3])
        angle=np.linalg.norm(Rotation.from_dcm(desired_flange[:3,:3].T@current[:3,:3]).as_rotvec())
        metrics=getattr(self,'metrics',None)
        if isinstance(metrics,PlannerMetrics):
            metrics.samples['ik_numeric_solver'].append(time.monotonic()-started)
            metrics.counts['ik_solver_nfev']+=int(fitted.nfev)
            metrics.error_values['ik_fk_position_m'].append(float(position))
            metrics.error_values['ik_fk_rotation_rad'].append(float(angle))
        if not fitted.success or position>.001 or angle>math.radians(.5):
            raise IKFailure('corrected-chain IK did not converge: position error %.2f mm, rotation error %.2f deg'%(
                position*1000,math.degrees(angle)))
        self.check_state(q)
        return q

    def check_trajectory(self,trajectory,start):
        jt=trajectory.joint_trajectory
        if set(jt.joint_names)!=set(self.active) or not jt.points:
            raise RuntimeError('invalid/empty joint trajectory')
        initial=dict(zip(start.joint_state.name,start.joint_state.position))
        q0=np.array([initial[n] for n in jt.joint_names])
        positions=np.array([p.positions for p in jt.points])
        if not np.isfinite(positions).all() or np.max(abs(positions[0]-q0))>.02:
            raise RuntimeError('trajectory start mismatch')
        if np.max(abs(positions-q0))>math.radians(self.config['max_joint_travel_deg']):
            raise RuntimeError('trajectory joint travel too large')
        if len(positions)>1 and np.max(abs(np.diff(positions,axis=0)))>math.radians(self.config['max_joint_jump_deg']):
            raise RuntimeError('trajectory joint jump')
        times=[p.time_from_start.to_sec() for p in jt.points]
        if not np.isfinite(times).all() or times[0]<0 or np.any(np.diff(times)<=0):
            raise RuntimeError('trajectory timing invalid')
        for index in range(len(positions)):
            self.check_planning_deadline()
            values=dict(zip(jt.joint_names,positions[index]));q=np.array([values[n] for n in self.active])
            self.check_state(q)
        return positions

    def dense_timed(self,trajectory,start):
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        positions=np.asarray([p.positions for p in trajectory.joint_trajectory.points])
        if positions.ndim!=2 or positions.shape[1]!=6 or not np.isfinite(positions).all():
            raise RuntimeError('cannot retime invalid trajectory')
        dense=RobotTrajectory();dense.joint_trajectory.joint_names=list(trajectory.joint_trajectory.joint_names)
        for i,q in enumerate(positions):
            if i==0:segments=[q]
            else:
                previous=positions[i-1]
                count=max(1,int(np.ceil(np.max(abs(q-previous))/np.radians(2))))
                segments=[previous+(q-previous)*alpha for alpha in np.linspace(0,1,count+1)[1:]]
            for value in segments:
                point=JointTrajectoryPoint();point.positions=value.tolist();dense.joint_trajectory.points.append(point)
        return self.group.retime_trajectory(start,dense,self.config['velocity_scaling'],self.config['acceleration_scaling'])

    @profiled('pregrasp_joint_path')
    def plan_joint(self,q,start):
        self.check_planning_deadline()
        remaining=8. if getattr(self,'planning_deadline',None) is None else max(.01,min(8.,self.planning_deadline-time.monotonic()))
        self.group.set_planning_time(remaining)
        self.group.set_start_state(start)
        self.group.set_joint_value_target(dict(zip(self.active,q)))
        answer=self.group.plan()
        self.check_planning_deadline()
        success,trajectory=(answer[0],answer[1]) if isinstance(answer,tuple) else (True,answer)
        if not success:raise RuntimeError('MoveIt approach plan failed')
        trajectory=self.dense_timed(trajectory,start)
        self.check_trajectory(trajectory,start)
        return trajectory

    def _cartesian_joints(self,flange_tcp,start_tcp,end_tcp,seed,step):
        if not np.allclose(start_tcp[:3,:3],end_tcp[:3,:3],atol=1e-8):
            raise ValueError('Cartesian position segment must preserve orientation')
        retries=int(self.config.get('cartesian_local_retry_depth',2))
        if not 0<=retries<=3:raise ValueError('local IK subdivision depth must be 0..3')
        count=max(2,int(np.ceil(np.linalg.norm(end_tcp[:3,3]-start_tcp[:3,3])/step)))
        metrics=getattr(self,'metrics',None)
        def advance(a,b,seed,depth):
            self.check_planning_deadline()
            try:
                q=self.solve(flange_target(b,flange_tcp),seed)
                if np.max(abs(q-seed))>math.radians(self.config['max_joint_jump_deg']):
                    raise JointConstraintFailure('calibrated Cartesian IK branch jump')
            except (IKFailure,JointConstraintFailure):
                if depth>=retries:raise
                if isinstance(metrics,PlannerMetrics):metrics.counts['local_ik_subdivisions']+=1
                middle=a.copy();middle[:3,3]=(a[:3,3]+b[:3,3])/2
                first=advance(a,middle,seed,depth+1)
                return first+advance(middle,b,first[-1],depth+1)
            if isinstance(metrics,PlannerMetrics):metrics.counts['cartesian_generated_points']+=1
            return [q]
        joints=[];previous=start_tcp
        for fraction in np.linspace(0,1,count+1)[1:]:
            target=start_tcp.copy();target[:3,3]=start_tcp[:3,3]*(1-fraction)+end_tcp[:3,3]*fraction
            points=advance(previous,target,seed,0)
            joints.extend(points);seed=points[-1];previous=target
        return joints

    @profiled('cartesian_coarse')
    def coarse_check(self,flange_tcp,start_tcp,end_tcp,seed):
        step=float(self.config.get('cartesian_coarse_step_m',.009))
        if not .003<=step<=.010:raise ValueError('experimental coarse step must be 3..10 mm')
        # This returns a seed only, never an executable/coarsely checked trajectory.
        return self._cartesian_joints(flange_tcp,start_tcp,end_tcp,np.asarray(seed),step)[-1]

    @profiled('cartesian_fine')
    def plan_descent(self,flange_tcp,start_tcp,end_tcp,start_state):
        self.check_planning_deadline()
        step=float(self.config.get('cartesian_fine_step_m',.003))
        if not np.isfinite(step) or not 0<step<=.003:
            raise ValueError('fine validation spacing must be positive and no larger than 3 mm')
        first=np.array([dict(zip(start_state.joint_state.name,start_state.joint_state.position))[n] for n in self.active])
        joints=self._cartesian_joints(flange_tcp,start_tcp,end_tcp,first,step)
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=self.active
        for q in [first]+joints:
            pt=JointTrajectoryPoint();pt.positions=q.tolist();trajectory.joint_trajectory.points.append(pt)
        timed=self.group.retime_trajectory(start_state,trajectory,self.config['velocity_scaling'],self.config['acceleration_scaling'])
        # Final trajectory is always checked at the original fine coverage,
        # including nominal and offset collision states. Coarse success cannot
        # reach the candidate solution collection without this validation.
        self.check_trajectory(timed,start_state)
        return timed

    def publish(self,camera_pose,base_pose,pregrasp,stamp,plans=None,start=None):
        captured = TransformStamped()
        captured.header.frame_id=self.config['base_frame'];captured.header.stamp=stamp
        captured.child_frame_id=self.capture_frame
        captured.transform.translation.x,captured.transform.translation.y,captured.transform.translation.z=self.base_depth[:3,3]
        quat=quaternion_from_matrix(self.base_depth)
        (captured.transform.rotation.x,captured.transform.rotation.y,captured.transform.rotation.z,captured.transform.rotation.w)=quat
        self.capture_tf.sendTransform(captured)
        self.camera_pub.publish(stamped(camera_pose,self.capture_frame,stamp))
        self.base_pub.publish(stamped(base_pose,self.config['base_frame'],stamp))
        self.pre_pub.publish(stamped(pregrasp,self.config['base_frame'],stamp))
        array=MarkerArray()
        for i,(t,color) in enumerate([(base_pose,(0.,1.,0.)),(pregrasp,(0.,1.,1.))]):
            marker=Marker();marker.header.frame_id=self.config['base_frame'];marker.header.stamp=stamp
            marker.ns='grasp_targets';marker.id=i;marker.type=Marker.ARROW;marker.action=Marker.ADD
            marker.pose=pose_msg(t);marker.scale.x=.05;marker.scale.y=.006;marker.scale.z=.006
            marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=1.;array.markers.append(marker)
        self.markers.publish(array)
        if plans:
            msg=DisplayTrajectory();msg.trajectory_start=start;msg.trajectory=plans;self.display.publish(msg)

    def execute_one(self,trajectory,flange_tcp,expected_tcp):
        self.verify_hardware()
        q,_=self.measured();q0=dict(zip(trajectory.joint_trajectory.joint_names,trajectory.joint_trajectory.points[0].positions))
        if max(abs(q[i]-q0[n]) for i,n in enumerate(self.active))>math.radians(.5):
            raise RuntimeError('robot changed after planning')
        if not self.motion_client.wait_for_server(rospy.Duration(3)):
            raise RuntimeError('real joint trajectory action server unavailable')
        goal=FollowJointTrajectoryGoal();goal.trajectory=deepcopy(trajectory.joint_trajectory)
        goal.trajectory.header.stamp=rospy.Time.now()+rospy.Duration(.2)
        planned=np.asarray([p.positions for p in trajectory.joint_trajectory.points])
        duration=trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        started=time.monotonic();last_progress=started
        self.moving=True
        try:
            self.motion_client.send_goal(goal)
            while not self.motion_client.wait_for_result(rospy.Duration(.1)):
                q,_=self.measured()
                if self.fault_message is not None and any(self.fault_message.joint):
                    raise RuntimeError('joint fault during execution')
                nearest=np.min(np.max(abs(planned-q),axis=1))
                if nearest>math.radians(3):raise RuntimeError('measured joints deviated from planned path')
                if time.monotonic()-started>duration*1.5+5:
                    raise RuntimeError('real trajectory timed out')
                if time.monotonic()-last_progress>10:
                    print('Moving at low speed: %.0f / %.0f seconds'%(time.monotonic()-started,duration),flush=True)
                    last_progress=time.monotonic()
            answer=self.motion_client.get_result()
            if self.motion_client.get_state()!=3 or answer is None or answer.error_code!=0:
                raise RuntimeError('real joint trajectory controller returned failure')
            self.moving=False
        except Exception:
            self.stop()
            raise
        rospy.sleep(.3)
        q,_=self.measured();achieved=self.chain.fk(q,self.offsets)@flange_tcp
        distance=np.linalg.norm(achieved[:3,3]-expected_tcp[:3,3])
        angle=np.degrees(np.linalg.norm(Rotation.from_dcm(expected_tcp[:3,:3].T@achieved[:3,:3]).as_rotvec()))
        if distance>self.config['position_tolerance_m'] or angle>self.config['orientation_tolerance_deg']:
            raise RuntimeError('measured calibrated TCP did not reach target: %.2f mm / %.2f deg'%(distance*1000,angle))
        return dict(measured_tcp_base=achieved.tolist(),position_model_error_m=float(distance),orientation_model_error_deg=float(angle))


class GripperIO:
    def __init__(self,config):
        self.config=config
        self.position_pub=rospy.Publisher('/rm_driver/Gripper_Set',Gripper_Set,queue_size=1)
        self.pick_pub=rospy.Publisher('/rm_driver/Gripper_Pick',Gripper_Pick,queue_size=1)

    def query(self,command):
        if command not in ('get_gripper_state','get_tool_voltage','get_current_tool_frame'):
            raise ValueError('query is read-only')
        with socket.create_connection((self.config['robot_ip'],self.config['robot_port']),timeout=2.) as s:
            s.settimeout(2.);s.sendall((json.dumps({'command':command})+'\r\n').encode())
            data=b''
            while b'\r\n' not in data:
                packet=s.recv(8192)
                if not packet:raise RuntimeError('robot read-only query disconnected')
                data+=packet
            return json.loads(data.split(b'\r\n')[0])

    def _connected(self,publisher):
        deadline=time.monotonic()+3
        while publisher.get_num_connections()==0 and time.monotonic()<deadline:rospy.sleep(.02)
        if publisher.get_num_connections()==0:raise RuntimeError('gripper driver subscriber unavailable')

    def open(self,width):
        if self.query('get_tool_voltage').get('voltage_type')!=3:
            raise RuntimeError('EG2 tool power is not 24V; check tool wiring/configuration')
        self._connected(self.position_pub)
        target=int(round(width/.070*1000))
        self.position_pub.publish(Gripper_Set(position=target))
        deadline=time.monotonic()+self.config['gripper_timeout_sec']
        history=[]
        while time.monotonic()<deadline:
            state=self.query('get_gripper_state');history.append(state)
            if state.get('error',0):raise RuntimeError('gripper reported hardware error')
            if state.get('status')==1 and state.get('enable')==1 and abs(state.get('actpos',-1000)-target)<35 and state.get('mode') in (1,2,3) and abs(state.get('current_force',10000))<50:
                recent=history[-3:]
                if (len(recent)==3 and all(s.get('status')==1 and s.get('enable')==1 and
                    s.get('mode') in (1,2,3) and abs(s.get('actpos',-1000)-target)<35 and
                    abs(s.get('current_force',10000))<50 for s in recent) and
                    max(s['actpos'] for s in recent)-min(s['actpos'] for s in recent)<15):
                    return dict(commanded_position=target,state=state,stable_feedback_samples=recent)
            rospy.sleep(.15)
        raise RuntimeError('gripper did not verify open state: '+str(history[-1] if history else None))

    def close(self):
        self._connected(self.pick_pub)
        self.pick_pub.publish(Gripper_Pick(speed=self.config['gripper_speed'],force=self.config['gripper_force']))
        deadline=time.monotonic()+self.config['gripper_timeout_sec']
        history=[]
        while time.monotonic()<deadline:
            state=self.query('get_gripper_state')
            history.append(dict(state))
            if state.get('error',0):raise RuntimeError('gripper error during force-controlled closure')
            recent=history[-3:]
            if len(recent)==3:
                valid=all(s.get('status')==1 and s.get('enable')==1 and s.get('error',0)==0 for s in recent)
                if valid and all(s.get('mode')==6 and s.get('actpos',0)>30 for s in recent):
                    if max(s['actpos'] for s in recent)-min(s['actpos'] for s in recent)<15:
                        return dict(state,stable_feedback_samples=recent)
                # mode=2 also means "at commanded OPEN position". It cannot
                # prove empty closure unless actual opening is near zero and
                # stable across several samples after the command.
                if valid and all(s.get('mode')==2 and 0<=s.get('actpos',-1)<=30 for s in recent):
                    error=RuntimeError('gripper fully closed without detecting an object')
                    error.gripper_feedback=history
                    raise error
            rospy.sleep(.15)
        error=RuntimeError('force-controlled contact not verified; last feedback: '+str(history[-1] if history else None))
        error.gripper_feedback=history
        raise error
