#!/usr/bin/env python3
"""Existing detector output -> multiple CAD-aware grasps -> real MTC task planning.

Offline input is a grasp_demo capture_context.json with exported curved centreline.
This command cannot move hardware. --execute fails before reading robot state.
"""
import argparse
import io
import json
from pathlib import Path
import sys
import numpy as np
import yaml
from grasp_candidates import generate
from grasp_scoring import rank
from grasp_geometry import GripperGeometry
from grasp_mtc import require_execution_disabled, validate_place

HERE = Path(__file__).resolve().parents[1]
SRC = HERE.parents[2]


def write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=HERE/'config/multigrasp.yaml')
    parser.add_argument('--arm-config', type=Path, default=HERE/'config/grasp.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidates-only', action='store_true')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--hold-seconds', type=float, default=5)
    args = parser.parse_args([a for a in sys.argv[1:] if ':=' not in a])
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        require_execution_disabled(args.execute)
        cfg = yaml.safe_load(args.config.read_text())
        arm = yaml.safe_load(args.arm_config.read_text())
        context = json.loads(args.context.read_text())
        visual = json.loads(Path(arm['visual_config']).read_text())
        mount = yaml.safe_load(Path(arm['mount_file']).read_text())['launch_defaults']
        gripper = GripperGeometry(SRC/'rml63_tools_description/upstream/EG2-4C2.urdf',
            SRC/'rml63_tools_description/meshes/eg2_4c2', mount['mount_xyz'], mount['mount_rpy'])
        candidates, rejected = generate(context['visual_result'], np.asarray(context['base_depth']), visual,
            cfg, gripper.at_width(visual['width_m'])['pad_bounds_tcp_m'])
        # Include all native observed non-target points. They are a partial scene;
        # no unobserved region is claimed to be obstacle-free.
        obstacles = None
        if 'observed_obstacle_points_base_m' in context:
            obstacles = np.asarray(context['observed_obstacle_points_base_m'], float).reshape(-1, 3)
        scores = rank(candidates, cfg, gripper, obstacles)
        write(args.output/'candidates.json', dict(candidates=[c.to_dict() for c in candidates],
            scores=scores, rejected=rejected, obstacle_evidence_available=obstacles is not None))
        if args.candidates_only:
            write(args.output/'status.json', dict(success=True, stage='candidates_only', hardware_motion_sent=False,
                full_task_verified=False))
            return 0
        validate_place(cfg)
        if obstacles is None:
            raise ValueError('MTC requires observed_obstacle_points_base_m in capture context; use capture_multigrasp.py')
        import rospy
        import moveit_commander
        from moveit_msgs.msg import RobotState, CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from grasp_ros import pose_msg
        from grasp_geometry import transform, collision_samples
        from mtc_model import load_planning_model
        from grasp_mtc import plan_ranked
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('multigrasp_demo')
        load_planning_model(arm)
        from visualization_msgs.msg import Marker, MarkerArray
        pub = rospy.Publisher('candidates', MarkerArray, queue_size=1, latch=True)
        markers = MarkerArray()
        score_by_id = {s['id']: s for s in scores}
        for c in candidates:
            marker = Marker(); marker.header.frame_id = arm['base_frame']
            marker.ns = 'TCP_X_along_rod'; marker.id = c.id
            marker.type = Marker.ARROW; marker.action = Marker.ADD; marker.pose = pose_msg(c.tcp)
            marker.scale.x = .035; marker.scale.y = .003; marker.scale.z = .005
            marker.color.a = 1.; marker.color.g = score_by_id[c.id]['score']
            marker.color.r = 0. if score_by_id[c.id]['accepted'] else 1.
            markers.markers.append(marker)
        pub.publish(markers)
        state = RobotState(); state.joint_state.name = ['joint%d' % i for i in range(1, 7)]+['4C2_Joint1']
        q = context.get('ros_capture', {}).get('joint_evidence', {}).get('measured_joints_mean_rad', context.get('q_after'))
        if q is None or np.asarray(q).shape != (6,) or not np.isfinite(q).all():
            raise ValueError('invalid recorded joint state')
        state.joint_state.position = list(q)+[gripper.at_width(cfg['open_width_m'])['linkage_angle_rad']]
        world = []
        # Basket itself must come from configured verified geometry. Exclude its
        # observed points only inside configured basket boxes, retaining all others.
        base_basket = np.asarray(cfg['basket_pose_base'])
        inside = np.zeros(len(obstacles), dtype=bool)
        for box in cfg['basket_boxes']:
            frame = base_basket @ np.asarray(box['pose_basket'])
            local = (obstacles-frame[:3, 3]) @ frame[:3, :3]
            inside |= np.all(abs(local) <= np.asarray(box['size_m'])/2, axis=1)
        voxel = arm['scene_voxel_m']; points = obstacles[~inside]
        obj = CollisionObject(); obj.id = 'multigrasp_observed_world'; obj.header.frame_id = arm['base_frame']
        centers, exact = collision_samples(points, np.asarray(context['base_grasp'])[:3, 3], voxel,
                                           arm['scene_grasp_detail_radius_m'])
        for p in centers:
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[voxel+2*cfg['scene_margin_m']]*3))
            obj.primitive_poses.append(pose_msg(transform(p, np.eye(3))))
        for p in exact:
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[cfg['scene_margin_m']]))
            obj.primitive_poses.append(pose_msg(transform(p, np.eye(3))))
        if obj.primitives: world.append(obj)
        feasible, failures, tasks = plan_ranked(candidates, scores, cfg, arm, gripper, state, world)
        write(args.output/'planning.json', dict(feasible=[{k: p[k] for k in ('id', 'cost', 'terms')} for p in feasible],
            failures=failures, start_state_source='recorded capture; not live CurrentState',
            calibrated_hardware_model_verified=False))
        if not feasible: raise RuntimeError('no complete MTC task; see planning.json')
        best = feasible[0]; best['task'].publish(best['solution'])
        buf = io.BytesIO(); best['message'].serialize(buf)
        (args.output/'solution.rosmsg').write_bytes(buf.getvalue())
        write(args.output/'status.json', dict(success=True, stage='MTC_plan_only', selected_id=best['id'],
            hardware_motion_sent=False, real_execution_enabled=False))
        rospy.sleep(args.hold_seconds)
        return 0
    except Exception as error:
        write(args.output/'status.json', dict(success=False, error=str(error), hardware_motion_sent=False))
        print(str(error), file=sys.stderr)
        return 2

if __name__ == '__main__': sys.exit(main())
