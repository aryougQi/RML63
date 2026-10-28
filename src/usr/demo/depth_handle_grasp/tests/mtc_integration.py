#!/usr/bin/env python3
"""Isolated, synthetic-scene MTC integration. No driver, camera or controllers."""
from pathlib import Path
import sys
import json
import yaml
import numpy as np
import rospy
import moveit_commander
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE/'scripts'))
from mtc_model import load_planning_model
from grasp_geometry import GripperGeometry, offset_target
from grasp_candidates import Candidate
from grasp_mtc import build_task, plan_ranked
from moveit_msgs.msg import RobotState


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mtc_integration')
    cfg = yaml.safe_load((HERE/'config/multigrasp.yaml').read_text())
    arm = yaml.safe_load((HERE/'config/grasp.yaml').read_text())
    xml = load_planning_model(arm)
    src = HERE.parents[2]
    mount = yaml.safe_load(Path(arm['mount_file']).read_text())['launch_defaults']
    g = GripperGeometry(src/'rml63_tools_description/upstream/EG2-4C2.urdf',
        src/'rml63_tools_description/meshes/eg2_4c2', mount['mount_xyz'], mount['mount_rpy'])
    sys.path.insert(0, str(src/'usr/calibration/rml63_handeye/scripts'))
    from joint_calibration_model import RobotChain
    chain = RobotChain(xml)
    # Existing measured arm posture; this test deliberately uses NOMINAL FK,
    # no claim that nominal MTC trajectories match the calibrated real robot.
    q = [.15082, .25714, -1.4884, -.10093, -1.92051, -.25692]
    base_tcp = chain.fk(q) @ g.at_width(.016)['flange_tcp']
    print('nominal test TCP:', base_tcp, flush=True)
    # Test grasp is below this collision-free initial pose along TCP approach.
    target = base_tcp.copy(); target[:3, 3] += target[:3, 2]*.02
    c = Candidate(0, target, .1, .08, .016, .007, -target[:3, 2], target[:3, 0], .0005, {})
    cfg['basket_geometry_verified'] = cfg['basket_relation_verified'] = True
    basket = np.eye(4); basket[:3, 3] = target[:3, 3]
    cfg['basket_pose_base'] = basket.tolist()
    place = basket.copy(); place[1, 3] += .015
    cfg['place_pose_base'] = place.tolist()
    # Small synthetic basket surrogate hangs below the TCP. This is NOT a
    # validated collision model of the user's basket.
    shape = np.eye(4); shape[2, 3] = -.10
    cfg['basket_boxes'] = [dict(size_m=[.03, .01, .025], pose_basket=shape.tolist())]
    cfg['place_approach_m'] = .015; cfg['retreat_m'] = .02
    cfg['planning_top_k'] = 2
    arm['pregrasp_distance_m'] = .02; arm['lift_distance_m'] = .02
    initial = RobotState(); initial.is_diff = False
    initial.joint_state.name = ['joint%d' % i for i in range(1, 7)]+['4C2_Joint1']
    initial.joint_state.position = q+[g.at_width(.045)['linkage_angle_rad']]
    impossible = Candidate(1, target.copy(), .1, .08, .016, .007, c.normal, c.tangent, .0005, {})
    impossible.tcp[0, 3] = 2.
    scores = [dict(id=1, accepted=True, score=.99, clearance_m=None),
              dict(id=0, accepted=True, score=.9, clearance_m=None)]
    feasible, failed, tasks = plan_ranked([c, impossible], scores, cfg, arm, g, initial)
    result = dict(synthetic_scene=True, hardware_motion_sent=False, feasible=[
        {k: x[k] for k in ('id', 'cost', 'terms')} for x in feasible], failures=failed,
        real_basket_geometry_verified=False)
    if feasible:
        msg = feasible[0]['message']
        assert len(msg.sub_trajectory) == 14
        assert msg.sub_trajectory[6].scene_diff.robot_state.attached_collision_objects[0].link_name == 'link6'
        attached = msg.sub_trajectory[6].scene_diff.robot_state.attached_collision_objects[0]
        approach = msg.sub_trajectory[4].trajectory.joint_trajectory
        endpoint = dict(zip(approach.joint_names, approach.points[-1].positions))
        flange = chain.fk([endpoint['joint%d' % i] for i in range(1, 7)])
        expected = np.linalg.inv(flange) @ basket @ shape
        from tf.transformations import quaternion_matrix
        def pose_matrix(pose):
            matrix = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            return matrix
        actual = pose_matrix(attached.object.primitive_poses[0])
        if hasattr(attached.object, 'pose'):
            actual = pose_matrix(attached.object.pose) @ actual
        np.testing.assert_allclose(actual, expected, atol=1e-5)
        result['attachment_relative_transform_verified'] = True
        for index in (7, 8, 9, 10):
            assert any(a.object.id == 'depth_grasp_basket' and a.object.operation == 0
                       for a in msg.sub_trajectory[index].scene_diff.robot_state.attached_collision_objects)
        detached = msg.sub_trajectory[12].scene_diff
        assert detached.robot_state.attached_collision_objects[0].object.operation == 1
        assert any(o.id == 'depth_grasp_basket' and o.operation == 0 for o in detached.world.collision_objects)
        assert not msg.sub_trajectory[13].scene_diff.robot_state.attached_collision_objects
        # Actual nominal FK along transport, independent of graph construction.
        from grasp_geometry import flange_target
        relative = np.linalg.inv(flange_target(target, g.at_width(.016)['flange_tcp'])) @ basket
        maximum_tilt = 0.
        for index in (7, 8, 9, 10):
            trajectory = msg.sub_trajectory[index].trajectory.joint_trajectory
            for point in trajectory.points:
                values = dict(zip(trajectory.joint_names, point.positions))
                rotation = (chain.fk([values['joint%d' % i] for i in range(1, 7)]) @ relative)[:3, :3]
                tilt = float(np.arccos(np.clip(rotation[2, 2], -1, 1)))
                maximum_tilt = max(maximum_tilt, tilt)
        assert maximum_tilt < cfg['upright_tolerance_rad']+.01
        result['attachment_persists_and_detach_restores_world'] = True
        result['maximum_basket_tilt_rad'] = maximum_tilt
        result['sub_trajectory_count'] = len(msg.sub_trajectory)
        result['stages'] = [s.name for s in feasible[0]['task']]
        import io
        buf = io.BytesIO(); msg.serialize(buf)
        (HERE/'reports/multigrasp_validation/solution.rosmsg').write_bytes(buf.getvalue())
        feasible[0]['task'].publish(feasible[0]['solution'])
        # A separate scene places an obstacle in the carried basket's lift path,
        # below the gripper. The complete task must be rejected.
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from grasp_ros import pose_msg
        blocker = CollisionObject(); blocker.id = 'lift_blocker'; blocker.header.frame_id = 'base_link'
        blocker.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.012, .012, .004])]
        obstacle_pose = basket @ shape
        obstacle_pose[2, 3] += .02
        blocker.primitive_poses = [pose_msg(obstacle_pose)]
        blocked, blocked_failures, blocked_tasks = plan_ranked([c], [scores[1]], cfg, arm, g, initial, [blocker])
        assert not blocked, 'basket collision should reject the complete task'
        result['carried_basket_collision_rejected'] = True
        result['collision_failures'] = blocked_failures
    (HERE/'reports/multigrasp_validation/mtc_integration.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    return 0 if feasible and failed else 1

if __name__ == '__main__': sys.exit(main())
