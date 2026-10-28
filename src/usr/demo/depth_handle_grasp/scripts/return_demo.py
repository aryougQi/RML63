#!/usr/bin/env python3
"""Return only from the verified lifted endpoint of one completed grasp run."""
import argparse
import json
from datetime import datetime
from pathlib import Path
import sys
import numpy as np
import yaml
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from trajectory_msgs.msg import JointTrajectoryPoint
from grasp_ros import RobotSession, GripperIO, state_after
from grasp_demo import HERE, write, serializable_trajectory


def reverse(session, saved, start):
    trajectory=RobotTrajectory()
    trajectory.joint_trajectory.joint_names=list(saved['joint_names'])
    positions=[p['positions'] for p in reversed(saved['points'])]
    if trajectory.joint_trajectory.joint_names!=session.active:
        raise RuntimeError('saved joint order mismatch')
    q0=dict(zip(start.joint_state.name,start.joint_state.position))
    actual=np.array([q0[n] for n in session.active])
    if np.max(abs(actual-np.array(positions[0])))>np.radians(.5):
        raise RuntimeError('return path start differs from recorded endpoint')
    positions[0]=actual.tolist()
    for q in positions:
        point=JointTrajectoryPoint();point.positions=q
        trajectory.joint_trajectory.points.append(point)
    trajectory=session.dense_timed(trajectory,start)
    session.check_trajectory(trajectory,start)
    return trajectory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    report=args.run/('return_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    report.mkdir(parents=True)
    config=yaml.safe_load((HERE/'config/grasp.yaml').read_text())
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('depth_handle_return')
    session=None
    try:
        session=RobotSession(config);session.verify_hardware()
        context=json.loads((args.run/'capture_context.json').read_text())
        if context['calibration_sha256']!=session.calibration_hash:
            raise RuntimeError('calibration differs from original grasp')
        status=json.loads((args.run/'status.json').read_text())
        if status.get('stage') not in ('grasp_and_small_lift_operator_confirmed','lifted_10mm_contact_retained'):
            raise RuntimeError('run is not at a completed small-lift stage')
        scene_service=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)
        request=GetPlanningSceneRequest()
        request.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene=scene_service(request).scene
        if not {'depth_grasp_table','depth_grasp_basket'}.issubset({o.id for o in scene.world.collision_objects}):
            raise RuntimeError('original obstacle scene missing')
        if 'depth_grasp_open_tool' not in {a.object.id for a in scene.robot_state.attached_collision_objects}:
            raise RuntimeError('original open tool collision model missing')
        plans=json.loads((args.run/'planned_trajectories.json').read_text())
        targets=json.loads((args.run/'selected_targets.json').read_text())
        tcp=np.array(targets['flange_tcp'])
        q,_=session.measured();rospy.sleep(.3);q2,_=session.measured()
        if np.max(abs(q-q2))>np.radians(.1):raise RuntimeError('robot is still moving')
        hardware=GripperIO(config)
        grip=hardware.query('get_gripper_state');write(report/'gripper_initial.json',grip)
        if grip.get('error') or grip.get('mode')!=6 or grip.get('status')!=1 or grip.get('enable')!=1:
            raise RuntimeError('held basket contact not verified; no automatic return')
        start=session.state_for(q2)
        paths=[]
        for name in ('lift','descent','pregrasp'):
            path=reverse(session,plans[name],start)
            paths.append(path);start=state_after(start,path)
        write(report/'planned_trajectories.json',{n:serializable_trajectory(p) for n,p in zip(('replace_basket','retreat','return_origin'),paths)})
        write(report/'status.json',dict(success=True,stage='return_planned',robot_motion_sent=False))
        print('All reverse paths checked. execute=%s; report=%s'%(args.execute,report),flush=True)
        if not args.execute:return 0
        feedback=[]
        def move(name,path,expected):
            write(report/'status.json',dict(success=False,stage=name,robot_motion_sent=True))
            result=session.execute_one(path,tcp,expected)
            feedback.append(dict(stage=name,**result));write(report/'execution_feedback.json',feedback)
        # Restore the precise pre-lift pose while maintaining grip, then release on table.
        move('replace_basket',paths[0],np.array(targets['base_grasp']))
        rospy.sleep(.5)
        write(report/'gripper_released.json',hardware.open(config['open_width_m']))
        move('retreat',paths[1],np.array(targets['base_pregrasp']))
        origin=np.array(plans['pregrasp']['points'][0]['positions'])
        move('return_origin',paths[2],session.chain.fk(origin,session.offsets)@tcp)
        actual,_=session.measured()
        error=float(np.degrees(np.max(abs(actual-origin))))
        if error>.5:raise RuntimeError('original joint pose not reached')
        write(report/'status.json',dict(success=True,stage='returned_to_original_pose',robot_motion_sent=True,
            basket_replaced_before_release=True,gripper_open_width_m=config['open_width_m'],
            max_original_joint_error_deg=error,measured_joints_rad=actual.tolist()))
        write(args.run/'return_result.json',dict(report_directory=str(report),success=True,stage='returned_to_original_pose'))
        print('Returned to original pose; basket replaced, gripper released. Max joint error %.4f degrees'%error,flush=True)
        return 0
    except Exception as error:
        if session:session.stop()
        write(report/'status.json',dict(success=False,stage='stopped',error=str(error)))
        print('STOP:',error,'report:',report,flush=True)
        return 2

if __name__=='__main__':sys.exit(main())
