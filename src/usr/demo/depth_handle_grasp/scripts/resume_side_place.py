#!/usr/bin/env python3
"""Explicit continuation after the known stale-open closure-feedback failure.

Default validates only. Never opens/recloses at the grasp pose. Requires the
operator's record that this run holds the handle and the basket is supported.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from grasp_demo import HERE, SRC, write
from grasp_geometry import GripperGeometry
from handle_pipeline import side_place_targets
from grasp_ros import RobotSession, GripperIO, state_after


def load_trajectory(saved):
    trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=saved['joint_names']
    for p in saved['points']:
        point=JointTrajectoryPoint(positions=p['positions'],velocities=p.get('velocities',[]),
            accelerations=p.get('accelerations',[]),time_from_start=rospy.Duration.from_sec(p['time_sec']))
        trajectory.joint_trajectory.points.append(point)
    return trajectory


def validate_record(status,feedback,confirmation,plans):
    if status.get('error')!='gripper fully closed without detecting an object':
        raise RuntimeError('resume supports only the recorded closure-feedback failure')
    if [f['stage'] for f in feedback]!=['pregrasp','descent']:
        raise RuntimeError('run did not stop immediately after descent; no resume')
    if not confirmation.get('gripped_handle_confirmed') or not confirmation.get('basket_supported'):
        raise RuntimeError('operator must confirm this run holds the handle and basket remains supported')
    if set(plans)!={'pregrasp','descent','lift','transfer','place','retreat'}:
        raise RuntimeError('complete saved side-place trajectories unavailable')


def read_contact(hardware,config):
    samples=[]
    for _ in range(3):
        state=hardware.query('get_gripper_state');samples.append(state)
        if (state.get('error',0) or state.get('mode')!=6 or state.get('status')!=1
                or state.get('enable')!=1 or state.get('actpos',0)<=30):
            raise RuntimeError('held contact is not verified; no motion or automatic release')
        if abs(state['actpos']*.070/1000-config['contact_width_m'])>.007:
            raise RuntimeError('held opening does not match the handle')
        rospy.sleep(.15)
    if max(s['actpos'] for s in samples)-min(s['actpos'] for s in samples)>=15:
        raise RuntimeError('held contact opening is not stable')
    return samples[-1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=HERE/'config/grasp.yaml')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    report=args.run/('resume_'+datetime.now().strftime('%Y%m%d_%H%M%S'));report.mkdir(parents=True,exist_ok=False)
    session=None;motion_sent=False
    try:
        config=yaml.safe_load(args.config.read_text())
        if args.execute and not config.get('side_place_scene_verified',False):
            raise RuntimeError('selected execution config must have side_place_scene_verified: true')
        if (args.run/'resume_completed.json').exists():raise RuntimeError('this run already completed its continuation')
        plans=json.loads((args.run/'planned_trajectories.json').read_text())
        validate_record(json.loads((args.run/'status.json').read_text()),
            json.loads((args.run/'execution_feedback.json').read_text()),
            json.loads((args.run/'operator_contact_confirmation.json').read_text()),plans)
        context=json.loads((args.run/'capture_context.json').read_text())
        selected=json.loads((args.run/'selected_targets.json').read_text())
        expected={k:np.asarray(v,float) for k,v in json.loads((args.run/'side_place_targets.json').read_text()).items()}
        moveit_commander.roscpp_initialize(sys.argv);rospy.init_node('resume_side_place')
        session=RobotSession(config);session.verify_hardware(require_execution=args.execute)
        if session.calibration_hash!=context['calibration_sha256']:raise RuntimeError('calibration changed since the saved run')
        tcp=np.asarray(selected['flange_tcp']);grasp=np.asarray(selected['base_grasp'])
        computed=side_place_targets(grasp,config)
        for name in expected:
            if not np.allclose(expected[name],computed[name],atol=1e-8):
                raise RuntimeError('side-place config differs from saved targets; no resume')
        q,_=session.measured();rospy.sleep(.2);q2,_=session.measured()
        if np.max(abs(q-q2))>np.radians(.1):raise RuntimeError('robot is still moving')
        start=load_trajectory(plans['lift'])
        if start.joint_trajectory.joint_names!=session.active:raise RuntimeError('joint order changed')
        if np.max(abs(q2-np.asarray(start.joint_trajectory.points[0].positions)))>np.radians(.5):
            raise RuntimeError('robot no longer matches the saved descent endpoint')
        hardware=GripperIO(config);contact=read_contact(hardware,config)
        write(report/'contact_before_validation.json',contact)
        mount=yaml.safe_load(Path(config['mount_file']).read_text())['launch_defaults']
        gripper=GripperGeometry(SRC/'rml63_tools_description/upstream/EG2-4C2.urdf',
            SRC/'rml63_tools_description/meshes/eg2_4c2',mount['mount_xyz'],mount['mount_rpy'])
        if not np.allclose(gripper.at_width(config['contact_width_m'])['flange_tcp'],tcp,atol=1e-8):
            raise RuntimeError('TCP/mount differs from saved run')
        import cv2
        raw=sorted(Path(context['raw_directory']).glob('frame_*'))[-1]
        depth=np.load(str(raw/'depth_raw.npy'),allow_pickle=False)
        mask=cv2.imread(str(Path(context['vision_report_directory'])/'handle_mask.png'),0)
        if mask is None:raise RuntimeError('saved target mask missing')
        session.base_depth=np.asarray(context['base_depth']);session.target=grasp
        capture_q=context['ros_capture']['joint_evidence']['measured_joints_mean_rad']
        scene=session.add_scene(session.base_depth,depth,context['metadata'],mask,
            context['visual_result']['support_plane'],gripper,gripper.at_width(config['open_width_m'])['linkage_angle_rad'],
            capture_q=capture_q)
        write(report/'scene.json',dict(scene,scene_source='saved capture with original capture joints; operator confirmed basket remains supported'))
        # Validate all remaining trajectories before any action. Rebuild filtering
        # with CAPTURE joints, not the arm's new grasp pose.
        paths={name:load_trajectory(plans[name]) for name in ('lift','transfer','place','retreat')}
        state=session.state_for(q2)
        for name,path in paths.items():
            session.set_tool_width(config['open_width_m'] if name=='retreat' else contact['actpos']*.070/1000)
            session.check_trajectory(path,state);state=state_after(state,path)
            print('Validated remaining stage:',name,flush=True)
        session.set_tool_width(contact['actpos']*.070/1000)
        write(report/'status.json',dict(success=True,stage='remaining_paths_validated',robot_motion_sent=False))
        if not args.execute:
            print('Validation only; no motion or gripper command sent. Report:',report,flush=True)
            return 0
        deadline=time.monotonic()+config['max_execution_sec'];feedback=[]
        for name in ('lift','transfer','place'):
            read_contact(hardware,config)
            if time.monotonic()>deadline:raise RuntimeError('continuation timed out')
            write(report/'status.json',dict(success=False,stage='executing_'+name,robot_motion_sent=True))
            motion_sent=True
            reached=session.execute_one(paths[name],tcp,expected[name])
            feedback.append(dict(stage=name,**reached));write(report/'execution_feedback.json',feedback)
        read_contact(hardware,config)
        if time.monotonic()>deadline:raise RuntimeError('continuation timed out before release')
        # Verified place arrival precedes the only gripper action in this tool.
        write(report/'release.json',hardware.open(config['open_width_m']))
        session.set_tool_width(config['open_width_m'])
        if time.monotonic()>deadline:raise RuntimeError('continuation timed out before retreat')
        reached=session.execute_one(paths['retreat'],tcp,expected['retreat'])
        feedback.append(dict(stage='retreat',**reached));write(report/'execution_feedback.json',feedback)
        result=dict(success=True,stage='placed_aside_and_retreated',robot_motion_sent=True,report=str(report))
        write(report/'status.json',result);write(args.run/'resume_completed.json',result)
        return 0
    except Exception as error:
        if session:session.stop()
        write(report/'status.json',dict(success=False,stage='stopped',error=str(error),robot_motion_sent=motion_sent,
            note='No automatic retry, release or recovery'))
        print('STOP:',error,'report:',report,file=sys.stderr,flush=True);return 2

if __name__=='__main__':sys.exit(main())
