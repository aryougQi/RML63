#!/usr/bin/env python3
"""Fresh L515 capture -> calibrated base pose -> preview/plan -> optional real grasp.

Default is plan-only. No arm commands are sent without --execute. All controller
motion uses the existing MoveIt/rm_control stack; raw socket commands are queries.
"""
import argparse
from copy import deepcopy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml
import rospy
import moveit_commander
from tf.transformations import quaternion_from_matrix

HERE=Path(__file__).resolve().parents[1]
SRC=HERE.parents[2]
VISION=SRC/'usr/depth_handle_detect'
sys.path.insert(0,str(HERE/'scripts'))
from grasp_geometry import GripperGeometry,optical_to_base,matrix_from_pose,flange_target,offset_target,inward_grasp,raised_pad_grasp
from grasp_ros import RobotSession,GripperIO,state_after
from handle_pipeline import prepare_candidates,path_cost,plan_side_place,side_place_targets,precheck_side_place,coarse_precheck_task
from planner_metrics import PlannerMetrics,PlanningTimeout,JointConstraintFailure,NoFeasiblePlan,failure_kind


def write(path,value):
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def serializable_trajectory(trajectory):
    return dict(joint_names=list(trajectory.joint_trajectory.joint_names),points=[dict(
        positions=list(p.positions),velocities=list(p.velocities),accelerations=list(p.accelerations),
        time_sec=p.time_from_start.to_sec()) for p in trajectory.joint_trajectory.points])


def acquire(session,config,report,run):
    q_before,stamp_before=session.measured()
    raw=VISION/'data'/'robot_grasp'/run
    visual=VISION/'outputs'/'robot_grasp'/run
    from ros_depth import capture_ros
    ros_capture=capture_ros(raw,config,count=30,joint_names=session.active)
    q_after,stamp_after=session.measured()
    if np.max(abs(q_after-q_before))>np.radians(config['max_capture_motion_deg']):
        raise RuntimeError('robot moved while acquiring vision; cannot bind camera pose to measured joints')
    write(report/'visual_config.json',json.loads(Path(config['visual_config']).read_text()))
    process=subprocess.run([str(VISION/'.venv/bin/python'),str(VISION/'run.py'),'stability','--input',str(raw),
                           '--config',config['visual_config'],'--output',str(visual)],timeout=30)
    details=VISION/'reports'/'runs'/'robot_grasp'/run
    if not (details/'stability_summary.json').exists():
        process.check_returncode()
        raise RuntimeError('vision did not produce a stability report: '+str(details))
    summary=json.loads((details/'stability_summary.json').read_text())
    result=json.loads((details/'handle_pose.json').read_text())
    if process.returncode or not summary['grasp_pose_stability_pass'] or not summary['sequence_contiguous']:
        write(report/'visual_failure.json',dict(raw_directory=str(raw),vision_report_directory=str(details),
              summary=summary,last_frame=result,exit_code=process.returncode))
        reason=result.get('reason','unknown')
        diagnostics='; '.join(result.get('diagnostics',[]))
        raise RuntimeError('vision rejected: detection %s/%s, grasp poses %s/%s; last frame: %s%s; details: %s'%(
            summary['successful_frames'],summary['frame_count'],summary['grasp_pose_ready_frames'],
            summary['frame_count'],reason,('; '+diagnostics) if diagnostics else '',details))
    if not result['success'] or result['pose'] is None:
        raise RuntimeError('no reliable fresh visual pose')
    paths=sorted(raw.glob('frame_*'));last=paths[-1]
    meta=json.loads((last/'metadata.json').read_text())
    depth=np.load(str(last/'depth_raw.npy'),allow_pickle=False)
    import cv2
    mask=cv2.imread(str(details/'handle_mask.png'),cv2.IMREAD_GRAYSCALE)
    if mask is None:raise RuntimeError('native depth handle mask missing')
    if meta.get('serial')!=config['camera_serial'] or not meta.get('depth_to_color'):
        raise RuntimeError('camera serial/factory depth-to-color calibration missing')
    q=np.array(ros_capture['joint_evidence']['measured_joints_mean_rad'])
    stamp_depth=rospy.Time(ros_capture['last_depth_stamp_ns']//10**9,ros_capture['last_depth_stamp_ns']%10**9)
    base_depth=optical_to_base(session.chain.fk(q,session.offsets),session.handeye,meta['depth_to_color'])
    camera_pose=matrix_from_pose(result['pose']);base_pose=base_depth@camera_pose
    record=dict(raw_directory=str(raw),vision_report_directory=str(details),
        q_before=q_before.tolist(),q_after=q_after.tolist(),capture_stamp_before=stamp_before.to_sec(),
        capture_stamp_after=stamp_after.to_sec(),base_depth=base_depth.tolist(),
        camera_pose=camera_pose.tolist(),base_grasp=base_pose.tolist(),visual_result=result,
        vision_stability=summary,metadata=meta,calibration_sha256=session.calibration_hash,
        ros_capture=ros_capture,
        pose_time_basis='continuous stationary joint evidence bracketing RealSense ROS header timestamps; no hardware synchronization claimed')
    write(report/'capture_context.json',record)
    return record,q,stamp_depth,depth,meta,mask


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=HERE/'config/grasp.yaml')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--publish-only',action='store_true')
    parser.add_argument('--lift-only',action='store_true',help='retain the original small-lift workflow, without side placement')
    parser.add_argument('--hold-seconds',type=float,default=5.)
    args=parser.parse_args()
    config=yaml.safe_load(args.config.read_text())
    place_aside=config.get('place_aside',True) and not args.lift_only
    planning_mode=config.get('planning_mode','first_feasible')
    if planning_mode not in ('first_feasible','best_within_budget'):
        raise ValueError('planning_mode must be first_feasible or best_within_budget')
    run=datetime.now().strftime('%Y%m%d_%H%M%S')
    report=HERE/'reports'/run;report.mkdir(parents=True,exist_ok=False)
    write(HERE/'reports/latest_run.json',dict(run_directory=str(report),execute_requested=args.execute))
    session=None
    metrics=PlannerMetrics()
    planning_complete=False
    try:
        if args.execute and not args.publish_only and place_aside and not config.get('side_place_scene_verified',False):
            raise RuntimeError('side-place execution requires side_place_scene_verified: true in the selected --config after checking carried-object and destination clearance; configuration rejected before capture/planning')
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('depth_handle_grasp',anonymous=False)
        session=RobotSession(config);session.verify_hardware(require_execution=args.execute)
        session.metrics=metrics
        mount=yaml.safe_load(Path(config['mount_file']).read_text())['launch_defaults']
        gripper=GripperGeometry(SRC/'rml63_tools_description/upstream/EG2-4C2.urdf',
            SRC/'rml63_tools_description/meshes/eg2_4c2',mount['mount_xyz'],mount['mount_rpy'])
        contact=gripper.at_width(config['contact_width_m']);opened=gripper.at_width(config['open_width_m'])
        tcp=contact['flange_tcp']
        capture,q,stamp,depth,meta,mask=acquire(session,config,report,run)
        metrics.begin_task()
        base_depth=np.array(capture['base_depth']);target=np.array(capture['base_grasp'])
        session.base_depth=base_depth
        camera=np.array(capture['camera_pose']);session.target=target
        pregrasp=offset_target(target,target[:3,2],config['pregrasp_distance_m'])
        lift=offset_target(target,target[:3,2],config['lift_distance_m'])
        current=session.chain.fk(q,session.offsets)@tcp
        if np.linalg.norm(target[:3,3]-current[:3,3])>config['max_target_distance_m']:
            raise RuntimeError('target exceeds allowed first-test travel')
        if target[2,2]>-.95:
            raise RuntimeError('grasp approach is not downward in the calibrated base frame')
        session.publish(camera,target,pregrasp,stamp)
        write(report/'targets.json',dict(base_grasp=target.tolist(),base_pregrasp=pregrasp.tolist(),
             base_lift=lift.tolist(),flange_tcp=tcp.tolist(),tcp_source=contact['source'],
             tool_open_width_m=opened['opening_m'],grasp_center_estimate_uses_approximate_thickness=True,
             calibration_position_rms_mm=session.cal['provenance']['independent_validation']['position_rms_mm']))
        print('Published camera/base grasp poses; no motion sent.',flush=True)
        visual_cfg=json.loads(Path(config['visual_config']).read_text())
        print('Generating and scoring handle candidates...',flush=True)
        with metrics.measure('candidate_geometry_scoring'):
            handle_candidates,handle_report=prepare_candidates(capture,visual_cfg,config,gripper,contact['pad_bounds_tcp_m'])
        write(report/'handle_candidates.json',handle_report)
        if not handle_candidates:
            raise RuntimeError('no feasible handle candidates; see handle_candidates.json')
        if args.publish_only:
            write(report/'status.json',dict(success=True,stage='published_only',robot_motion_sent=False))
            rospy.sleep(args.hold_seconds);return 0
        print('Building observed collision scene and CAD tool geometry...',flush=True)
        scene=session.add_scene(base_depth,depth,meta,mask,capture['visual_result']['support_plane'],gripper,opened['linkage_angle_rad'])
        write(report/'scene.json',scene)
        session.check_state(q)
        initial=session.state_for(q)
        print('Scene ready. Planning %d handle positions.'%len(handle_candidates),flush=True)
        total_budget=float(config.get('planning_total_timeout_sec',20.))
        position_budget=float(config.get('planning_candidate_timeout_sec',6.))
        max_attempts=int(config.get('planning_max_approach_attempts',4))
        if not np.isfinite([total_budget,position_budget]).all() or min(total_budget,position_budget)<=0 or max_attempts<1:
            raise ValueError('invalid planning time/attempt limits')
        planning_started=time.monotonic();total_deadline=planning_started+total_budget
        metrics.search_started=planning_started
        position_deadlines={};attempts={}
        def progress(stage,candidate_id=None):
            metrics.current_stage=stage
            record=dict(stage=stage,candidate_id=candidate_id,elapsed_sec=round(time.monotonic()-planning_started,2),robot_motion_sent=False)
            write(report/'planning_progress.json',record)
            print('[planning %.1fs] candidate=%s stage=%s'%(record['elapsed_sec'],candidate_id,stage),flush=True)
        measurement=capture['visual_result']['measurement']
        normal=base_depth[:3,:3]@np.asarray(measurement['outward_normal'])
        rod_axis=base_depth[:3,:3]@np.asarray(measurement['axis_direction'])
        visual_cfg=json.loads(Path(config['visual_config']).read_text())
        tilts=config.get('inward_grasp_tilt_candidates_deg',[0])
        if not tilts or any(not np.isfinite(v) or not 0<=v<=15 for v in tilts):
            raise RuntimeError('invalid inward grasp tilt candidates (allowed 0..15 degrees)')
        accepted=[];rejected=[];selected=None;complete=[];planned_positions=set()
        baseline_clearance=session.minimum_observed_clearance_m
        angles=sorted(set([0]+list(tilts)))
        azimuths=config.get('grasp_approach_azimuth_candidates_deg',[0])
        if any(not np.isfinite(v) or abs(v)>90 for v in azimuths):
            raise RuntimeError('invalid approach azimuth candidates')
        primary=[v for v in angles if v!=0]+[0] if config.get('prefer_inward_grasp',False) else angles
        approaches=[(v,0) for v in primary]+[(v,a) for v in angles if v!=0 for a in azimuths if a!=0]
        heights=config.get('grasp_pad_raise_candidates_m',[0])
        if any(not np.isfinite(v) or not 0<=v<=.01 for v in heights):
            raise RuntimeError('invalid pad contact height candidates')
        approaches=[(t,a,h) for h in sorted(set([0]+list(heights))) for t,a in approaches]
        approaches=[(c,t,a,h) for c in handle_candidates for t,a,h in approaches]
        for handle_candidate,tilt,azimuth,pad_raise in approaches:
            if handle_candidate.id in planned_positions:continue
            now=time.monotonic()
            if now>=total_deadline:
                progress('total_time_budget_reached')
                break
            cid=handle_candidate.id
            if cid not in position_deadlines:position_deadlines[cid]=min(total_deadline,now+position_budget)
            if now>=position_deadlines[cid] or attempts.get(cid,0)>=max_attempts:continue
            attempts[cid]=attempts.get(cid,0)+1
            session.planning_deadline=position_deadlines[cid]
            progress('approach_%d tilt=%g azimuth=%g pad_raise=%g'%(attempts[cid],tilt,azimuth,pad_raise),cid)
            local_target=handle_candidate.tcp
            normal=handle_candidate.normal
            rod_axis=handle_candidate.tangent
            if np.linalg.norm(local_target[:3,3]-current[:3,3])>config['max_target_distance_m']:
                metrics.failures['target_distance']+=1
                rejected.append(dict(candidate_id=handle_candidate.id,stage='target_distance',reason='candidate exceeds allowed first-test travel'))
                continue
            session.set_tool_width(config['open_width_m'])
            try:
                tilted=inward_grasp(local_target,normal,rod_axis,tilt,
                    visual_cfg.get('max_face_approach_angle_deg',25.),
                    visual_cfg.get('max_axis_approach_dot',.35),azimuth_deg=azimuth)
                tilted=raised_pad_grasp(tilted,local_target[:3,3],normal,visual_cfg['thickness_m'],
                    contact['pad_bounds_tcp_m'],pad_raise)
                if tilted[2,2]>-.95:
                    raise RuntimeError('tilted approach exceeds downward-direction limit')
            except (ValueError,RuntimeError) as error:
                metrics.failures['geometry']+=1
                rejected.append(dict(candidate_id=handle_candidate.id,inward_tilt_deg=tilt,azimuth_from_inward_deg=azimuth,pad_raise_m=pad_raise,stage='geometry',reason=str(error)))
                continue
            candidates=[]
            endpoint_started=time.monotonic()
            for flip in (False,True):
                alternative=tilted.copy()
                if flip:alternative[:3,:3]=alternative[:3,:3]@np.diag([-1.,-1.,1.])
                pre=offset_target(alternative,alternative[:3,2],config['pregrasp_distance_m'])
                ik_stage='pregrasp_ik'
                try:
                    solution=session.solve(flange_target(pre,tcp),q)
                    travel=float(np.max(abs(solution-q)))
                    if travel>np.radians(config['max_joint_travel_deg']):
                        error=JointConstraintFailure('pregrasp joint travel %.2f degrees exceeds bound'%np.degrees(travel))
                        error.joint_state_rad=solution.tolist()
                        raise error
                    ik_stage='grasp_endpoint'
                    contact_solution=session.solve(flange_target(alternative,tcp),solution)
                    ik_stage='contact_endpoint'
                    session.set_tool_width(config['contact_width_m'])
                    session.check_state(contact_solution)
                    if place_aside:
                        ik_stage='side_place_endpoint_precheck'
                        progress(ik_stage,cid)
                        precheck_side_place(session,tcp,alternative,contact_solution,config)
                    session.set_tool_width(config['open_width_m'])
                    candidates.append((travel,alternative,pre,solution,flip))
                    accepted.append(dict(candidate_id=handle_candidate.id,inward_tilt_deg=tilt,azimuth_from_inward_deg=azimuth,pad_raise_m=pad_raise,max_joint_travel_deg=float(np.degrees(travel)),
                        reported_joint_target_rad=solution.tolist(),flipped_180=flip))
                except RuntimeError as error:
                    metrics.failures[failure_kind(error)]+=1
                    session.set_tool_width(config['open_width_m'])
                    rejected.append(dict(candidate_id=handle_candidate.id,inward_tilt_deg=tilt,azimuth_from_inward_deg=azimuth,pad_raise_m=pad_raise,flipped=flip,stage=ik_stage,reason=str(error),failure_kind=failure_kind(error),joint_state_rad=getattr(error,'joint_state_rad',None),contacts=getattr(error,'contacts',None),joint_state_basis=getattr(error,'joint_state_basis',None),reported_joint_state_rad=getattr(error,'reported_joint_state_rad',None)))
                    print('  rejected %s flip=%s: %s'%(ik_stage,flip,error),flush=True)
                    write(report/'approach_ik_candidates.json',dict(accepted=accepted,rejected=rejected))
            metrics.samples['endpoint_precheck'].append(time.monotonic()-endpoint_started)
            for travel,alternative,pre,solution,flip in sorted(candidates,key=lambda v:v[0]):
                # Once all endpoints pass, give the complete trajectory the
                # remaining global budget; do not discard it at the cheap IK
                # prefilter's short per-position deadline.
                session.planning_deadline=total_deadline
                session.minimum_observed_clearance_m=baseline_clearance
                stage='pregrasp_plan'
                try:
                    if config.get('coarse_check_enabled',False):
                        stage='coarse_check';progress(stage,cid)
                        coarse_precheck_task(session,tcp,pre,alternative,solution,config,place_aside)
                    stage='pregrasp_plan'
                    progress(stage,cid)
                    first=session.plan_joint(solution,initial)
                    after_first=state_after(initial,first)
                    stage='descent_plan'
                    progress(stage,cid)
                    descent=session.plan_descent(tcp,pre,alternative,after_first)
                    after_descent=state_after(after_first,descent)
                    candidate_lift=offset_target(alternative,alternative[:3,2],config['lift_distance_m'])
                    stage='lift_plan'
                    session.set_tool_width(config['contact_width_m'])
                    if place_aside:
                        stage='lift_transfer_place_retreat_plan'
                        side_targets,side_plans=plan_side_place(session,tcp,alternative,after_descent,config,
                            progress=lambda name:progress(name,cid))
                        candidate_lift=side_targets['lift']
                        candidate_plans=[first,descent]+side_plans
                    else:
                        lift_plan=session.plan_descent(tcp,alternative,candidate_lift,after_descent)
                        candidate_plans=[first,descent,lift_plan]
                    choice=(tilt,azimuth,pad_raise,alternative,pre,candidate_lift,flip,candidate_plans)
                    complete.append((path_cost(choice[-1]),handle_candidate.id,choice,session.minimum_observed_clearance_m))
                    metrics.solution()
                    planned_positions.add(handle_candidate.id)
                    # Checkpoint only complete executable trajectories. Later
                    # failures/timeouts must never overwrite this known solution.
                    best_so_far=min(complete,key=lambda item:(item[0],item[1]))
                    write(report/'complete_solution_checkpoint.json',dict(candidate_id=best_so_far[1],
                        cost=best_so_far[0],planning_mode=planning_mode,
                        plans=[serializable_trajectory(plan) for plan in best_so_far[2][-1]]))
                    progress('complete_path',cid)
                    break
                except RuntimeError as error:
                    metrics.failures[failure_kind(error)]+=1
                    rejected.append(dict(candidate_id=handle_candidate.id,inward_tilt_deg=tilt,azimuth_from_inward_deg=azimuth,pad_raise_m=pad_raise,flipped=flip,stage=stage,reason=str(error),failure_kind=failure_kind(error),joint_state_rad=getattr(error,'joint_state_rad',None),contacts=getattr(error,'contacts',None),joint_state_basis=getattr(error,'joint_state_basis',None),reported_joint_state_rad=getattr(error,'reported_joint_state_rad',None)))
                    session.set_tool_width(config['open_width_m'])
                    print('  rejected %s: %s'%(stage,error),flush=True)
            write(report/'approach_ik_candidates.json',dict(accepted=accepted,rejected=rejected))
            if complete and planning_mode=='first_feasible':break
        session.planning_deadline=None
        progress('select_complete_solution' if complete else 'no_complete_solution')
        budget_exhausted=time.monotonic()>=total_deadline
        termination=('first_feasible' if complete and planning_mode=='first_feasible' else
            'budget_exhausted_with_solution' if complete and budget_exhausted else
            'budget_exhausted_no_solution' if budget_exhausted else 'search_limits_reached')
        write(report/'planning_search.json',dict(attempts=attempts,completed_candidate_ids=sorted(planned_positions),
            planning_mode=planning_mode,
            termination=termination,complete_solution_count=len(complete),
            total_budget_sec=total_budget,candidate_budget_sec=position_budget,max_attempts=max_attempts,
            elapsed_sec=time.monotonic()-planning_started,search_exhaustive=False))
        if complete:
            complete.sort(key=lambda item:(item[0],item[1]))
            _,selected_id,selected,session.minimum_observed_clearance_m=complete[0]
            write(report/'handle_planning.json',dict(planner='existing calibrated RobotSession; not MTC',
                selected_candidate_id=selected_id,complete=[dict(cost=item[0],candidate_id=item[1]) for item in complete]))
        write(report/'approach_ik_candidates.json',dict(accepted=accepted,rejected=rejected))
        if selected is None:
            if time.monotonic()>=total_deadline or any(r.get('failure_kind')=='budget_timeout' for r in rejected):
                raise PlanningTimeout('planning budget exhausted without a complete feasible trajectory')
            raise NoFeasiblePlan('no complete collision-checked grasp path in the configured search set; see approach_ik_candidates.json')
        tilt,azimuth,pad_raise,target,pregrasp,lift,flip,plans=selected
        first,descent,lift_plan=plans[:3]
        camera=np.linalg.inv(base_depth)@target
        session.set_tool_width(config['open_width_m'])
        write(report/'selected_targets.json',dict(base_grasp=target.tolist(),base_pregrasp=pregrasp.tolist(),
              base_lift=lift.tolist(),flange_tcp=tcp.tolist(),flipped_180=flip,
              inward_tilt_deg=tilt,azimuth_from_inward_deg=azimuth,pad_raise_m=pad_raise,
              tilt_direction='tool tips toward arm base' if azimuth==0 else 'diagonal/side approach fallback'))
        session.publish(camera,target,pregrasp,stamp,plans,initial)
        plan_names=['pregrasp','descent','lift','transfer','place','retreat'] if place_aside else ['pregrasp','descent','lift']
        write(report/'planned_trajectories.json',{name:serializable_trajectory(plan) for name,plan in zip(plan_names,plans)})
        if place_aside:
            side_targets=side_place_targets(target,config)
            write(report/'side_place_targets.json',{name:pose.tolist() for name,pose in side_targets.items()})
        write(report/'clearance_check.json',dict(minimum_observed_open_tool_clearance_m=session.minimum_observed_clearance_m,
            clearance_semantics='conservative lower bound when CAD bounding spheres prove clearance; otherwise exact original sample distance',
            required_margin_m=config['scene_margin_m'],corrected_and_nominal_joint_states_checked=True,
            complete_obstacle_geometry_verified=False))
        planning_complete=True
        write(report/'status.json',dict(success=True,stage='plan_only',robot_motion_sent=False,
            planning_success=True,execution_success=None,execution_requested=args.execute,
            scene_limitations=scene['unobserved_geometry_verified']))
        write(report/'planning_performance.json',metrics.snapshot('complete_feasible'))
        print('Complete %s paths planned. execute=%s; report=%s'%(plan_names,args.execute,report),flush=True)
        if not args.execute:
            rospy.sleep(args.hold_seconds);return 0
        if place_aside and not config.get('side_place_scene_verified',False):
            raise RuntimeError('side-place execution requires side_place_scene_verified: true after checking carried-object and destination clearance; planning only is available without it')
        # A stale snapshot must never start a real descent.
        if (rospy.Time.now()-stamp).to_sec()>config['max_capture_age_sec']:
            raise RuntimeError('vision snapshot too old before first motion; recapture/replan')
        if hashlib.sha256(Path(config['calibration_file']).read_bytes()).hexdigest()!=session.calibration_hash:
            raise RuntimeError('calibration changed after planning')
        hardware=GripperIO(config)
        opened_state=hardware.open(config['open_width_m'])
        write(report/'gripper_open.json',opened_state)
        feedback=[];deadline=time.monotonic()+config['max_execution_sec']
        for name,trajectory,expected in [('pregrasp',first,pregrasp),('descent',descent,target)]:
            if time.monotonic()>deadline:raise RuntimeError('first-grasp time budget exceeded')
            write(report/'status.json',dict(success=False,stage='executing_'+name,robot_motion_sent=True))
            reached=session.execute_one(trajectory,tcp,expected);feedback.append(dict(stage=name,**reached))
            write(report/'execution_feedback.json',feedback)
        held=hardware.close();write(report/'gripper_contact.json',held)
        estimated_width=held['actpos']*.070/1000
        if abs(estimated_width-config['contact_width_m'])>.007:
            raise RuntimeError('force contact width inconsistent with the target wood strip; no lift')
        session.set_tool_width(estimated_width)
        actual_q,_=session.measured()
        session.check_trajectory(lift_plan,session.state_for(actual_q))
        if time.monotonic()>deadline:raise RuntimeError('grasp time budget exceeded before lift')
        write(report/'status.json',dict(success=False,stage='executing_lift',robot_motion_sent=True))
        reached=session.execute_one(lift_plan,tcp,lift);feedback.append(dict(stage='lift',**reached))
        write(report/'execution_feedback.json',feedback)
        rospy.sleep(1.)
        state=hardware.query('get_gripper_state');write(report/'gripper_after_lift.json',state)
        if state.get('error',0) or state.get('mode')!=6 or state.get('actpos',0)<30:
            raise RuntimeError('grip contact not retained after small lift')
        if place_aside:
            for name,trajectory in [('transfer',plans[3]),('place',plans[4])]:
                held_state=hardware.query('get_gripper_state')
                if held_state.get('error',0) or held_state.get('mode')!=6 or held_state.get('actpos',0)<30:
                    raise RuntimeError('grip contact lost before '+name+'; no automatic release or recovery')
                if time.monotonic()>deadline:raise RuntimeError('side placement time budget exceeded')
                write(report/'status.json',dict(success=False,stage='executing_'+name,robot_motion_sent=True))
                reached=session.execute_one(trajectory,tcp,side_targets[name])
                feedback.append(dict(stage=name,**reached));write(report/'execution_feedback.json',feedback)
            # Release only after execute_one has verified arrival at the placed pose.
            final_contact=hardware.query('get_gripper_state')
            if final_contact.get('error',0) or final_contact.get('mode')!=6 or final_contact.get('actpos',0)<30:
                raise RuntimeError('contact state unknown at placement; no automatic release')
            if time.monotonic()>deadline:raise RuntimeError('side placement time budget exceeded before release')
            released=hardware.open(config['open_width_m']);write(report/'gripper_release.json',released)
            session.set_tool_width(config['open_width_m'])
            if time.monotonic()>deadline:raise RuntimeError('side placement time budget exceeded before retreat')
            reached=session.execute_one(plans[5],tcp,side_targets['retreat'])
            feedback.append(dict(stage='retreat',**reached));write(report/'execution_feedback.json',feedback)
            write(report/'status.json',dict(success=True,stage='placed_aside_and_retreated',robot_motion_sent=True,
                planning_success=True,execution_success=True,
                side_place_offset_base_m=config.get('side_place_offset_base_m',[0.,-.05,0.]),automatic_object_tracking_verified=False))
            rospy.sleep(args.hold_seconds);return 0
        write(report/'status.json',dict(success=True,stage='lifted_10mm_contact_retained',robot_motion_sent=True,
            planning_success=True,execution_success=True,
            visual_or_operator_lift_confirmation_required=True,automatic_object_tracking_verified=False))
        print('Small lift completed; gripper contact retained. Need camera/operator confirmation of basket lift.',flush=True)
        rospy.sleep(args.hold_seconds)
        return 0
    except Exception as error:
        if session:session.stop()
        if hasattr(error,'gripper_feedback'):
            write(report/'gripper_failure_feedback.json',error.gripper_feedback)
        if not planning_complete:write(report/'planning_performance.json',metrics.snapshot(failure_kind(error)))
        write(report/'status.json',dict(success=False,stage='stopped',error=str(error),
             planning_success=planning_complete,execution_success=False if planning_complete and args.execute else None,
             planning_outcome=failure_kind(error),
             note='No automatic recovery, retreat or release is attempted after failure'))
        print('STOP:',error,'report:',report,file=sys.stderr,flush=True)
        return 2

if __name__=='__main__':
    sys.exit(main())
