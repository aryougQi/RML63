#!/usr/bin/env python3
"""Replay saved perception/state through RobotSession; hardware execution is disabled.

Uses existing MoveIt planning services, reconstructs the saved observed scene,
never launches drivers/camera or sends an action/GripperIO command. Do not run
concurrently with any hardware grasp process. Report index is isolated.
"""
import argparse
import functools
import json
import sys
import time
import types
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE/'scripts'))


def summary(values):
    return dict(count=len(values),total_sec=sum(values),mean_sec=float(np.mean(values)) if values else None,
                p95_sec=float(np.percentile(values,95)) if values else None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--baseline',type=Path)
    p.add_argument('--mode',default='first_feasible')
    p.add_argument('--budget',type=float,default=20.)
    p.add_argument('--coarse',action='store_true')
    p.add_argument('--offset-y',type=float)
    p.add_argument('--scenario',choices=['normal','unreachable'],default='normal')
    p.add_argument('--validate-run',type=Path,help='validate saved trajectories using the selected code version; no search')
    args=p.parse_args()
    # Read-only process guard: never interfere with an execution that is running.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==__import__('os').getpid():continue
        try:cmd=(proc/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if b'--execute' in cmd and any(Path(c.decode(errors='ignore')).name in
                ('grasp_demo.py','resume_side_place.py','return_demo.py') for c in cmd[:3] if c):
            raise RuntimeError('hardware execution process is active; replay refused')
    if args.baseline:
        # Compile exact archived sources with original __file__ so existing
        # workspace paths resolve identically. No source files are overwritten.
        for name in ('grasp_geometry','grasp_candidates','grasp_scoring','handle_pipeline','grasp_ros','grasp_demo'):
            file=args.baseline/'scripts'/(name+'.py')
            module=types.ModuleType(name);module.__file__=str(HERE/'scripts'/(name+'.py'))
            sys.modules[name]=module;exec(compile(file.read_text(),module.__file__,'exec'),module.__dict__)
    import grasp_demo as demo
    from grasp_ros import RobotSession
    import rospy
    context=json.loads((args.run/'capture_context.json').read_text())
    q=np.asarray(context['ros_capture']['joint_evidence']['measured_joints_mean_rad'])
    raw=sorted(Path(context['raw_directory']).glob('frame_*'))[-1]
    depth=np.load(str(raw/'depth_raw.npy'),allow_pickle=False)
    import cv2
    mask=cv2.imread(str(Path(context['vision_report_directory'])/'handle_mask.png'),0)
    if mask is None:raise RuntimeError('saved mask unavailable')
    cfg=yaml.safe_load((args.baseline/'config/grasp.yaml' if args.baseline else HERE/'config/grasp.yaml').read_text())
    cfg.update(planning_mode=args.mode,planning_total_timeout_sec=args.budget,
        coarse_check_enabled=args.coarse)
    if args.offset_y is not None:cfg['side_place_offset_base_m']=[0.,args.offset_y,0.]
    args.output.mkdir(parents=True,exist_ok=False)
    config=args.output/'replay_config.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    timings={};rpc=[];paths=[];events=[]
    class Replay(RobotSession):
        def __init__(self,config):
            super().__init__(config)
            if self.calibration_hash!=context['calibration_sha256']:
                raise RuntimeError('saved calibration does not match current model')
            proxy=self.validity
            def validity(request):
                start=time.monotonic()
                try:return proxy(request)
                finally:rpc.append(time.monotonic()-start)
            self.validity=validity
        def measured(self):return q.copy(),rospy.Time.now()
        def verify_hardware(self,*a,**k):pass
        def execute_one(self,*a,**k):raise AssertionError('HARDWARE EXECUTION FORBIDDEN IN REPLAY')
        def stop(self):pass
    for name in ('solve','check_state','plan_descent','plan_joint','add_scene'):
        original=getattr(Replay,name)
        def wrap(original,name):
            @functools.wraps(original)
            def wrapped(self,*a,**kw):
                start=time.monotonic();count_before=len(timings.get('solve',[]));failed=False;result=None
                try:
                    result=original(self,*a,**kw);return result
                except Exception:
                    failed=True;raise
                finally:
                    timings.setdefault(name,[]).append(time.monotonic()-start)
                    if name=='plan_descent':
                        paths.append(dict(duration_sec=time.monotonic()-start,ik_calls=len(timings.get('solve',[]))-count_before,
                            failed=failed,waypoints=len(result.joint_trajectory.points) if result is not None else None))
            return wrapped
        setattr(Replay,name,wrap(original,name))
    if args.validate_run:
        import moveit_commander
        from grasp_ros import state_after
        from resume_side_place import load_trajectory
        from scipy.spatial.transform import Rotation
        moveit_commander.roscpp_initialize(sys.argv);rospy.init_node('recorded_trajectory_oracle',anonymous=True)
        s=Replay(cfg);s.base_depth=np.asarray(context['base_depth']);s.target=np.asarray(context['base_grasp'])
        mount=yaml.safe_load(Path(cfg['mount_file']).read_text())['launch_defaults']
        g=demo.GripperGeometry(demo.SRC/'rml63_tools_description/upstream/EG2-4C2.urdf',
            demo.SRC/'rml63_tools_description/meshes/eg2_4c2',mount['mount_xyz'],mount['mount_rpy'])
        s.add_scene(s.base_depth,depth,context['metadata'],mask,context['visual_result']['support_plane'],g,g.at_width(cfg['open_width_m'])['linkage_angle_rad'])
        saved=json.loads((args.validate_run/'planned_trajectories.json').read_text())
        selected=json.loads((args.validate_run/'selected_targets.json').read_text())
        side=json.loads((args.validate_run/'side_place_targets.json').read_text())
        tcp=np.asarray(selected['flange_tcp']);grasp=np.asarray(selected['base_grasp']);pre=np.asarray(selected['base_pregrasp'])
        endpoints=dict(descent=(pre,grasp),lift=(grasp,np.asarray(side['lift'])),
            transfer=(np.asarray(side['lift']),np.asarray(side['transfer'])),
            place=(np.asarray(side['transfer']),np.asarray(side['place'])),
            retreat=(np.asarray(side['place']),np.asarray(side['retreat'])))
        state=s.state_for(q);errors={};points=0
        for name in ('pregrasp','descent','lift','transfer','place','retreat'):
            trajectory=load_trajectory(saved[name])
            s.set_tool_width(cfg['open_width_m'] if name in ('pregrasp','descent','retreat') else cfg['contact_width_m'])
            s.check_trajectory(trajectory,state);state=state_after(state,trajectory)
            for point in trajectory.joint_trajectory.points:
                values=dict(zip(trajectory.joint_trajectory.joint_names,point.positions))
                vector=np.asarray([values[n] for n in s.active]);points+=1
                for n,value in zip(s.active,vector):
                    low,high=s.robot.get_joint(n).bounds()
                    assert low<=value<=high,'joint limit exceeded'
                if name not in endpoints:continue
                a,b=endpoints[name];pose=s.chain.fk(vector,s.offsets)@tcp
                line=b[:3,3]-a[:3,3];den=float(line@line)
                fraction=float(np.clip((pose[:3,3]-a[:3,3])@line/den,0,1)) if den else 0.
                position=float(np.linalg.norm(pose[:3,3]-a[:3,3]-fraction*line))
                rotation=(Rotation.from_matrix if hasattr(Rotation,'from_matrix') else Rotation.from_dcm)(a[:3,:3].T@pose[:3,:3])
                angle=float(np.linalg.norm(rotation.as_rotvec()))
                errors.setdefault(name,[]).append((position,angle))
                assert position<=.0011 and angle<=np.radians(.51),(name,position,angle)
        result=dict(success=True,source_run=str(args.run),validated_run=str(args.validate_run),
            oracle='archived original dense checks' if args.baseline else 'current checks',points_checked=points,
            collision_rpc=summary(rpc),hardware_motion_sent=False,
            errors={k:dict(max_position_m=max(p for p,a in v),max_orientation_rad=max(a for p,a in v)) for k,v in errors.items()})
        (args.output/'benchmark.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0
    original_prepare=demo.prepare_candidates
    def prepare(*a,**k):
        start=time.monotonic()
        try:
            candidates,report=original_prepare(*a,**k)
            if args.scenario=='unreachable':
                for candidate in candidates:candidate.tcp[0,3]+=2.
            return candidates,report
        finally:timings.setdefault('candidate_geometry_scoring',[]).append(time.monotonic()-start)
    original_write=demo.write
    start=time.monotonic()
    def write(path,value):
        if Path(path).name=='planning_progress.json':events.append(dict(value,wall_elapsed_sec=time.monotonic()-start))
        return original_write(path,value)
    def acquire(*a,**k):return context,q.copy(),rospy.Time.now(),depth,context['metadata'],mask
    def no_gripper(*a,**k):raise AssertionError('GRIPPER COMMANDS FORBIDDEN IN REPLAY')
    real_init=rospy.init_node
    def init(*a,**k):return real_init('robot_session_replay',anonymous=True)
    with patch.object(demo,'HERE',args.output),patch.object(demo,'RobotSession',Replay), \
         patch.object(demo,'acquire',acquire),patch.object(demo,'GripperIO',no_gripper), \
         patch.object(demo,'prepare_candidates',prepare),patch.object(demo,'write',write), \
         patch.object(rospy,'init_node',init), \
         patch.object(sys,'argv',['grasp_demo.py','--config',str(config),'--hold-seconds','0']):
        code=demo.main()
    total=time.monotonic()-start
    index=json.loads((args.output/'reports/latest_run.json').read_text())
    run=Path(index['run_directory']);status=json.loads((run/'status.json').read_text())
    complete=[e for e in events if e['stage']=='complete_path']
    result=dict(run=str(args.run),output_run=str(run),baseline=bool(args.baseline),exit_code=code,status=status,
        end_to_end_sec=total,first_solution_sec=complete[0]['elapsed_sec'] if complete else None,
        search_return_sec=events[-1]['elapsed_sec'] if events else None,
        methods={k:summary(v) for k,v in timings.items()},collision_rpc=summary(rpc),paths=paths,
        hardware_motion_sent=False,source_state='recorded joints; no live arm state used as planning start')
    (args.output/'benchmark.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    return code
if __name__=='__main__':sys.exit(main())
