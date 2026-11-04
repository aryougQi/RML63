#!/usr/bin/env python3
"""Compare complete PLAN_ONLY paths on an isolated master; never start drivers."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
from unittest.mock import patch

import numpy as np
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root/'reports/planning_comparison')
    parser.add_argument('--port', type=int, default=11339)
    parser.add_argument('--repeats', type=int, default=2)
    args = parser.parse_args()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', args.port))
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % args.port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(args.output/'ros_logs')
    os.environ['ROS_HOME'] = str(args.output/'ros_home')
    os.environ.pop('ROS_IP', None)
    import rosgraph
    import rospy
    import moveit_commander
    from sensor_msgs.msg import JointState
    import backend
    from backend import Backend, Session, load_settings
    stop = threading.Event()
    runs = []
    with (args.output/'moveit.log').open('w') as log:
        launch = subprocess.Popen(['roslaunch', 'rml63_tools_description', 'realrobot.launch',
            'start_driver:=false', 'allow_execution:=false', 'use_rviz:=false'],
            stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy(), start_new_session=True)
        try:
            deadline = time.monotonic()+45.
            while not rosgraph.is_master_online():
                if launch.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('isolated MoveIt launch failed; inspect moveit.log')
                time.sleep(.1)
            moveit_commander.roscpp_initialize([])
            rospy.init_node('fixed_grasp_offline_comparison', anonymous=True, disable_signals=True)
            rospy.wait_for_service('/check_state_validity', timeout=40.)
            assert rospy.get_param('/move_group/allow_trajectory_execution') is False
            pubs, subs, services = rosgraph.Master(rospy.get_name()).getSystemState()
            nodes = {node for entries in (pubs, subs, services) for _, owners in entries for node in owners}
            assert '/rm_driver' not in nodes
            shared = yaml.safe_load((root/'config/waypoints.yaml').read_text())
            names = shared['identity']['joint_names']
            q = shared['waypoints']['Home']['joints_rad']
            publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)
            def publish():
                while not stop.is_set():
                    msg = JointState(); msg.header.stamp = rospy.Time.now(); msg.name = names; msg.position = q
                    publisher.publish(msg)
                    stop.wait(.05)
            threading.Thread(target=publish, daemon=True).start()
            rospy.sleep(1.)

            class OfflineSession(Session):
                def execute_one(self, *args, **kwargs):
                    raise AssertionError('motion forbidden in offline benchmark')

            class OfflineGripper:
                def __init__(self, *args): pass
                def query(self, *args): raise AssertionError('hardware query forbidden')
                def open(self, *args): raise AssertionError('gripper motion forbidden')
                def close(self, *args): raise AssertionError('gripper motion forbidden')

            path_names = ['Home', 'Pregrasp', 'Grasp', 'Lift', 'Preplace', 'Place', 'Retreat', 'ReturnHome']
            with patch.object(backend, 'Session', OfflineSession), patch.object(backend, 'Gripper', OfflineGripper):
                for repetition in range(args.repeats):
                    for grasp in (1, 2):
                        for fast in ((False, True) if repetition % 2 == 0 else (True, False)):
                            config = load_settings(root/'config/fixed.yaml')
                            config['fast_ik_enabled'] = fast
                            config['planning_attempts'] = 1  # Isolate IK; compare the same search policy.
                            b = Backend(config, root/'config/waypoints.yaml', False, lambda: None, 'offline',
                                grasp_waypoints=root/'config/waypoints_2.yaml' if grasp == 2 else None)
                            b.preflight(False, True)
                            started = time.monotonic()
                            result = dict(repetition=repetition, grasp=grasp, fast_ik=fast)
                            try:
                                plans = b.plan(path_names, lambda: None)
                                result.update(success=True, points={n: len(t.joint_trajectory.points) for n, t in plans.items()})
                            except Exception as error:
                                result.update(success=False, reason=str(error))
                            result['planning_wall_sec'] = time.monotonic()-started
                            result['metrics'] = b.s.metrics.snapshot('success' if result['success'] else 'failed')
                            b.restore_preview()
                            runs.append(result)
                            print(json.dumps({k: v for k, v in result.items() if k not in ('metrics', 'points')}), flush=True)
            summary = {}
            for fast, label in ((False, 'numerical_ik'), (True, 'compiled_ik')):
                selected = [r for r in runs if r['fast_ik'] == fast]
                successful = [r['planning_wall_sec'] for r in selected if r['success']]
                summary[label] = dict(runs=len(selected), successes=len(successful),
                    median_success_wall_sec=float(np.median(successful)) if successful else None)
            report = dict(scope='Isolated MoveIt PLAN_ONLY, drivers absent and execution disabled; same existing-scene robot model, taught Home start, 8 stages. Numerical and compiled IK use the same measured-waypoint strategy and one OMPL search attempt. Small stochastic sample; not hardware acceptance.',
                summary=summary, runs=runs)
            (args.output/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps(summary, indent=2), flush=True)
        finally:
            stop.set()
            moveit_commander.roscpp_shutdown()
            if not rospy.is_shutdown():
                rospy.signal_shutdown('offline comparison complete')
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try: launch.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()


if __name__ == '__main__': main()
