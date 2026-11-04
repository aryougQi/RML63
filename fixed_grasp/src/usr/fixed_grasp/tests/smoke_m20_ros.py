#!/usr/bin/env python3
"""Isolated ROS protocol simulation. No real driver, MoveIt, socket query or motion."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 11349))
    with tempfile.TemporaryDirectory(prefix='m20_ros_smoke_') as directory:
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11349'
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'
        os.environ['ROS_HOME'] = directory
        os.environ['ROS_LOG_DIR'] = directory+'/logs'
        os.environ.pop('ROS_IP', None)
        import rosgraph
        import rospy
        import yaml
        from std_msgs.msg import String
        from sensor_msgs.msg import JointState
        from rm_msgs.msg import Manual_Set_Force_Pose, Joint_En_Flag
        from fixed_grasp.msg import FixedGraspResult, FixedGraspFeedback
        from core import Cancelled
        from m20_bridge import M20Bridge, Gripper
        log = open(directory+'/roscore.log', 'w')
        process = subprocess.Popen(['roscore', '-p', '11349'], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        bridge = None; stop = threading.Event()
        try:
            deadline = time.monotonic()+20.
            while not rosgraph.is_master_online():
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('isolated roscore unavailable')
                time.sleep(.05)
            # A synthetic publisher on this separate master represents hardware.
            # It contains no rm_driver implementation and never opens robot TCP.
            rospy.init_node('rm_driver', disable_signals=True)
            rospy.set_param('~config', str(ROOT/'config/fixed.yaml'))
            rospy.set_param('~waypoints', str(ROOT/'config/waypoints.yaml'))
            rospy.set_param('~grasp2_waypoints', str(ROOT/'config/waypoints_2.yaml'))
            rospy.set_param('~m20_database', directory+'/transactions.sqlite3')
            rospy.set_param('~allow_execution', True)
            rospy.set_param('/move_group/allow_trajectory_execution', True)
            statuses = []; feedback = []
            rospy.Subscriber('/m20_cooperation/arm/status', String,
                             lambda msg: statuses.append(json.loads(msg.data)), queue_size=100)
            rospy.Subscriber('/m20_cooperation/arm/feedback', String,
                             lambda msg: feedback.append(json.loads(msg.data)), queue_size=100)
            publishers = {name: rospy.Publisher(topic, kind, queue_size=100) for name, topic, kind in (
                ('request', '/m20_cooperation/arm/request', String),
                ('robot', '/m20_cooperation/robot/status', String),
                ('task', '/m20_cooperation/task/status', String),
                ('joint', '/joint_states', JointState),
                ('fault', '/rm_driver/JointErrorCode', Manual_Set_Force_Pose),
                ('enable', '/rm_driver/Udp_Joint_En_Flag', Joint_En_Flag))}

            class FakeArm:
                def __init__(self):
                    self.lock = threading.RLock(); self.active = None; self.needs_recovery = False
                    self.calls = []; self.cancelled = threading.Event(); self.duration = .15

                def goal(self, handle, grasp=1, guard=None):
                    with self.lock:
                        assert self.active is None
                        self.active = handle; self.calls.append(grasp); self.cancelled.clear()
                    def work():
                        try:
                            handle.set_accepted()
                            started = time.monotonic()
                            while time.monotonic()-started < self.duration:
                                guard()
                                if self.cancelled.is_set(): raise Cancelled('simulated stop')
                                time.sleep(.02)
                            handle.publish_feedback(FixedGraspFeedback(stage='DONE', detail='synthetic completion', holding=False))
                            handle.set_succeeded(FixedGraspResult(success=True, planning_success=True,
                                execution_success=True, grasp_verified=True, stage='DONE', reason='synthetic completion'))
                        except Cancelled as error:
                            handle.set_canceled(FixedGraspResult(stage='Lift', reason=str(error)))
                        finally:
                            with self.lock: self.active = None
                    threading.Thread(target=work, daemon=True).start()

                def cancel(self, handle, grasp):
                    with self.lock:
                        if self.active is handle: self.cancelled.set()

            arm = FakeArm()
            idle_gripper = dict(status=1, enable=1, error=0, mode=3, actpos=714, current_force=0)
            with patch.object(Gripper, 'query', return_value=idle_gripper):
                bridge = M20Bridge(arm, read_gripper=lambda: idle_gripper)
                shared = yaml.safe_load((ROOT/'config/waypoints.yaml').read_text())
                state = dict(seq=0, tx='tx-1', heartbeats=True)
                def publish():
                    while not stop.is_set():
                        now = rospy.Time.now(); stamp = now.to_sec(); state['seq'] += 1
                        joints = JointState(); joints.header.stamp = now
                        joints.name = shared['identity']['joint_names']; joints.position = shared['waypoints']['Home']['joints_rad']
                        publishers['joint'].publish(joints)
                        publishers['fault'].publish(Manual_Set_Force_Pose(joint=[0]*6))
                        publishers['enable'].publish(Joint_En_Flag(joint_en_flag=[True]*6))
                        if state['heartbeats']:
                            common = dict(schema_version=1, seq=state['seq'], sample_stamp=stamp)
                            publishers['robot'].publish(String(data=json.dumps(dict(common, boot_id='dog-sim',
                                healthy=True, stationary=True, posture='crouched', basic_age_s=.1, motion_age_s=.05))))
                            publishers['task'].publish(String(data=json.dumps(dict(common, boot_id='task-sim',
                                state='ARM_EXECUTING', run_id='run-sim', transaction_id=state['tx']))))
                        stop.wait(.05)
                threading.Thread(target=publish, daemon=True).start()
                def wait(condition, label, timeout=8.):
                    deadline = time.monotonic()+timeout
                    while not condition():
                        if time.monotonic() >= deadline:
                            print(json.dumps(dict(timeout=label, calls=arm.calls, feedback=feedback[-6:],
                                status=statuses[-1] if statuses else None, local=bridge.local())), flush=True)
                            raise RuntimeError('timeout: '+label)
                        time.sleep(.02)
                def request(tx, request_id):
                    now = rospy.Time.now().to_sec()
                    return dict(schema_version=1, request_id=request_id, source='task', run_id='run-sim',
                        mission_id='mission-sim', waypoint_id=tx, transaction_id=tx, operation='execute', task_type='place',
                        issued_at=now, expires_at=now+10., robot_state_ref=dict(boot_id='dog-sim', seq=state['seq']), payload={})
                def succeeded(tx):
                    return any(f.get('transaction_id') == tx and f['status'] == 'succeeded' and f['task_success'] for f in feedback)
                wait(lambda: statuses and statuses[-1]['ready'], 'initial measured readiness')
                first = request('tx-1', 'req-1')
                publishers['request'].publish(String(data=json.dumps(first)))
                wait(lambda: succeeded('tx-1'), 'first placement')
                wait(lambda: statuses[-1]['arm_safe_to_move'] and statuses[-1]['transaction_id'] == 'tx-1', 'first stow confirmation')
                publishers['request'].publish(String(data=json.dumps(first)))
                time.sleep(.2)
                assert arm.calls == [1], arm.calls
                state['tx'] = 'tx-2'; time.sleep(.15)
                second = request('tx-2', 'req-2')
                publishers['request'].publish(String(data=json.dumps(second)))
                wait(lambda: succeeded('tx-2'), 'second placement')
                wait(lambda: statuses[-1]['arm_safe_to_move'] and statuses[-1]['transaction_id'] == 'tx-2', 'second stow confirmation')
                assert arm.calls == [1, 2], arm.calls
                assert statuses[-1]['next_grasp'] is None
                # Explicit simulated replenishment starts a new cycle; old rows remain.
                with bridge.engine.lock: bridge.ledger.reset_sequence()
                arm.duration = 1.5; state['tx'] = 'tx-3'; time.sleep(.15)
                publishers['request'].publish(String(data=json.dumps(request('tx-3', 'req-3'))))
                wait(lambda: arm.calls == [1, 2, 1], 'watchdog job accepted')
                state['heartbeats'] = False
                wait(lambda: any(f.get('transaction_id') == 'tx-3' and f['status'] == 'failed' for f in feedback), 'heartbeat stop')
                failure = next(f for f in reversed(feedback) if f.get('transaction_id') == 'tx-3' and f['status'] == 'failed')
                assert failure['task_success'] is False
                assert bridge.ledger.cursor == 0
                report = dict(scope='Separate loopback ROS master, synthetic hardware and arm controller only; no physical driver, robot query or movement.',
                    success=True, dispatched_blocks=arm.calls, duplicate_replayed_without_dispatch=True,
                    first_cycle_blocks=arm.calls[:2], explicit_replenishment_reset=True,
                    heartbeat_loss_error=failure['error_code'], status_messages=len(statuses), feedback_messages=len(feedback))
                target = ROOT/'reports/m20_ros_smoke.json'
                target.write_text(json.dumps(report, indent=2)+'\n')
                print(json.dumps(report, indent=2), flush=True)
        finally:
            stop.set()
            if bridge: bridge.shutdown()
            rospy.signal_shutdown('isolated protocol simulation complete')
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
            log.close()


if __name__ == '__main__': main()
