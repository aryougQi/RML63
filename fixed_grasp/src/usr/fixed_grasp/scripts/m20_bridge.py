"""M20 topic adapter embedded in the existing single-owner arm server."""
from collections import deque
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rm_msgs.msg import Manual_Set_Force_Pose, Joint_En_Flag
from fixed_grasp.msg import FixedGraspGoal
from backend import Gripper, load_settings
from grasp_profiles import load_task_waypoints
from m20_ledger import Ledger, DEFAULT_PATH, TEST_PATH, canonical
from m20_protocol import Coordinator, parse_json, number


class ArmObserver:
    """Fresh hardware evidence for readiness and actual, empty Home stowage."""
    def __init__(self, config, data):
        self.config = config; self.lock = threading.RLock()
        self.names = data['identity']['joint_names']
        self.home = np.asarray(data['waypoints']['Home']['joints_rad'], float)
        self.joints = deque(maxlen=100); self.grippers = deque(maxlen=3)
        self.faults = None; self.enabled = None

    def joint(self, message, now):
        with self.lock:
            try:
                stamp = message.header.stamp.to_sec()
                values = dict(zip(message.name, message.position))
                q = np.asarray([values[name] for name in self.names], float)
                if (not np.isfinite(q).all() or not 0 <= now-stamp <= self.config['joint_feedback_max_age_sec'] or
                        (self.joints and stamp <= self.joints[-1][0])):
                    self.joints.clear()
                    return
                self.joints.append((stamp, q))
            except (AttributeError, KeyError, TypeError, ValueError):
                self.joints.clear()

    def gripper(self, state, now):
        with self.lock:
            if state is None:
                self.grippers.clear()
            else:
                self.grippers.append((now, dict(state)))

    def fault(self, message, now):
        with self.lock: self.faults = (now, list(message.joint)) if message is not None else None

    def enable(self, message, now):
        with self.lock: self.enabled = (now, list(message.joint_en_flag)) if message is not None else None

    def snapshot(self, now, busy, recovery, allow_execution):
        with self.lock:
            fresh = lambda sample, age=.5: sample is not None and 0 <= now-sample[0] <= age
            joints_ok = bool(self.joints and fresh(self.joints[-1], self.config['joint_feedback_max_age_sec']))
            faults_ok = fresh(self.faults) and self.faults[1] == [0]*len(self.names)
            enable_ok = fresh(self.enabled) and self.enabled[1] == [True]*len(self.names)
            def gripper_ok(sample, empty=False):
                if not fresh(sample): return False
                state = sample[1]; force = state.get('current_force')
                return (state.get('status') == 1 and state.get('enable') == 1 and state.get('error') == 0 and
                        state.get('mode') in ((1, 2, 3) if empty else (1, 2, 3, 4, 5, 6)) and
                        number(force) and (not empty or abs(force) < 50))
            healthy = bool(allow_execution and joints_ok and faults_ok and enable_ok and
                           self.grippers and gripper_ok(self.grippers[-1]))
            recent = [(t, q) for t, q in self.joints if 0 <= now-t <= .2]
            home = bool(len(recent) >= 3 and recent[-1][0]-recent[0][0] >= .1 and
                        np.max(abs(recent[-1][1]-self.home)) <= np.radians(.5) and
                        np.max(np.ptp(np.asarray([q for _, q in recent]), axis=0)) <=
                        np.radians(self.config['max_capture_motion_deg']))
            empty = len(self.grippers) == 3 and all(gripper_ok(sample, empty=True) for sample in self.grippers)
            safe = bool(healthy and not busy and home and empty)
            return dict(healthy=healthy, ready=bool(safe and not recovery), arm_safe_to_move=safe)


class LocalGoal:
    def __init__(self, bridge, record):
        self.bridge = bridge; self.record = record
        self.tx = record['tx']
        request = json.loads(record['request'])
        self.goal_id = SimpleNamespace(id='m20:'+self.tx+':'+request['request_id'])
        self.goal = FixedGraspGoal(mode=1, full_transfer=True)

    def get_goal(self): return self.goal
    def get_goal_id(self): return self.goal_id
    def set_accepted(self): pass

    def publish_feedback(self, feedback):
        self.bridge.engine.progress(self.tx, feedback.stage, feedback.detail)
        self.bridge.flush()

    def terminal(self, result, succeeded=False, cancelled=False):
        complete = bool(succeeded and result.success and result.planning_success and result.execution_success and
                        result.grasp_verified and result.stage == 'DONE')
        self.bridge.engine.finish(self.tx, complete, cancelled, result.reason, result.stage)
        self.bridge.flush()

    def set_succeeded(self, result): self.terminal(result, succeeded=True)
    def set_rejected(self, result): self.terminal(result)
    def set_aborted(self, result): self.terminal(result)
    def set_canceled(self, result): self.terminal(result, cancelled=True)


class M20Bridge:
    def __init__(self, server, read_gripper=None):
        self.server = server; self.handle = None; self.stop = threading.Event()
        self.clock = lambda: rospy.Time.now().to_sec()
        config = load_settings(rospy.get_param('~config'))
        data, _ = load_task_waypoints(rospy.get_param('~waypoints'), rospy.get_param('~grasp2_waypoints'))
        self.observer = ArmObserver(config, data)
        self.test_mode = rospy.get_param('~m20_test_mode', False)
        self.ledger = Ledger(rospy.get_param('~m20_test_database', TEST_PATH) if self.test_mode else
                             rospy.get_param('~m20_database', DEFAULT_PATH))
        self.engine = Coordinator(self.ledger, self.clock)
        self.allow_execution = (rospy.get_param('~allow_execution', False) and
                                rospy.get_param('/move_group/allow_trajectory_execution', False))
        prefix = '/m20_cooperation_test' if self.test_mode else '/m20_cooperation'
        self.feedback_pub = rospy.Publisher(prefix+'/arm/feedback', String, queue_size=100, latch=False)
        self.status_pub = rospy.Publisher(prefix+'/arm/status', String, queue_size=10, latch=False)
        self.subscribers = [
            rospy.Subscriber(prefix+'/arm/request', String, self.request, queue_size=50),
            rospy.Subscriber(prefix+'/robot/status', String, lambda msg: self.state('robot', msg), queue_size=100),
            rospy.Subscriber(prefix+'/task/status', String, lambda msg: self.state('task', msg), queue_size=100),
            rospy.Subscriber('/joint_states', JointState, lambda msg: self.hardware('joint', msg), queue_size=100),
            rospy.Subscriber('/rm_driver/JointErrorCode', Manual_Set_Force_Pose,
                             lambda msg: self.hardware('fault', msg), queue_size=10),
            rospy.Subscriber('/rm_driver/Udp_Joint_En_Flag', Joint_En_Flag,
                             lambda msg: self.hardware('enable', msg), queue_size=10)]
        # Query only: the observer never creates or publishes gripper commands.
        self.gripper = Gripper.__new__(Gripper)
        self.gripper.config = config; self.gripper.check = lambda: None
        self.read_gripper = read_gripper or (lambda: self.gripper.query('get_gripper_state'))
        self.worker = threading.Thread(target=self.poll_gripper, daemon=True)
        self.worker.start()
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('M20 place bridge enabled: prefix=%s, sequence 1 -> 2; ledger=%s', prefix, self.ledger.path)

    def poll_gripper(self):
        while not self.stop.is_set():
            try:
                self.observer.gripper(self.read_gripper(), self.clock())
            except Exception as error:
                self.observer.gripper(None, self.clock())
                rospy.logwarn_throttle(5., 'M20 arm observer gripper query failed: %s', error)
            # Three samples must fit comfortably inside the 0.5 s freshness
            # window; slower polling would make otherwise-ready status flicker.
            self.stop.wait(.1)

    def local(self):
        # Never acquire the server lock while holding the coordinator lock.
        with self.server.lock:
            busy = self.server.active is not None
            recovery = self.server.needs_recovery
        return self.observer.snapshot(self.clock(), busy, recovery, self.allow_execution)

    def hardware(self, channel, message):
        caller = getattr(message, '_connection_header', {}).get('callerid')
        getattr(self.observer, channel)(message if caller == '/rm_driver' else None, self.clock())

    def flush(self):
        for message in self.engine.drain():
            self.feedback_pub.publish(String(data=canonical(message)))

    def dispatch(self, command):
        if not command or self.stop.is_set(): return
        operation, record = command
        if operation == 'execute':
            self.handle = LocalGoal(self, record)
            self.server.goal(self.handle, record['block'], guard=lambda: self.engine.guard(record['tx']))
        elif self.handle and self.handle.tx == record['tx']:
            self.server.cancel(self.handle, record['block'])

    def request(self, message):
        if self.stop.is_set(): return
        try:
            request = parse_json(message.data)
        except (ValueError, TypeError) as error:
            rospy.logwarn('Invalid M20 arm request: %s', error)
            return
        # Refresh readiness before acceptance; timer and requests share no control lock.
        self.dispatch(self.engine.tick(self.local()))
        command = self.engine.handle(request)
        self.flush()  # Acceptance was committed durably before any dispatch.
        self.dispatch(command)

    def state(self, channel, message):
        if self.stop.is_set(): return
        try: data = parse_json(message.data)
        except (ValueError, TypeError): data = {}
        self.engine.ingest(channel, data)

    def tick(self, _event):
        if self.stop.is_set(): return
        self.dispatch(self.engine.tick(self.local()))
        self.flush()
        with self.engine.lock:
            status = self.engine.status()
            status['manual_test'] = self.test_mode
            if self.test_mode:
                now = self.clock()
                status['manual_robot_state_ref'] = (dict(boot_id=self.engine.robot.boot, seq=self.engine.robot.seq)
                                                    if self.engine.robot.fresh(now) else None)
                task = self.engine.task.current if self.engine.task.fresh(now) else None
                status['manual_authorized_transaction_id'] = (task.get('transaction_id') if task and
                    task.get('state', task.get('phase')) == 'ARM_EXECUTING' else None)
        self.status_pub.publish(String(data=canonical(status)))

    def shutdown(self):
        if self.stop.is_set(): return
        self.stop.set(); self.timer.shutdown()
        if self.handle:
            self.server.cancel(self.handle, self.handle.record['block'])
        self.engine.shutdown()
        self.ledger.close()
