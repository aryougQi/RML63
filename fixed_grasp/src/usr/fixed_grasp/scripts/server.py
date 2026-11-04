#!/usr/bin/env python3
"""Single active goal; new goals are rejected, never preempt the current task."""
import fcntl
import threading
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import actionlib
import rospy
import moveit_commander
from fixed_grasp.msg import FixedGraspAction, FixedGraspFeedback, FixedGraspResult
from core import Runner, Cancelled
from backend import Backend, load_settings
from grasp_profiles import action_name, load_task_waypoints


class Server:
    def __init__(self):
        self.lock = threading.RLock()
        self.runner = None
        self.active = None
        self.active_grasp = None
        self.backend = None
        self.needs_recovery = False
        self.recovery_stage = ''
        self.recovery_reason = ''
        self.m20_test_mode = rospy.get_param('~m20_test_mode', False)
        self.m20_enabled = rospy.get_param('~m20_enabled', False) or self.m20_test_mode
        # Shared across node names/namespaces on this host, released on exit.
        self.lease = open('/tmp/fixed_grasp_robot.lock', 'a')
        fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.server = actionlib.ActionServer(action_name(1), FixedGraspAction,
                                             self.goal, self.cancel, auto_start=False)
        self.server_2 = actionlib.ActionServer(action_name(2), FixedGraspAction,
            lambda handle: self.goal(handle, 2), lambda handle: self.cancel(handle, 2), auto_start=False)
        self.home_server = actionlib.ActionServer('/fixed_grasp/home', FixedGraspAction,
            lambda handle: self.goal(handle, 'home'), lambda handle: self.cancel(handle, 'home'), auto_start=False)
        self.empty_home_server = actionlib.ActionServer('/fixed_grasp/home_empty', FixedGraspAction,
            lambda handle: self.goal(handle, 'home', confirm_empty=True),
            lambda handle: self.cancel(handle, 'home'), auto_start=False)
        self.server.start()
        self.server_2.start()
        self.home_server.start()
        self.empty_home_server.start()
        rospy.on_shutdown(self.shutdown)
        self.m20 = None
        if self.m20_enabled:
            from m20_bridge import M20Bridge
            self.m20 = M20Bridge(self)

    def goal(self, handle, grasp=1, confirm_empty=False, guard=None):
        with self.lock:
            if self.active is not None:
                stage = self.runner.stage if self.runner else ''
                handle.set_rejected(FixedGraspResult(stage=stage,
                    reason='BUSY: another task is active; wait for it to finish or cancel its own goal'))
                return
            if confirm_empty and grasp != 'home':
                handle.set_rejected(FixedGraspResult(reason='empty confirmation is only supported for Home'))
                return
            if self.needs_recovery and not (grasp == 'home' and confirm_empty):
                handle.set_rejected(FixedGraspResult(stage=getattr(self, 'recovery_stage', ''),
                    reason='RECOVERY_REQUIRED: manually recover and confirm the gripper is empty, then use '
                           'client.py --home --confirm-empty [--execute]; previous result: '+
                           getattr(self, 'recovery_reason', 'unknown')))
                return
            if handle.get_goal().mode not in (0, 1):
                handle.set_rejected(FixedGraspResult(reason='invalid mode'))
                return
            if getattr(self, 'm20_enabled', False) and handle.get_goal().mode == 1 and guard is None:
                if not (getattr(self, 'm20_test_mode', False) and grasp == 'home' and confirm_empty):
                    handle.set_rejected(FixedGraspResult(reason='M20_MODE: execute must come from the authorized M20 place bridge; '
                        'restart with m20_enabled:=false for manual recovery'))
                    return
            if grasp == 2:
                try:
                    load_task_waypoints(rospy.get_param('~waypoints'), rospy.get_param('~grasp2_waypoints'))
                except Exception as error:
                    handle.set_rejected(FixedGraspResult(reason=str(error)))
                    return
            self.active = handle
            self.active_grasp = grasp
            self.runner = Runner(None, lambda stage, detail, holding:
                handle.publish_feedback(FixedGraspFeedback(stage, detail, holding)), guard=guard)
            handle.set_accepted()
            threading.Thread(target=self.work, args=(handle, self.runner, grasp, confirm_empty), daemon=True).start()

    def cancel(self, handle, grasp=1):
        with self.lock:
            if (self.active and grasp == getattr(self, 'active_grasp', 1) and
                    handle.get_goal_id().id == self.active.get_goal_id().id):
                self.runner.cancel.set()
                if self.backend:
                    try:
                        self.backend.stop()
                    except Exception as error:
                        rospy.logwarn('stop request failed; worker will finalize recovery state: %s', error)

    def shutdown(self):
        with self.lock:
            if self.runner: self.runner.cancel.set()
            if self.backend:
                try:
                    self.backend.stop()
                except Exception as error:
                    rospy.logwarn('shutdown stop failed: %s', error)

    def work(self, handle, runner, grasp=1, confirm_empty=False):
        result = FixedGraspResult()
        cancelled = False
        command_attempted = False
        restoration_failed = False
        try:
            config = load_settings(rospy.get_param('~config'))
            b = Backend(config, rospy.get_param('~waypoints'), rospy.get_param('~allow_execution', False),
                        runner.check, handle.get_goal_id().id,
                        grasp_waypoints=rospy.get_param('~grasp2_waypoints') if grasp == 2 else None)
            with self.lock:
                self.backend = b
                runner.b = b
                # Serialize trajectory dispatch against cancellation. Existing
                # RobotSession still monitors controller, joints, faults and TCP.
                send = b.s.motion_client.send_goal
                def guarded_send(*args, **kwargs):
                    nonlocal command_attempted
                    with self.lock:
                        runner.check()
                        # A transport error can leave command delivery unknown.
                        command_attempted = True
                        return send(*args, **kwargs)
                b.s.motion_client.send_goal = guarded_send
                for publisher in (b.gripper.pick_pub, b.gripper.position_pub):
                    publish = publisher.publish
                    def guarded_publish(*args, _publish=publish, **kwargs):
                        nonlocal command_attempted
                        with self.lock:
                            runner.check()
                            command_attempted = True
                            return _publish(*args, **kwargs)
                    publisher.publish = guarded_publish
            if grasp == 'home':
                if confirm_empty:
                    runner.run_home(handle.get_goal().mode, confirm_empty=True)
                else:
                    runner.run_home(handle.get_goal().mode)
            else:
                runner.run(handle.get_goal().mode, handle.get_goal().full_transfer)
            result.success = True
            result.reason = 'planned only' if handle.get_goal().mode == 0 else 'completed; basket held at Lift' if runner.holding else 'completed'
        except Exception as error:
            cancelled = isinstance(error, Cancelled) or runner.cancel.is_set()
            result.reason = str(error)
            rospy.logwarn('fixed grasp stopped at %s: %s', runner.stage, error)
        finally:
            with self.lock:
                if self.backend:
                    try:
                        self.backend.stop()
                    except Exception as error:
                        result.success = False
                        result.reason += '; stop failed: '+str(error)
                        restoration_failed = True
                    if not command_attempted:
                        try:
                            self.backend.restore_preview()
                        except Exception as error:
                            result.success = False
                            result.reason += '; scene restoration failed: '+str(error)
                            restoration_failed = True
                if self.backend and self.backend.config.get('scene_mode', 'measured') in ('robot_only', 'existing'):
                    result.reason += '; '+self.backend.config['scene_mode']+': external environment and carried basket collisions not certified by fixed_grasp'
                result.planning_success = runner.planned
                result.execution_success = runner.executed
                result.grasp_verified = runner.grasp_verified
                result.stage = runner.stage
                # Preflight/planning errors sent no commands and can be retried.
                # Once dispatch is attempted, failed delivery/execution is unknown.
                recovery = runner.holding or restoration_failed or (command_attempted and not result.success)
                if recovery:
                    self.needs_recovery = True
                    self.recovery_stage = result.stage
                    self.recovery_reason = result.reason
                elif confirm_empty and handle.get_goal().mode == 1 and result.success:
                    self.needs_recovery = False
                    self.recovery_stage = self.recovery_reason = ''
                try:
                    if cancelled: handle.set_canceled(result)
                    elif result.success: handle.set_succeeded(result)
                    else: handle.set_aborted(result)
                finally:
                    self.active = self.backend = self.runner = None
                    self.active_grasp = None


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fixed_grasp')
    server = Server()
    rospy.spin()
