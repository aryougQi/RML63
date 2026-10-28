#!/usr/bin/env python3
"""Run upstream easy_handeye's pose pattern with checked execution and sampling.

Default mode plans only. Explicit execute + measured-tag flags are required for motion.
The calibration services and pose generation come from the cloned easy_handeye repository.
"""
import math
from pathlib import Path
import time

import moveit_commander
import numpy as np
import rospy
import rostopic
import tf2_ros
import yaml
from easy_handeye.handeye_robot import CalibrationMovements
from easy_handeye_msgs.srv import ComputeCalibration, TakeSample
from std_srvs.srv import Empty
from tf.transformations import quaternion_matrix


def tf_arrays(value):
    t = value.transform.translation
    q = value.transform.rotation
    return np.array([t.x, t.y, t.z]), quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]


def angle_between(a, b):
    return math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))


class AutomaticCalibration:
    def __init__(self):
        self.execute = bool(rospy.get_param('~execute', False))
        self.namespace = rospy.get_param('~calibration_namespace', '/rml63_l515_eye_on_hand').rstrip('/')
        self.marker = rospy.get_param('~tag_frame', 'rml63_apriltag_0')
        self.tf_max_age = float(rospy.get_param('~tf_max_age_sec', .8))
        self.tf_wait_timeout = float(rospy.get_param('~tf_wait_timeout_sec', 5.0))
        self.settle_timeout = float(rospy.get_param('~settle_timeout_sec', 10.0))
        if min(self.tf_max_age, self.tf_wait_timeout, self.settle_timeout) <= 0:
            raise ValueError('TF and settling timeouts must be positive.')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_end_effector_link('link6')
        self.group.set_pose_reference_frame('base_link')
        self.group.set_planner_id('RRTConnect')
        self.group.set_max_velocity_scaling_factor(.05)
        self.group.set_max_acceleration_scaling_factor(.05)
        self.group.set_planning_time(5.0)
        self.active = self.group.get_active_joints()
        if self.execute:
            if not rospy.get_param('~tag_size_verified', False):
                raise RuntimeError('Set tag_size_verified:=true only after measuring the printed black square.')
            if not rospy.get_param('/move_group/allow_trajectory_execution', False):
                raise RuntimeError('MoveIt execution is disabled.')
        self.output = Path(rospy.get_param('~output_dir', '~/.ros/rml63_handeye_runs')).expanduser() / time.strftime('%Y%m%d_%H%M%S')
        self.output.mkdir(parents=True, exist_ok=True)

    def fresh_tf(self, parent, child, newer_than=None, deadline=None):
        deadline = min(deadline or float('inf'), time.monotonic()+self.tf_wait_timeout)
        reason = 'no transform received'
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                value = self.buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0))
                age = (rospy.Time.now()-value.header.stamp).to_sec()
                advances = newer_than is None or value.header.stamp > newer_than
                if -.05 <= age <= self.tf_max_age and advances:
                    return value
                reason = 'age=%.3fs (limit %.3fs), new frame=%s' % (age, self.tf_max_age, advances)
            except tf2_ros.TransformException as error:
                reason = str(error)
            rospy.sleep(.03)
        raise RuntimeError('Timed out waiting for fresh TF %s -> %s: %s' % (parent, child, reason))

    def stable(self):
        deadline = time.monotonic()+self.settle_timeout
        # Never use an exposure from before this settling check (including in-flight frames).
        after = rospy.Time.now()
        previous_tag = self.fresh_tf('camera_color_optical_frame', self.marker, after, deadline)
        previous_robot = self.fresh_tf('base_link', 'link6', deadline=deadline)
        consecutive = 0
        # Require three distinct, advancing camera frames and a stationary robot.
        while consecutive < 3:
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for robot and AprilTag to settle; no sample taken.')
            rospy.sleep(.3)
            tag = self.fresh_tf('camera_color_optical_frame', self.marker, previous_tag.header.stamp, deadline)
            robot = self.fresh_tf('base_link', 'link6', previous_robot.header.stamp, deadline)
            stationary = True
            for old, new, max_t, max_r in [(previous_robot, robot, .001, .3), (previous_tag, tag, .003, 1.0)]:
                a, ar = tf_arrays(old)
                b, br = tf_arrays(new)
                if np.linalg.norm(a-b) > max_t or angle_between(ar, br) > math.radians(max_r):
                    stationary = False
            consecutive = consecutive+1 if stationary else 0
            previous_robot, previous_tag = robot, tag

    def check_faults(self):
        for name, field, expected in [('/rm_driver/JointErrorCode', 'joint', [0]*6),
                                      ('/rm_driver/Udp_Joint_En_Flag', 'joint_en_flag', [True]*6)]:
            cls, _, _ = rostopic.get_topic_class(name, blocking=False)
            if cls is None:
                raise RuntimeError('Missing robot status: '+name)
            msg = rospy.wait_for_message(name, cls, timeout=3)
            if list(getattr(msg, field)) != expected:
                raise RuntimeError('Robot fault or disabled joint: '+str(msg))

    def plan(self, target):
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(target, 'link6')
        response = self.group.plan()
        success, trajectory = (response[0], response[1]) if isinstance(response, tuple) else (True, response)
        self.group.clear_pose_targets()
        if not success or not trajectory.joint_trajectory.points:
            raise RuntimeError('Planning failed; run aborted.')
        points = np.array([p.positions for p in trajectory.joint_trajectory.points])
        if np.max(np.ptp(points, axis=0)) > math.radians(30):
            raise RuntimeError('A joint would move over 30 degrees; reject this automatic sample pose.')
        return trajectory

    def run(self):
        self.check_faults()
        self.stable()
        start = self.group.get_current_pose('link6')
        rotation = math.radians(rospy.get_param('~rotation_delta_degrees', 10.0))
        translation = rospy.get_param('~translation_delta_meters', .02)
        targets = CalibrationMovements._compute_poses_around_state(start, rotation, translation)
        for i, target in enumerate(targets):
            self.plan(target)
            rospy.loginfo('Preflight plan %d/%d OK (no motion)', i+1, len(targets))
        if not self.execute:
            rospy.loginfo('Planning-only check completed. No robot movement or samples were commanded.')
            return
        # Use the upstream services, but never implicitly erase or mix an earlier run.
        for service in ['get_sample_list', 'take_sample', 'compute_calibration', 'save_calibration']:
            rospy.wait_for_service(self.namespace+'/'+service, timeout=10)
        get_samples = rospy.ServiceProxy(self.namespace+'/get_sample_list', TakeSample)
        take = rospy.ServiceProxy(self.namespace+'/take_sample', TakeSample)
        if get_samples().samples.hand_world_samples:
            raise RuntimeError('Calibration server already has samples. Restart it before a new automatic run.')
        for i, target in enumerate(targets):
            self.check_faults()
            self.stable()
            plan = self.plan(target)
            if not self.group.execute(plan, wait=True):
                raise RuntimeError('MoveIt execution failed. No sample taken.')
            self.group.stop()
            expected = dict(zip(plan.joint_trajectory.joint_names, plan.joint_trajectory.points[-1].positions))
            actual = dict(zip(self.active, self.group.get_current_joint_values()))
            if any(abs(actual[name]-value) > .02 for name, value in expected.items()):
                raise RuntimeError('Measured joints did not reach the goal despite controller status. No sample taken.')
            self.check_faults()
            self.stable()
            before = len(get_samples().samples.hand_world_samples)
            samples = take().samples
            if len(samples.hand_world_samples) != before+1:
                raise RuntimeError('easy_handeye did not append a sample.')
            (self.output / 'samples.yaml').write_text(str(samples))
            rospy.loginfo('Sample %d/%d saved', i+1, len(targets))
        compute = rospy.ServiceProxy(self.namespace+'/compute_calibration', ComputeCalibration)()
        if not compute.valid:
            raise RuntimeError('Solver rejected the calibration; raw samples remain on disk.')
        rospy.ServiceProxy(self.namespace+'/save_calibration', Empty)()
        (self.output / 'calibration.yaml').write_text(str(compute.calibration))
        rospy.loginfo('Calibration computed and saved. Validate with independent poses before using for grasping. Robot stays at the final sample pose.')


if __name__ == '__main__':
    rospy.init_node('rml63_auto_calibrate')
    runner = None
    try:
        runner = AutomaticCalibration()
        runner.run()
    except Exception as error:
        if runner is not None:
            if runner.execute:
                runner.group.stop()
            runner.group.clear_pose_targets()
        rospy.logerr('Automatic calibration stopped: %s', error)
        raise SystemExit(1)
