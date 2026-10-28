#!/usr/bin/env python3
"""Capture a fixed AprilTag, plan a TCP approach/touch/retract, optionally execute via ROS."""
from copy import deepcopy
from pathlib import Path
import hashlib
import math
import sys
import time

import moveit_commander
import numpy as np
import rosgraph
import rosnode
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from rm_msgs.msg import Joint_En_Flag, Manual_Set_Force_Pose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from touch_geometry import (calibration_matrix, delta, joint_calibration_matrix, mean_transform, tcp_transform,
                            touch_targets, transform)


def tf_matrix(value):
    t, q = value.transform.translation, value.transform.rotation
    return transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


def pose_message(value, frame):
    result = PoseStamped()
    result.header.frame_id = frame
    result.header.stamp = rospy.Time.now()
    result.pose.position.x, result.pose.position.y, result.pose.position.z = value[:3, 3]
    q = quaternion_from_matrix(value)
    (result.pose.orientation.x, result.pose.orientation.y,
     result.pose.orientation.z, result.pose.orientation.w) = q
    return result


def state_after(state, trajectory):
    result = deepcopy(state)
    updates = dict(zip(trajectory.joint_trajectory.joint_names,
                       trajectory.joint_trajectory.points[-1].positions))
    result.joint_state.position = [updates.get(name, position) for name, position in
                                   zip(result.joint_state.name, result.joint_state.position)]
    result.joint_state.velocity = []
    result.joint_state.effort = []
    result.joint_state.header.stamp = rospy.Time(0)
    result.is_diff = False
    return result


def check_trajectory(trajectory, start, active, travel_deg, step_deg=None):
    jt = trajectory.joint_trajectory
    if not jt.points or set(jt.joint_names) != set(active):
        raise RuntimeError('Empty trajectory or unexpected joints')
    names = dict(zip(start.joint_state.name, start.joint_state.position))
    initial = np.array([names[name] for name in jt.joint_names])
    positions = np.array([p.positions for p in jt.points])
    if positions.shape != (len(jt.points), len(jt.joint_names)) or not np.isfinite(positions).all():
        raise RuntimeError('Invalid joint trajectory positions')
    if np.max(np.abs(positions[0] - initial)) > .02:
        raise RuntimeError('Trajectory starts away from the specified robot state')
    if np.max(np.abs(positions - initial)) > math.radians(travel_deg):
        raise RuntimeError('Trajectory exceeds max_joint_travel_deg')
    if step_deg and len(positions) > 1 and np.max(np.abs(np.diff(positions, axis=0))) > math.radians(step_deg):
        raise RuntimeError('Cartesian path contains a joint jump')
    times = np.array([p.time_from_start.to_sec() for p in jt.points])
    if not np.isfinite(times).all() or times[0] < 0 or (len(times) > 1 and np.any(np.diff(times) <= 0)):
        raise RuntimeError('Trajectory has invalid timing')


class TouchTag:
    def __init__(self):
        self.execute = bool(rospy.get_param('~execute', False))
        self.base = rospy.get_param('~base_frame', 'base_link')
        self.tool = rospy.get_param('~tool_frame', 'link6')
        self.camera = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.tag = rospy.get_param('~tag_frame', 'rml63_apriltag_0')
        self.use_joint_offsets = bool(rospy.get_param('~use_joint_offsets', True))
        self.calibration_path = Path(rospy.get_param('~calibration_file')).expanduser().resolve()
        raw = self.calibration_path.read_bytes()
        data = yaml.safe_load(raw)
        self.calibration_hash = hashlib.sha256(raw).hexdigest()
        self.calibration_publisher = rospy.get_param('~calibration_publisher', '/rml63_handeye_tf').rstrip('/')
        if self.use_joint_offsets:
            self.handeye, self.corrected_tool = joint_calibration_matrix(data, self.base, self.tool, self.camera)
            rospy.loginfo('Paired joint-offset + camera calibration: %s; FK frame: %s',
                          self.calibration_path, self.corrected_tool)
        else:
            self.handeye = calibration_matrix(data, self.base, self.tool, self.camera, self.tag)
            rospy.loginfo('Legacy handeye-only calibration: %s', self.calibration_path)
        self.tcp = tcp_transform(rospy.get_param('~tcp_xyz', [0., 0., .150]),
                                 rospy.get_param('~tcp_rpy', [0., 0., 0.]))
        self.approach = self.number('approach_distance_m', .10)
        self.clearance = self.number('clearance_m', .05, allow_zero=True)
        if self.approach <= self.clearance:
            raise ValueError('approach_distance_m must be greater than clearance_m')
        self.velocity = self.number('velocity_scaling', .02)
        self.acceleration = self.number('acceleration_scaling', .02)
        if max(self.velocity, self.acceleration) > 1:
            raise ValueError('Velocity/acceleration scaling must be <= 1')
        self.age = self.number('tf_max_age_sec', .5)
        self.capture_timeout = self.number('capture_timeout_sec', 15.)
        self.sample_count = int(rospy.get_param('~capture_samples', 15))
        if self.sample_count < 3:
            raise ValueError('capture_samples must be >= 3')
        self.duration = self.number('capture_duration_sec', .6)
        self.spread = self.number('max_tag_spread_m', .004)
        self.tag_angle = self.number('max_tag_rotation_deg', 2.)
        self.max_distance = self.number('max_target_distance_m', .40)
        self.travel = self.number('max_joint_travel_deg', 90.)
        self.step = self.number('max_cartesian_joint_step_deg', 10.)
        self.eef_step = self.number('eef_step_m', .002)
        self.pos_tol = self.number('position_tolerance_m', .003)
        self.rot_tol = self.number('orientation_tolerance_deg', 1.)
        self.buffer = tf2_ros.Buffer(rospy.Duration(30))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(rospy.get_param('~planning_group', 'arm'))
        self.group.set_end_effector_link(self.tool)
        self.group.set_pose_reference_frame(self.base)
        self.group.set_max_velocity_scaling_factor(self.velocity)
        self.group.set_max_acceleration_scaling_factor(self.acceleration)
        self.group.set_goal_position_tolerance(self.pos_tol)
        self.group.set_goal_orientation_tolerance(math.radians(self.rot_tol))
        self.group.set_planning_time(8.)
        self.active = self.group.get_active_joints()
        self.executing = False
        self.display = rospy.Publisher('~display_planned_path', DisplayTrajectory, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher('~target_tcp', PoseStamped, queue_size=1, latch=True)
        self.approach_pub = rospy.Publisher('~approach_tcp', PoseStamped, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('~markers', MarkerArray, queue_size=1, latch=True)
        rospy.on_shutdown(self.stop_owned_motion)

    def stop_owned_motion(self):
        if self.executing:
            self.group.stop()

    @staticmethod
    def number(name, default, allow_zero=False):
        value = float(rospy.get_param('~' + name, default))
        if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
            raise ValueError(name + ' must be finite and ' + ('nonnegative' if allow_zero else 'positive'))
        return value

    def fresh_tool(self):
        value = self.buffer.lookup_transform(self.base, self.tool, rospy.Time(0), rospy.Duration(2))
        if not -.05 <= (rospy.Time.now() - value.header.stamp).to_sec() <= self.age:
            raise RuntimeError('Robot TF is stale')
        return tf_matrix(value)

    def verify_calibration_source(self):
        if not self.use_joint_offsets:
            return
        if hashlib.sha256(self.calibration_path.read_bytes()).hexdigest() != self.calibration_hash:
            raise RuntimeError('Calibration file changed during this run; restart publisher and preview')
        published = rospy.get_param(self.calibration_publisher + '/calibration_sha256', '')
        if published != self.calibration_hash:
            raise RuntimeError('Calibrated TF publisher has a different/unverified parameter set. '
                               'Restart roslaunch rml63_handeye publish.launch with the same calibration_file')

    def camera_at(self, stamp, nominal_tool):
        if not self.use_joint_offsets:
            return nominal_tool @ self.handeye
        # Offsets are applied ONCE by the calibrated publisher's FK(q + delta).
        # Never multiply the corrected camera extrinsic by the nominal link6 TF.
        corrected = self.buffer.lookup_transform(self.base, self.corrected_tool, stamp, rospy.Duration(.1))
        return tf_matrix(corrected) @ self.handeye

    def capture(self):
        deadline = time.monotonic() + self.capture_timeout
        after = rospy.Time.now()
        last, samples = after, []
        reason = 'Tag not visible or TF unavailable'
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                tag = self.buffer.lookup_transform(self.camera, self.tag, rospy.Time(0), rospy.Duration(0))
                stamp = tag.header.stamp
                if stamp <= last or not -.05 <= (rospy.Time.now() - stamp).to_sec() <= self.age:
                    time.sleep(.03)
                    continue
                robot = self.buffer.lookup_transform(self.base, self.tool, stamp, rospy.Duration(.1))
                g = tf_matrix(robot)
                camera = self.camera_at(stamp, g)
                world_tag = camera @ tf_matrix(tag)
                last = stamp
                if samples:
                    distance, angle = delta(samples[0][1], g)
                    if distance > .001 or angle > .3:
                        samples = []
                        reason = 'Robot moved during capture'
                samples.append((stamp, g, camera, world_tag))
                if len(samples) >= self.sample_count and (stamp - samples[0][0]).to_sec() >= self.duration:
                    average = mean_transform([item[3] for item in samples])
                    deviations = [delta(average, item[3]) for item in samples]
                    if max(x[0] for x in deviations) > self.spread or max(x[1] for x in deviations) > self.tag_angle:
                        reason = 'Tag detection is unstable'
                        samples = []
                        continue
                    return average, mean_transform([item[2] for item in samples]), g
            except tf2_ros.TransformException as error:
                reason = str(error)
            time.sleep(.02)
        raise RuntimeError('Capture timed out: ' + reason)

    def execution_checks(self):
        self.verify_calibration_source()
        if not rospy.get_param('/move_group/allow_trajectory_execution', False):
            raise RuntimeError('MoveIt trajectory execution is disabled')
        nodes = rosnode.get_node_names()
        conflicts = [n for n in nodes if n.rsplit('/', 1)[-1] in
                     ('rml63_auto_collect_joints', 'rml63_auto_calibrate', 'rml63_auto_collect')]
        if conflicts:
            raise RuntimeError('Stop the active automatic calibration motion first: ' + ', '.join(conflicts))
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        joint_publishers = dict(publishers).get('/joint_states', [])
        if joint_publishers != ['/rm_driver']:
            raise RuntimeError('Expected only /rm_driver to publish /joint_states: ' + str(joint_publishers))
        joints = rospy.wait_for_message('/joint_states', JointState, timeout=3)
        if not -.05 <= (rospy.Time.now() - joints.header.stamp).to_sec() <= self.age:
            raise RuntimeError('Measured joint states are stale')
        if not set(self.active).issubset(joints.name):
            raise RuntimeError('Measured arm joint feedback is incomplete')
        # This rm_driver publishes Manual_Set_Force_Pose on JointErrorCode.
        faults = rospy.wait_for_message('/rm_driver/JointErrorCode', Manual_Set_Force_Pose, timeout=3)
        enabled = rospy.wait_for_message('/rm_driver/Udp_Joint_En_Flag', Joint_En_Flag, timeout=3)
        if list(faults.joint) != [0] * len(self.active) or list(enabled.joint_en_flag) != [True] * len(self.active):
            raise RuntimeError('Robot joint fault or disabled joint')

    def plan_pose(self, goal, start):
        self.group.set_start_state(start)
        self.group.set_pose_target(pose_message(goal, self.base), self.tool)
        try:
            response = self.group.plan()
        finally:
            self.group.clear_pose_targets()
        success, trajectory = (response[0], response[1]) if isinstance(response, tuple) else (True, response)
        if not success:
            raise RuntimeError('Approach planning failed')
        check_trajectory(trajectory, start, self.active, self.travel)
        return trajectory

    def plan_line(self, goal, start):
        self.group.set_start_state(start)
        trajectory, fraction = self.group.compute_cartesian_path(
            [pose_message(goal, self.base).pose], self.eef_step, avoid_collisions=True)
        if not math.isfinite(fraction) or fraction < 1. - 1e-6:
            raise RuntimeError('Cartesian path incomplete (%.1f%%); nothing will execute' % (100 * fraction))
        trajectory = self.group.retime_trajectory(start, trajectory, self.velocity, self.acceleration)
        check_trajectory(trajectory, start, self.active, self.travel, self.step)
        return trajectory

    def preview(self, tcp_goals, trajectories, start):
        self.approach_pub.publish(pose_message(tcp_goals[0], self.base))
        self.target_pub.publish(pose_message(tcp_goals[1], self.base))
        message = DisplayTrajectory()
        message.trajectory_start = start
        message.trajectory = trajectories
        self.display.publish(message)
        markers = MarkerArray()
        for index, value in enumerate(tcp_goals):
            marker = Marker()
            pose = pose_message(value, self.base)
            marker.header, marker.pose = pose.header, pose.pose
            marker.ns, marker.id = 'tcp_goals', index
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = .008
            marker.color.a = 1.
            marker.color.g = 1. if index == 0 else 0.
            marker.color.r = 1. if index == 1 else 0.
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def execute_one(self, label, trajectory, expected_tcp):
        self.execution_checks()
        measured = self.robot.get_current_state().joint_state
        positions = dict(zip(measured.name, measured.position))
        jt = trajectory.joint_trajectory
        if any(abs(positions[n] - p) > .02 for n, p in zip(jt.joint_names, jt.points[0].positions)):
            raise RuntimeError('Robot moved away from the planned start; abort instead of replanning')
        rospy.loginfo('Executing %s', label)
        self.executing = True
        success = self.group.execute(trajectory, wait=True)
        self.group.stop()
        self.executing = False
        if not success:
            raise RuntimeError(label + ' execution failed; no further commands sent')
        rospy.sleep(.3)
        distance, angle = delta(self.fresh_tool() @ self.tcp, expected_tcp)
        if distance > self.pos_tol or angle > self.rot_tol:
            raise RuntimeError('%s did not reach TCP goal (%.2f mm / %.2f deg)' % (label, 1000 * distance, angle))

    def run(self):
        self.verify_calibration_source()
        if self.execute:
            self.execution_checks()
        rospy.loginfo('Keep Tag fixed and arm stationary; capturing fresh synchronized poses...')
        base_tag, base_camera, captured_tool = self.capture()
        current_tool = self.fresh_tool()
        distance, angle = delta(captured_tool, current_tool)
        if distance > .001 or angle > .3:
            raise RuntimeError('Robot moved after capture')
        tcp_goals, tool_goals, normal = touch_targets(
            base_tag, base_camera, current_tool, self.tcp, self.approach, self.clearance,
            bool(rospy.get_param('~align_tcp_z', True)))
        current_tcp = current_tool @ self.tcp
        distances = [float(np.linalg.norm(goal[:3, 3] - current_tcp[:3, 3])) for goal in tcp_goals]
        rospy.loginfo('Current TCP in %s (m): %s; approach distance %.3f m; final distance %.3f m; limit %.3f m',
                      self.base, np.round(current_tcp[:3, 3], 6), distances[0], distances[1], self.max_distance)
        rospy.loginfo('Tag center in %s (m): %s; camera-facing normal: %s', self.base,
                      np.round(base_tag[:3, 3], 6), np.round(normal, 4))
        if max(distances) > self.max_distance:
            raise RuntimeError('TCP target exceeds max_target_distance_m: approach=%.3f m, final=%.3f m, limit=%.3f m'
                               % (distances[0], distances[1], self.max_distance))
        rospy.loginfo('TCP offset (m): %s; final gap %.1f mm; execute=%s', self.tcp[:3, 3],
                      1000 * self.clearance, self.execute)
        initial = self.robot.get_current_state()
        approach = self.plan_pose(tool_goals[0], initial)
        approach_state = state_after(initial, approach)
        touch = self.plan_line(tool_goals[1], approach_state)
        plans = [approach, touch]
        final_label = 'touch center' if self.clearance == 0 else 'stop %.1f mm above center' % (1000 * self.clearance)
        labels, expected = ['approach', final_label], list(tcp_goals)
        if rospy.get_param('~retract', False):
            plans.append(self.plan_line(tool_goals[0], state_after(approach_state, touch)))
            labels.append('retract')
            expected.append(tcp_goals[0])
        self.preview(tcp_goals, plans, initial)
        rospy.loginfo('All %d segments planned. RViz DisplayTrajectory topic: %s/display_planned_path',
                      len(plans), rospy.get_name())
        if self.execute:
            rospy.sleep(self.number('preview_duration_sec', 3., allow_zero=True))
            for label, plan, tcp in zip(labels, plans, expected):
                self.execute_one(label, plan, tcp)
            rospy.loginfo('TCP motion finished; final target gap %.1f mm; retract=%s.',
                          1000 * self.clearance, rospy.get_param('~retract', False))
        else:
            rospy.loginfo('Preview only: NO motion command sent. Set execute:=true to execute.')
        if rospy.get_param('~keep_alive', True):
            rospy.spin()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('rml63_touch_tag')
    node = None
    try:
        node = TouchTag()
        node.run()
        return 0
    except (Exception, KeyboardInterrupt) as error:
        rospy.logerr('Touch stopped: %s', error)
        return 1
    finally:
        # A planning-only process must never stop another process's robot motion.
        if node is not None and node.executing:
            node.stop_owned_motion()
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    sys.exit(main())
