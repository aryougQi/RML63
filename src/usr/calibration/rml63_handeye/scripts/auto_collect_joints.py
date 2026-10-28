#!/usr/bin/env python3
"""Opt-in automatic motion with synchronized joint/vision data and offline zero-offset fitting."""
from copy import deepcopy
from pathlib import Path
import math
import subprocess
import sys

import numpy as np
import rospy
import yaml
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_matrix

sys.path.insert(0, str(Path(__file__).resolve().parent))
from auto_calibrate import AutomaticCalibration
from collect_joint_calibration import JointHistory, CameraSnapshot, capture_joint_sample, ensure_detection_view
from joint_calibration_model import JOINT_NAMES, RobotChain
from validate_calibration import load_calibration
from easy_handeye.handeye_robot import CalibrationMovements


def targets_around(start, angle, distance):
    targets = CalibrationMovements._compute_poses_around_state(start, angle, distance)
    ori = start.pose.orientation
    base = quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
    for a, b in [(0, 1), (0, 2), (1, 2)]:
        for sa, sb in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
            angles = np.zeros(3)
            angles[a], angles[b] = sa*angle/math.sqrt(2), sb*angle/math.sqrt(2)
            target = deepcopy(start)
            q = quaternion_from_matrix(base @ euler_matrix(*angles))
            target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = q
            targets.append(target)
    for sx, sy, sz in [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]:
        target = deepcopy(start)
        target.pose.position.x += sx*distance/2
        target.pose.position.y += sy*distance/2
        target.pose.position.z += sz*distance/2
        targets.append(target)
    return targets


def checked_trajectory(response, start_state, label):
    success, trajectory = (response[0], response[1]) if isinstance(response, tuple) else (True, response)
    if not success or not trajectory.joint_trajectory.points:
        raise RuntimeError(label + ': planning failed')
    names = trajectory.joint_trajectory.joint_names
    start = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
    points = np.asarray([[start[n] for n in names]] + [list(p.positions) for p in trajectory.joint_trajectory.points])
    if not np.isfinite(points).all():
        raise RuntimeError(label + ': invalid joint trajectory')
    spans = np.degrees(np.ptp(points, axis=0))
    if np.max(spans) > 30.000001:
        i = int(np.argmax(spans))
        raise RuntimeError('%s: %s trajectory span %.2f degrees exceeds 30-degree limit' % (label, names[i], spans[i]))
    return trajectory


def trajectory_end_state(start_state, trajectory):
    state = deepcopy(start_state)
    angles = dict(zip(state.joint_state.name, state.joint_state.position))
    angles.update(zip(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[-1].positions))
    state.joint_state.name = list(angles)
    state.joint_state.position = list(angles.values())
    state.joint_state.velocity = []
    state.joint_state.effort = []
    return state


class JointAutomaticCalibration(AutomaticCalibration):
    def __init__(self):
        super().__init__()
        seed = Path(rospy.get_param('~initial_calibration', '~/.ros/easy_handeye/rml63_l515_manual_eye_on_hand.yaml')).expanduser()
        calibration, self.initial_x = load_calibration(seed)
        self.frames = [calibration['parameters'][k] for k in
                       ('robot_base_frame', 'robot_effector_frame', 'tracking_base_frame', 'tracking_marker_frame')]
        if self.frames != ['base_link', 'link6', 'camera_color_optical_frame', self.marker]:
            raise ValueError('Initial calibration frames do not match automatic collector')
        self.xml = rospy.get_param('/robot_description')
        self.chain = RobotChain(self.xml)
        self.history, self.camera = JointHistory(), CameraSnapshot()
        self.data = {'schema': 1, 'role': 'train', 'joint_names': JOINT_NAMES, 'frames': self.frames,
                     'initial_calibration': calibration, 'samples': []}

    def plan_from_state(self, target, state, label, joint_target=False):
        error = None
        try:
            for _ in range(3):
                self.group.clear_pose_targets()
                self.group.set_start_state(state)
                if joint_target:
                    self.group.set_joint_value_target(target)
                else:
                    self.group.set_pose_target(target, 'link6')
                try:
                    return checked_trajectory(self.group.plan(), state, label)
                except RuntimeError as exc:
                    error = exc
            raise error
        finally:
            self.group.clear_pose_targets()
            self.group.set_start_state_to_current_state()

    def execute_joint_goal(self, goal, label):
        self.check_faults()
        plan = self.plan_from_state(goal, self.group.get_current_state(), label, joint_target=True)
        if not self.group.execute(plan, wait=True):
            raise RuntimeError(label + ': trajectory execution failed')
        self.group.stop()
        actual = dict(zip(self.active, self.group.get_current_joint_values()))
        if any(abs(actual[name]-value) > .02 for name, value in goal.items()):
            raise RuntimeError(label + ': measured joints did not reach the target')
        self.check_faults()

    def run(self):
        self.check_faults()
        self.stable()
        angle = math.radians(rospy.get_param('~rotation_delta_degrees', 20.))
        distance = float(rospy.get_param('~translation_delta_meters', .04))
        if not 0 < angle <= math.radians(30) or not 0 < distance <= .1:
            raise ValueError('Use rotation in (0,30] degrees and translation in (0,0.1] metres')
        center_state = self.group.get_current_state()
        center_angles = dict(zip(center_state.joint_state.name, center_state.joint_state.position))
        center_goal = {name: center_angles[name] for name in self.active}
        targets = targets_around(self.group.get_current_pose('link6'), angle, distance)
        goals = []
        for i, target in enumerate(targets):
            # Fix the IK branch at the center, then check both legs of the actual route.
            outbound = self.plan_from_state(target, center_state, 'Center -> pose %d' % (i+1))
            end_state = trajectory_end_state(center_state, outbound)
            end_angles = dict(zip(end_state.joint_state.name, end_state.joint_state.position))
            goal = {name: end_angles[name] for name in self.active}
            self.plan_from_state(center_goal, end_state, 'Pose %d -> center' % (i+1), joint_target=True)
            goals.append(goal)
            rospy.loginfo('Joint-data preflight %d/%d outbound + return OK (no motion)', i+1, len(targets))
        (self.output/'preflight_plan.yaml').write_text(yaml.safe_dump({
            'route': 'center -> pose -> center -> next pose', 'center_joint_goal_rad': center_goal,
            'sample_joint_goals_rad': goals, 'max_joint_span_deg': 30}, sort_keys=False))
        if not self.execute:
            rospy.loginfo('Planning only. No motion or samples. Check Tag visibility separately; planning cannot predict it.')
            return
        if (self.output/'dataset.yaml').exists():
            raise RuntimeError('Output already contains a dataset; refusing to overwrite it')
        self.data['tag_config'] = rospy.get_param('/rml63_apriltag/detector/standalone_tags')
        (self.output/'robot.urdf').write_text(self.xml)
        current = dict(zip(self.active, self.group.get_current_joint_values()))
        if any(abs(current[name]-value) > .02 for name, value in center_goal.items()):
            raise RuntimeError('Robot moved during preflight; restart from the new current pose')
        self.data['preflight_center_joint_goal_rad'] = center_goal
        self.data['route'] = 'center -> pose -> center -> next pose'
        for i, goal in enumerate(goals):
            if i:
                rospy.loginfo('Return to sampling center before pose %d/%d', i+1, len(targets))
                self.execute_joint_goal(center_goal, 'Return to center')
            self.execute_joint_goal(goal, 'Execute pose %d' % (i+1))
            self.stable()
            sample = capture_joint_sample(self.buffer, self.frames, self.initial_x, self.chain,
                                           self.history, self.camera, self.output, i+1)
            self.data['samples'].append(sample)
            temporary = self.output/'dataset.yaml.tmp'
            temporary.write_text(yaml.safe_dump(self.data, sort_keys=False))
            temporary.replace(self.output/'dataset.yaml')
            rospy.loginfo('Synchronized joint/Tag pose %d/%d saved', i+1, len(targets))
        rospy.loginfo('Dataset: %s; robot stays at last pose', self.output/'dataset.yaml')
        if rospy.get_param('~optimize_after_collection', True):
            subprocess.run([sys.executable, str(Path(__file__).resolve().parent/'optimize_joint_calibration.py'),
                            str(self.output/'dataset.yaml')], check=True)
            rospy.loginfo('Offline fitting complete, no offsets applied. New independent validation poses still required.')


if __name__ == '__main__':
    rospy.init_node('rml63_auto_collect_joints')
    runner = None
    try:
        view = ensure_detection_view()
        if view is not None:
            rospy.on_shutdown(lambda: view.terminate() if view.poll() is None else None)
        runner = JointAutomaticCalibration()
        runner.run()
    except Exception as error:
        rospy.logerr('Automatic joint calibration stopped: %s', error)
        sys.exit(1)
    finally:
        if runner is not None:
            if runner.execute:
                runner.group.stop()
            runner.group.clear_pose_targets()
        rospy.signal_shutdown('Automatic joint data collection finished')
