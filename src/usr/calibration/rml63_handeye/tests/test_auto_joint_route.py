#!/usr/bin/env python3
"""Regression for opposite targets individually safe but unsafe when joined directly."""
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock
import numpy as np
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from auto_collect_joints import checked_trajectory, trajectory_end_state, JointAutomaticCalibration


def state(degrees):
    result = RobotState()
    result.joint_state.name = ['joint1', 'joint2']
    result.joint_state.position = [np.radians(degrees), .3]
    return result


def trajectory(*degrees):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ['joint1']
    for value in degrees:
        point = JointTrajectoryPoint()
        point.positions = [np.radians(value)]
        result.joint_trajectory.points.append(point)
    return result


class RouteRegression(unittest.TestCase):
    def test_opposite_goals_rejected_directly_but_center_legs_pass(self):
        checked_trajectory((True, trajectory(0, 20), 0, None), state(0), 'out')
        checked_trajectory((True, trajectory(20, 0), 0, None), state(20), 'return')
        checked_trajectory((True, trajectory(0, -20), 0, None), state(0), 'out')
        with self.assertRaisesRegex(RuntimeError, '40.00 degrees'):
            checked_trajectory((True, trajectory(20, -20), 0, None), state(20), 'unsafe direct')

    def test_current_state_is_included_even_if_planner_omits_start_point(self):
        with self.assertRaisesRegex(RuntimeError, '35.00 degrees'):
            checked_trajectory((True, trajectory(20, 25), 0, None), state(-10), 'omitted start')

    def test_virtual_endpoint_keeps_unplanned_joints_and_original_state(self):
        original = state(0)
        end = trajectory_end_state(original, trajectory(20))
        self.assertAlmostEqual(end.joint_state.position[0], np.radians(20))
        self.assertEqual(end.joint_state.position[1], .3)
        self.assertEqual(original.joint_state.position[0], 0)

    def test_runtime_joint_goal_does_not_reselect_a_cartesian_ik_branch(self):
        runner = JointAutomaticCalibration.__new__(JointAutomaticCalibration)
        runner.group = MagicMock()
        runner.group.plan.return_value = (True, trajectory(0, 20), 0, None)
        runner.plan_from_state({'joint1': np.radians(20)}, state(0), 'fixed branch', joint_target=True)
        runner.group.set_joint_value_target.assert_called_once()
        runner.group.set_pose_target.assert_not_called()
        runner.group.set_start_state_to_current_state.assert_called_once()


if __name__ == '__main__':
    unittest.main()
