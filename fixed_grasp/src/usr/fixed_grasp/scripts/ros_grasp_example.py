#!/usr/bin/env python3
"""Example for another ROS program: execute one full transfer and wait for success."""
import argparse

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from fixed_grasp.msg import FixedGraspAction, FixedGraspGoal


def execute_grasp(grasp):
    """Call after rospy.init_node; return only after placing, retreating and returning Home."""
    if grasp not in (1, 2):
        raise ValueError('grasp must be 1 or 2')
    endpoint = '/fixed_grasp/task' if grasp == 1 else '/fixed_grasp/task_2'
    client = actionlib.SimpleActionClient(endpoint, FixedGraspAction)
    if not client.wait_for_server(rospy.Duration(10)):
        raise RuntimeError('grasp action unavailable: '+endpoint)
    goal = FixedGraspGoal(mode=FixedGraspGoal.EXECUTE, full_transfer=True)
    client.send_goal(goal)
    try:
        while not rospy.is_shutdown():
            if client.wait_for_result(rospy.Duration(.2)):
                break
        else:
            raise rospy.ROSInterruptException('ROS shutting down')
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        client.cancel_goal()
        raise
    result = client.get_result()
    if not (client.get_state() == GoalStatus.SUCCEEDED and result is not None and
            result.success and result.planning_success and result.execution_success and
            result.grasp_verified and result.stage == 'DONE'):
        raise RuntimeError('grasp %d failed: %s' % (grasp, result.reason if result else 'no result'))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grasp', type=int, choices=(1, 2), required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('external_grasp_program', anonymous=True, disable_signals=True)
    try:
        execute_grasp(args.grasp)
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.logwarn('抓取已请求取消，不启动后续程序')
        return 1
    except RuntimeError as error:
        rospy.logerr(str(error))
        return 1
    # Put the application's next step here; this point is reached only on success.
    rospy.loginfo('方块 %d 完整搬运成功，已回 Home，可以启动后续程序', args.grasp)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
