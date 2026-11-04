#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import actionlib
import rospy
from fixed_grasp.msg import FixedGraspAction, FixedGraspGoal
from grasp_profiles import action_name


def send_task(execute=False, full_transfer=False, grasp=1, home=False, confirm_empty=False):
    if confirm_empty and not home:
        raise ValueError('confirm_empty is only supported for Home')
    endpoint = ('/fixed_grasp/home_empty' if confirm_empty else '/fixed_grasp/home') if home else action_name(grasp)
    client = actionlib.SimpleActionClient(endpoint, FixedGraspAction)
    if not client.wait_for_server(rospy.Duration(10)):
        raise RuntimeError('fixed grasp server unavailable: '+endpoint)
    client.send_goal(FixedGraspGoal(mode=1 if execute else 0, full_transfer=full_transfer),
                     feedback_cb=lambda f: print('%s holding=%s %s'%(f.stage, f.holding, f.detail), flush=True))
    try:
        while not client.wait_for_result(rospy.Duration(.2)):
            if rospy.is_shutdown():
                client.cancel_goal()
                raise RuntimeError('ROS shut down while waiting for task result; cancellation requested')
    except KeyboardInterrupt:
        client.cancel_goal()
        print('Cancel requested; waiting for server acknowledgement', flush=True)
        if not client.wait_for_result(rospy.Duration(10)):
            raise RuntimeError('cancel acknowledgement timed out; check server and robot state')
    print('Action status:', client.get_state())
    print(client.get_result())
    return client.get_result()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--full-transfer', action='store_true')
    parser.add_argument('--confirm-empty', action='store_true',
                        help='人工确认夹爪空载且机械臂停止；仅回 Home，允许夹爪未初始化，成功后清除恢复锁')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--grasp', type=int, choices=(1, 2), default=1,
                           help='抓取位姿编号；两套位姿共用放置位置')
    selection.add_argument('--home', action='store_true', help='通过现有服务只回 Home，不运行抓取流程')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.home and args.full_transfer:
        parser.error('--home 不能与 --full-transfer 同时使用')
    if args.confirm_empty and not args.home:
        parser.error('--confirm-empty 只能与 --home 同时使用')
    rospy.init_node('fixed_grasp_client', anonymous=True, disable_signals=True)
    options = {'confirm_empty': True} if args.confirm_empty else {}
    result = send_task(args.execute, args.full_transfer, args.grasp, args.home, **options)
    return 0 if result is not None and result.success else 1


if __name__ == '__main__':
    raise SystemExit(main())
