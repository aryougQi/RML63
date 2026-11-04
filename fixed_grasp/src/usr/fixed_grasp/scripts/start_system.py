#!/usr/bin/env python3
"""Start the real ROS stack and initialize an operator-confirmed empty gripper."""
import argparse
import fcntl
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import actionlib
import rosgraph
import roslaunch
import rospkg
import rospy
from std_msgs.msg import Byte
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from fixed_grasp.msg import FixedGraspAction
from backend import Gripper, load_settings


def initialize_gripper(config, check):
    """Explicit startup operation: power 24V, open, and verify actual feedback."""
    gripper = Gripper(config, check)
    state = gripper.query('get_gripper_state')
    force = state.get('current_force')
    if (state.get('error', 0) or state.get('mode') in (4, 5, 6) or
            not isinstance(force, (int, float)) or not abs(force) < 50):
        raise RuntimeError('夹爪存在受力、运动、故障或无效反馈，请先人工处理：'+str(state))
    publisher = rospy.Publisher('/rm_driver/SetToolVoltage', Byte, queue_size=1)
    try:
        gripper._connected(publisher)
        check()
        publisher.publish(Byte(data=3))
        deadline = time.monotonic()+10.
        while True:
            check()
            voltage = gripper.query('get_tool_voltage')
            if voltage.get('voltage_type') == 3:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('末端 24V 上电未确认：'+str(voltage))
            rospy.sleep(.2)
        rospy.loginfo('末端 24V 已确认；正在将空载夹爪打开到 %.1f mm', config['open_width_m']*1000)
        # No require_open() before this command: startup feedback may be all zero.
        # The existing open() checks power and verifies enabled, idle, low-force,
        # stable opening feedback after commanding the requested position.
        check()
        result = gripper.open(config['open_width_m'])
        check()
        rospy.loginfo('夹爪上电/开爪验证成功：%s', result)
        return result
    finally:
        publisher.unregister()
        gripper.position_pub.unregister()
        gripper.pick_pub.unregister()


def wait_for_stack(check, timeout=60.):
    deadline = time.monotonic()+timeout
    while True:
        check()
        try:
            pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            if dict(pubs).get('/joint_states') == ['/rm_driver']:
                message = rospy.wait_for_message('/joint_states', JointState, timeout=1.)
                age = (rospy.Time.now()-message.header.stamp).to_sec()
                if -.05 <= age <= .3:
                    rospy.wait_for_service('/check_state_validity', timeout=1.)
                    controller = actionlib.SimpleActionClient('/rm_group/follow_joint_trajectory', FollowJointTrajectoryAction)
                    if controller.wait_for_server(rospy.Duration(1.)):
                        return
        except (rospy.ROSException, OSError):
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError('等待真实驱动、关节反馈、MoveIt 或轨迹控制器超时；请检查启动日志和机器人网络')
        rospy.sleep(.2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-empty', action='store_true',
                        help='确认夹爪空载且开合区域无障碍；启动会开启末端24V并实际打开夹爪')
    parser.add_argument('--no-rviz', action='store_true')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--m20', action='store_true', help='启用机械狗place接口：第一次新事务放1，第二次放2')
    mode.add_argument('--m20-test', action='store_true', help='不连机械狗的手动真机测试；使用独立话题和测试计数')
    parser.add_argument('--udp-ip', default='192.168.1.10')
    args = parser.parse_args(argv)
    if not args.confirm_empty:
        parser.error('启动会实际上电并打开夹爪，请确认空载后添加 --confirm-empty')
    pack = rospkg.RosPack()
    config = load_settings(Path(pack.get_path('fixed_grasp'))/'config/fixed.yaml')
    launch_files = []
    # Serialize startup operations and exclude an already running task server.
    with open('/tmp/fixed_grasp_startup.lock', 'a') as startup_lease, \
            open('/tmp/fixed_grasp_robot.lock', 'a') as control_lease:
        try:
            fcntl.flock(startup_lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(control_lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有固定抓取服务或一键启动脚本运行；请先在旧终端 Ctrl+C，避免并行控制')
        if rosgraph.is_master_online():
            pubs, subs, services = rosgraph.Master('/fixed_grasp_startup_check').getSystemState()
            nodes = {node for entries in (pubs, subs, services) for _, owners in entries for node in owners}
            if '/rm_driver' in nodes or '/move_group' in nodes:
                raise RuntimeError('已有驱动或 MoveIt 运行；请先在旧启动终端 Ctrl+C，再用一键脚本启动')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        try:
            robot_launch = str(Path(pack.get_path('rml63_tools_description'))/'launch/realrobot.launch')
            robot = roslaunch.parent.ROSLaunchParent(uuid, [(robot_launch, [
                'allow_execution:=true', 'Arm_IP:='+config['robot_ip'], 'Udp_IP:='+args.udp_ip,
                'use_rviz:='+('false' if args.no_rviz else 'true')])])
            launch_files.append(robot)
            robot.start()  # roslaunch starts roscore when no master is running.
            rospy.init_node('fixed_grasp_startup', anonymous=True, disable_signals=True)

            def check():
                robot.spin_once()
                if rospy.is_shutdown() or robot.pm.is_shutdown:
                    raise RuntimeError('机器人启动已停止')

            rospy.loginfo('等待驱动、MoveIt 和真实关节反馈……')
            wait_for_stack(check)
            initialize_gripper(config, check)
            # Hand control to the shared action server only after initialization.
            fcntl.flock(control_lease, fcntl.LOCK_UN)
            task_args = ['allow_execution:=true']
            if args.m20 or args.m20_test:
                task_args.append('m20_enabled:=true')
            if args.m20_test:
                task_args.append('m20_test_mode:=true')
            task = roslaunch.parent.ROSLaunchParent(uuid, [(str(Path(pack.get_path('fixed_grasp'))/
                'launch/fixed_grasp.launch'), task_args)])
            launch_files.append(task)
            task.start()
            client = actionlib.SimpleActionClient('/fixed_grasp/home', FixedGraspAction)
            if not client.wait_for_server(rospy.Duration(20.)):
                raise RuntimeError('固定抓取服务启动超时')
            print('\n启动完成：ROS、驱动、MoveIt、夹爪和固定抓取服务已就绪。', flush=True)
            if args.m20_test:
                print('手动真机测试模式：另开终端运行 m20_manual_test.py place --execute；每次搬一块，先1后2。', flush=True)
            elif args.m20:
                print('M20模式：查看 /m20_cooperation/arm/status，ready=true 后等待机械狗的 place；两次新事务依次放1、2。', flush=True)
            else:
                print('另开终端运行 client.py。', flush=True)
            print('本终端保持运行；Ctrl+C 关闭本脚本启动的节点。', flush=True)
            while not rospy.is_shutdown():
                check()
                task.spin_once()
                if task.pm.is_shutdown:
                    raise RuntimeError('固定抓取服务已停止')
                rospy.sleep(.2)
        finally:
            for parent in reversed(launch_files):
                try:
                    parent.shutdown()
                except Exception as error:
                    print('关闭启动节点时发生异常：'+str(error), file=sys.stderr)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\n已停止一键启动。')
        raise SystemExit(130)
    except Exception as error:
        print('启动失败：'+str(error), file=sys.stderr)
        raise SystemExit(1)
