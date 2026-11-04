#!/usr/bin/env python3
"""MoveIt return to taught Home joints only; explicit --execute required."""
import argparse
import fcntl
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parent))
import moveit_commander
import rospkg
import rospy
from backend import Backend, load_settings
from client import send_task


def return_home(backend, execute):
    backend.preflight(execute, False, home_only=True)
    if backend.config.get('scene_mode', 'measured') == 'robot_only':
        print('robot_only：未验证外部环境及篮体碰撞，仅检查现有机器人/工具模型与场景。', flush=True)
    elif backend.config.get('scene_mode') == 'existing':
        print('existing：直接使用 MoveIt 已有模型和场景，不新增或修改碰撞物体。', flush=True)
    print('正在规划到已示教 Home 的关节姿态……', flush=True)
    plan = backend.plan(['Home'], backend.check)['Home']
    if not execute:
        print('Home 规划通过，未执行运动。实际执行需显式添加 --execute。', flush=True)
        return
    backend.check()
    backend.preflight(True, False, home_only=True)
    print('开始执行 Home 轨迹……', flush=True)
    backend.move('Home', plan)
    print('已到达 Home，通过控制器和实际 TCP 检查。', flush=True)


def main():
    defaults = Path(rospkg.RosPack().get_path('fixed_grasp'))/'config'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(defaults/'fixed.yaml'))
    parser.add_argument('--waypoints', default=str(defaults/'waypoints.yaml'))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--timing-report', type=Path, help='保存各阶段耗时 JSON')
    args = parser.parse_args(rospy.myargv()[1:])
    # Same lease as the Action server; do not run two independent controllers.
    with open('/tmp/fixed_grasp_robot.lock', 'a') as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            if (Path(args.config).resolve() != (defaults/'fixed.yaml').resolve() or
                    Path(args.waypoints).resolve() != (defaults/'waypoints.yaml').resolve() or args.timing_report):
                raise RuntimeError('服务运行时回 Home 请使用 client.py --home；'
                                   '配置和示教文件由服务启动参数指定，--timing-report 仅用于独立模式')
            rospy.init_node('fixed_grasp_home_client', anonymous=True, disable_signals=True)
            print('通过运行中的 fixed_grasp 服务请求回 Home。', flush=True)
            result = send_task(execute=args.execute, home=True)
            if result is None or not result.success:
                raise RuntimeError('Home request failed: '+(result.reason if result else 'no action result'))
            return
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('fixed_grasp_return_home', anonymous=True)
        def check():
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException('return Home cancelled')
        backend = None
        started = time.monotonic()
        completed = False
        try:
            backend = Backend(load_settings(args.config), args.waypoints, args.execute, check, 'return_home')
            return_home(backend, args.execute)
            completed = True
        finally:
            if backend:
                backend.stop()
                if not args.execute:
                    original_error = sys.exc_info()[0] is not None
                    try:
                        backend.restore_preview()
                    except Exception as error:
                        if not original_error:
                            raise
                        rospy.logwarn('Home preview cleanup also failed; preserving original error: %s', error)
                if args.timing_report:
                    report = backend.s.metrics.snapshot('success' if completed else 'failed')
                    report.update(total_wall_sec=time.monotonic()-started,
                                  execute_requested=args.execute,
                                  start_joints_rad=getattr(backend, 'plan_start_joints_rad', None))
                    args.timing_report.parent.mkdir(parents=True, exist_ok=True)
                    args.timing_report.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
                    print('耗时报告：'+str(args.timing_report), flush=True)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        print('已取消回 Home，不执行后续动作。')
    except Exception as error:
        print('回 Home 已停止：'+str(error), file=sys.stderr)
        sys.exit(1)
