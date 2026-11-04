#!/usr/bin/env python3
"""交互记录实际关节角和标定 TCP；不发送运动或夹爪命令。"""
import argparse
from pathlib import Path
import os
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import time
import numpy as np
import yaml
import rospy
import rospkg
import moveit_commander
from backend import Session, geometry, identity, load_settings
from grasp_profiles import read_shared, new_grasp_profile, validate_profile_reference
from waypoint_offsets import derive_from_grasp

NAMES = ['Home', 'Pregrasp', 'Grasp', 'Lift', 'Preplace', 'Place', 'Retreat']
HINTS = {
    'Home': '初始安全位置',
    'Pregrasp': '预抓取位置，与 Grasp 保持相同姿态',
    'Grasp': '抓取位置，夹爪对准目标物体',
    'Lift': '从 Grasp 沿基座 +Z 抬升到所需位置，保持姿态',
    'Preplace': '放置预备位置',
    'Place': '最终放置位置，篮子应得到可靠支撑',
    'Retreat': '安全撤离位置，与 Place 保持相同姿态',
}


def capture(session, config, tcp):
    """Only acquire feedback after confirmation, preserving original checks."""
    session.verify_hardware(require_execution=False)
    samples, stamps = [], []
    for _ in range(5):
        q, stamp = session.measured()
        samples.append(q); stamps.append(stamp.to_nsec())
        time.sleep(.1)
    if len(set(stamps)) != len(stamps) or np.max(np.ptp(samples, axis=0)) > np.radians(config['max_capture_motion_deg']):
        raise RuntimeError('fresh stationary joint feedback required')
    q = samples[-1]
    return dict(joints_rad=q.tolist(), tcp_base=(session.chain.fk(q, session.offsets)@tcp).tolist(),
                joint_stamp_ns=stamps[-1], recorded_unix=time.time(), stationary_samples=5)


def save_pose(output, data, name, record):
    """Save each confirmed pose immediately; update memory after atomic write."""
    updated = dict(data, waypoints=dict(data['waypoints'], **{name: record}))
    save_data(output, data, updated)


def save_data(output, data, updated):
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix+'.tmp')
    temporary.write_text(yaml.safe_dump(updated, sort_keys=False))
    os.replace(str(temporary), str(output))
    data.clear()
    data.update(updated)


def save_second_grasp(output, data, record):
    updated = derive_from_grasp(dict(data, waypoints={'Grasp': record}), .15)
    save_data(output, data, updated)


def save_first_grasp(output, data, record):
    updated = derive_from_grasp(dict(data, waypoints=dict(data['waypoints'], Grasp=record)), .15)
    save_data(output, data, updated)


def interactive(names, data, record, input_fn=input, emit=print):
    emit('依次手动移动并停止，按回车记录。s 跳过，q 退出；每个位置保存后立即写入文件。')
    for index, name in enumerate(names, 1):
        while True:
            exists = name in data['waypoints']
            hint = HINTS[name]
            emit('\n[{}/{}] {}：{}'.format(index, len(names), name, hint))
            prompt = ('已有记录：回车保留，r 重新记录，q 退出：' if exists else
                      '请手动到位并停止，回车记录，s 跳过，q 退出：')
            try:
                command = input_fn(prompt).strip().lower()
            except (EOFError, KeyboardInterrupt):
                emit('\n已退出，先前保存的位置保留。')
                return
            if command == 'q':
                emit('已退出，先前保存的位置保留。')
                return
            if command == 's' or (exists and command == ''):
                emit('保留已有记录。' if exists else '已跳过。')
                break
            if command != ('r' if exists else ''):
                emit('请输入提示中的选项。')
                continue
            try:
                record(name)
            except (EOFError, KeyboardInterrupt):
                emit('\n已退出，先前保存的位置保留。')
                return
            except Exception as error:
                emit('记录失败：{}。请检查机械臂已停止和反馈正常，再按回车重试{}。'.format(
                    error, '（已有记录请按 r）' if exists else ''))
                continue
            break
    missing = [name for name in names if name not in data['waypoints']]
    emit('示教结束。' if not missing else '示教结束，尚缺位置：'+', '.join(missing))


def main():
    defaults = Path(rospkg.RosPack().get_path('fixed_grasp'))/'config'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', choices=NAMES, nargs='?', help='单点直接记录；省略则交互示教全部位置')
    parser.add_argument('--output', help='默认第一套写 waypoints.yaml，第二套写 waypoints_2.yaml')
    parser.add_argument('--grasp', type=int, choices=(1, 2), default=1)
    parser.add_argument('--shared-waypoints', default=str(defaults/'waypoints.yaml'))
    parser.add_argument('--config', default=str(defaults/'fixed.yaml'))
    parser.add_argument('--replace', action='store_true', help='单点模式允许覆盖已有记录')
    parser.add_argument('--interactive', action='store_true', help='单点模式也等待回车确认')
    parser.add_argument('--grasp-only', action='store_true', help='交互示教 Home、Pregrasp、Grasp、Lift')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.name and args.grasp_only:
        parser.error('--grasp-only 不能与单点名称同时使用')
    if args.grasp == 2 and args.name not in (None, 'Grasp'):
        parser.error('第二套只示教 Grasp；Pregrasp/Lift 自动生成，共用原 Home 和放置位置')
    shared_path = Path(args.shared_waypoints).expanduser()
    output = Path(args.output).expanduser() if args.output else (
        shared_path if args.grasp == 1 else shared_path.with_name('waypoints_2.yaml'))
    if args.grasp == 2 and output.resolve() == shared_path.resolve():
        parser.error('第二套输出不能覆盖共用示教文件')
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fixed_grasp_teach', anonymous=True)
    config = load_settings(args.config)
    session = Session(config, lambda: None)
    tcp = geometry(config).at_width(config['contact_width_m'])['flange_tcp']
    fingerprint = identity(session, tcp)
    if args.grasp == 2:
        shared, digest = read_shared(shared_path)
        if shared['identity'] != fingerprint:
            raise RuntimeError('shared file model differs from the actual robot')
        data = (yaml.safe_load(output.read_text()) if output.exists() and not args.replace
                else new_grasp_profile(shared, digest))
        validate_profile_reference(data, shared, digest)
    else:
        data = yaml.safe_load(output.read_text()) if output.exists() else dict(schema=1, identity=fingerprint, waypoints={})
    if not isinstance(data, dict) or data.get('schema') != 1 or data.get('identity') != fingerprint:
        raise RuntimeError('existing file model differs; use a new teaching file')
    if not isinstance(data.get('waypoints'), dict):
        raise ValueError('invalid waypoints mapping')

    def record(name):
        value = capture(session, config, tcp)
        if args.grasp == 2:
            save_second_grasp(output, data, value)
            print('第二套 Pregrasp = Lift = Grasp 沿基座 +Z 上方 15 cm；共用原放置位置。', flush=True)
        elif name == 'Grasp':
            save_first_grasp(output, data, value)
            print('第一套 Pregrasp = Lift = Grasp 沿基座 +Z 上方 15 cm；请实测后更新放置确认。', flush=True)
        else:
            save_pose(output, data, name, value)
        position = np.asarray(value['tcp_base'])[:3, 3]
        print('已保存 {} → {}\nTCP 基座位置 [m]：{}'.format(name, output, position.tolist()), flush=True)

    print('示教文件：'+str(output), flush=True)
    if args.name is None or args.interactive:
        names = ['Grasp'] if args.grasp == 2 else [args.name] if args.name else NAMES[:4] if args.grasp_only else NAMES
        interactive(names, data, record)
    else:
        if args.name in data['waypoints'] and not args.replace:
            raise RuntimeError('pose exists; use --replace explicitly')
        record(args.name)


if __name__ == '__main__':
    try:
        main()
    except (EOFError, KeyboardInterrupt, rospy.ROSInterruptException):
        print('\n已退出，先前保存的位置保留。')
