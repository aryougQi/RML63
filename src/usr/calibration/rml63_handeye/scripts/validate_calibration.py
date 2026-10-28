#!/usr/bin/env python3
"""Read-only eye-in-hand validation: offline residuals or independent manual poses."""
import argparse
import datetime
from pathlib import Path
import time

import numpy as np
import yaml
from tf.transformations import quaternion_matrix


def matrix(value):
    result = quaternion_matrix([value['rotation'][k] for k in 'xyzw'])
    result[:3, 3] = [value['translation'][k] for k in 'xyz']
    return result


def transform_dict(value):
    return {'translation': {k: float(getattr(value.translation, k)) for k in 'xyz'},
            'rotation': {k: float(getattr(value.rotation, k)) for k in 'xyzw'}}


def mean_transform(values):
    values = np.asarray(values)
    result = np.eye(4)
    result[:3, 3] = values[:, :3, 3].mean(axis=0)
    u, _, vt = np.linalg.svd(values[:, :3, :3].mean(axis=0))
    result[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return result


def difference(a, b):
    distance = float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000)
    cosine = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2
    return distance, float(np.degrees(np.arccos(np.clip(cosine, -1, 1))))


def statistics(values):
    center = mean_transform(values)
    errors = np.array([difference(center, value) for value in values])
    pair = max(difference(a, b)[0] for a in values for b in values)
    return {'pose_count': len(values), 'tag_center_base_m': center[:3, 3].tolist(),
            'position_rms_mm': float(np.sqrt(np.mean(errors[:, 0] ** 2))),
            'position_max_mm': float(errors[:, 0].max()),
            'position_max_pair_mm': pair,
            'rotation_rms_deg': float(np.sqrt(np.mean(errors[:, 1] ** 2))),
            'rotation_max_deg': float(errors[:, 1].max())}


def print_statistics(label, values):
    report = statistics(values)
    print('\n' + label)
    print(yaml.safe_dump(report, sort_keys=False), flush=True)
    return report


def load_calibration(path):
    data = yaml.safe_load(path.read_text())
    p = data['parameters']
    if not p['eye_on_hand']:
        raise ValueError('Expected eye-on-hand calibration')
    t = data['transformation']
    x = matrix({'translation': t, 'rotation': {k: t['q' + k] for k in 'xyzw'}})
    if not np.isfinite(x).all():
        raise ValueError('Non-finite calibration')
    return data, x


def live_capture(buffer, frames, x, timeout=12):
    import rospy
    import tf2_ros
    start = rospy.Time.now()
    deadline = time.monotonic() + timeout
    last = start
    pending = []
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            camera = buffer.lookup_transform(frames[2], frames[3], rospy.Time(0), rospy.Duration(.1))
            stamp = camera.header.stamp
            age = (rospy.Time.now() - stamp).to_sec()
            if stamp <= last or not -.05 <= age <= .5:
                time.sleep(.03)
                continue
            robot = buffer.lookup_transform(frames[0], frames[1], stamp, rospy.Duration(.2))
            last = stamp
            gd, cd = transform_dict(robot.transform), transform_dict(camera.transform)
            g, c = matrix(gd), matrix(cd)
            if pending:
                dg, ag = difference(pending[0][0], g)
                dc, ac = difference(pending[0][1], c)
                if dg > 1 or ag > .5 or dc > 4 or ac > 1.5:
                    pending = []
            pending.append((g, c, {'stamp_sec': stamp.to_sec(), 'hand_world': gd, 'camera_marker': cd}))
            if len(pending) >= 15 and (stamp.to_sec() - pending[0][2]['stamp_sec']) >= .6:
                ys = [g @ x @ c for g, c, _ in pending]
                return mean_transform(ys), mean_transform([v[0] for v in pending]), {
                    'observations': [v[2] for v in pending], 'within_pose': statistics(ys)}
        except tf2_ros.TransformException:
            time.sleep(.03)
    raise RuntimeError('采集超时：请停稳机械臂，保持完整 Tag 可见，检查相机时间戳和 TF。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=Path.home() / '.ros/easy_handeye/rml63_l515_eye_on_hand.yaml')
    parser.add_argument('--samples', type=Path, help='Offline auto-run samples.yaml; defaults to newest run')
    parser.add_argument('--live', action='store_true', help='Manually capture independent poses; never commands motion')
    args = parser.parse_args()
    calibration, x = load_calibration(args.calibration)
    if not args.live:
        files = sorted((Path.home() / '.ros/rml63_handeye_runs').glob('*/samples.yaml'))
        path = args.samples or (files[-1] if files else None)
        if path is None:
            raise RuntimeError('No saved samples found')
        samples = yaml.safe_load(path.read_text())
        gs, cs = samples['hand_world_samples'], samples['camera_marker_samples']
        if len(gs) != len(cs) or len(gs) < 3:
            raise ValueError('Need matching sample lists with at least 3 poses')
        ys = [matrix(g) @ x @ matrix(c) for g, c in zip(gs, cs)]
        report = {'calibration_file': str(args.calibration), 'samples_file': str(path),
                  'training_residuals': print_statistics('训练样本残差（不是独立精度验证）', ys)}
        output = path.parent / 'validation_training_report.yaml'
        output.write_text(yaml.safe_dump(report, sort_keys=False, allow_unicode=True))
        print('报告已保存：' + str(output))
        return

    import rospy
    import tf2_ros
    rospy.init_node('rml63_validate_handeye', anonymous=True, disable_signals=True)
    buffer = tf2_ros.Buffer(rospy.Duration(30))
    listener = tf2_ros.TransformListener(buffer)
    p = calibration['parameters']
    frames = [p[k] for k in ('robot_base_frame', 'robot_effector_frame', 'tracking_base_frame', 'tracking_marker_frame')]
    directory = Path.home() / '.ros/rml63_handeye_validations' / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    directory.mkdir(parents=True)
    report_path = directory / 'validation.yaml'
    poses, robots, details = [], [], []
    print('只读取 TF，不发送运动命令。固定 Tag 和相机支架，保持检测程序运行。\n'
          '手动换到未用于标定的新姿态、停稳后按 Enter 采集；建议 5～6 个姿态、至少绕两条轴改变朝向。\n'
          '每姿态采集至少 15 帧/0.6 秒，只有停稳数据才保留。输入 q 结束。', flush=True)
    try:
        while not rospy.is_shutdown():
            if input('姿态 {}：停稳后 Enter；q 结束 > '.format(len(poses) + 1)).strip().lower() == 'q':
                break
            try:
                y, g, detail = live_capture(buffer, frames, x)
            except RuntimeError as exc:
                print(str(exc), flush=True)
                continue
            if any((lambda da: da[0] < 10 and da[1] < 5)(difference(old, g)) for old in robots):
                print('与已采集姿态过近，请改变位置至少 10 mm 或朝向至少 5° 后重试。')
                continue
            poses.append(y)
            robots.append(g)
            details.append(detail)
            report = {'calibration_file': str(args.calibration), 'calibration': calibration,
                      'independent_pose_statistics': print_statistics('独立姿态验证（相对均值的漂移）', poses),
                      'base_tag_matrices': [value.tolist() for value in poses], 'captures': details}
            report_path.write_text(yaml.safe_dump(report, sort_keys=False))
            print('已保存：' + str(report_path), flush=True)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rospy.signal_shutdown('Read-only validation finished')
    if len(poses) < 5:
        print('姿态数量不足 5，请增加独立姿态后再判断；单姿态或静止抖动不能验证手眼标定。')
    print('位置散布反映相对一致性，不能证明绝对抓取精度或 Tag 尺寸正确。报告：' + str(report_path))


if __name__ == '__main__':
    main()
