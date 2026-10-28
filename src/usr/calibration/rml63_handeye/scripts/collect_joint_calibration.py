#!/usr/bin/env python3
"""Manual, read-only collection of joint angles and AprilTag poses at matched times."""
import argparse
from collections import deque
from datetime import datetime
from pathlib import Path
import threading
import sys
import subprocess
import rosnode

import numpy as np
import yaml
import rospy
import tf2_ros
from sensor_msgs.msg import JointState, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from apriltag_ros.msg import AprilTagDetectionArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joint_calibration_model import JOINT_NAMES, RobotChain, mean_transform
from validate_calibration import live_capture, load_calibration, matrix, difference


class JointHistory:
    def __init__(self):
        self.lock = threading.Lock()
        self.values = deque(maxlen=6000)
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size=100)

    def callback(self, msg):
        angles = dict(zip(msg.name, msg.position))
        if not all(k in angles for k in JOINT_NAMES) or msg.header.stamp.to_sec() <= 0:
            return
        with self.lock:
            self.values.append((msg.header.stamp.to_sec(), [angles[k] for k in JOINT_NAMES]))

    def at(self, stamp):
        with self.lock:
            values = sorted(self.values)
        for (a, qa), (b, qb) in zip(values, values[1:]):
            if a <= stamp <= b and b > a and b-a <= .15:
                # Reported revolute coordinates are interpolated without guessing a wrap/turn.
                if max(abs(np.asarray(qb)-qa)) > np.radians(2):
                    raise RuntimeError('Joint feedback moved or jumped during capture')
                return (np.asarray(qa)+(np.asarray(qb)-qa)*(stamp-a)/(b-a)).tolist()
        raise RuntimeError('No bracketing joint states at the image timestamp; retry when stopped')


class CameraSnapshot:
    def __init__(self):
        self.images = deque(maxlen=60)
        self.info = None
        self.subscribers = [rospy.Subscriber('/camera/color/image_raw', Image, self.images.append, queue_size=2),
                            rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.set_info, queue_size=1)]

    def set_info(self, msg):
        self.info = msg

    def save(self, directory, index, stamp):
        images = list(self.images)
        if not images or self.info is None:
            return False
        image = min(images, key=lambda m: abs(m.header.stamp.to_sec()-stamp))
        if abs(image.header.stamp.to_sec()-stamp) > .04:
            return False
        path = directory / ('pose_%03d_raw.png' % index)
        if not cv2.imwrite(str(path), CvBridge().imgmsg_to_cv2(image, 'bgr8')):
            return False
        info = self.info
        path.with_suffix('.yaml').write_text(yaml.safe_dump({
            'stamp_sec': image.header.stamp.to_sec(), 'frame': info.header.frame_id,
            'width': info.width, 'height': info.height, 'K': list(info.K), 'D': list(info.D),
            'P': list(info.P), 'distortion_model': info.distortion_model}))
        return True


def ensure_detection_view():
    """Start only the missing viewer/detector; return owned child for cleanup."""
    nodes = set(rosnode.get_node_names())
    if '/rml63_apriltag/detector' not in nodes:
        rospy.wait_for_message('/camera/color/camera_info', CameraInfo, timeout=6)
        return subprocess.Popen(['roslaunch', 'rml63_handeye', 'detect.launch',
                                 'tag_size:=0.151', 'show_debug:=true'], stdout=subprocess.DEVNULL)
    if any(n.startswith('/rml63_apriltag/') and ('image_debug' in n or 'joint_calibration_debug' in n) for n in nodes):
        return None
    return subprocess.Popen([sys.executable, str(Path(__file__).resolve().parent/'image_debug_view.py'),
                             '__ns:=/rml63_apriltag', '__name:=joint_calibration_debug'], stdout=subprocess.DEVNULL)


def capture_joint_sample(buffer, frames, x, chain, history, camera, directory, index):
    _, _, capture = live_capture(buffer, frames, x)
    qs = []
    for observation in capture['observations']:
        q = history.at(observation['stamp_sec'])
        observation['joint_positions_rad'] = q
        d, a = difference(chain.fk(q), matrix(observation['hand_world']))
        if d > .5 or a > .1:
            raise RuntimeError('URDF FK and observed TF disagree (%.3f mm, %.3f deg)' % (d, a))
        qs.append(q)
    if np.max(np.ptp(qs, axis=0)) > np.radians(.2):
        raise RuntimeError('关节尚未停稳，请重试')
    mid_stamp = capture['observations'][len(qs)//2]['stamp_sec']
    return {'joint_positions_rad': np.mean(qs, axis=0).tolist(),
            'camera_marker_matrix': mean_transform([matrix(o['camera_marker']) for o in capture['observations']]).tolist(),
            'capture': capture, 'raw_image_saved': camera.save(directory, index, mid_stamp), 'used_for_fit': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['train', 'validation'], default='train')
    parser.add_argument('--calibration', type=Path, default=Path.home()/'.ros/easy_handeye/rml63_l515_manual_eye_on_hand.yaml')
    parser.add_argument('--no-view', action='store_true', help='Do not start missing detector/debug window')
    args = parser.parse_args()
    calibration, x = load_calibration(args.calibration)
    rospy.init_node('joint_calibration_readonly_collector', anonymous=True, disable_signals=True)
    view = None if args.no_view else ensure_detection_view()
    if view is not None:
        rospy.on_shutdown(lambda: view.terminate() if view.poll() is None else None)
        rospy.wait_for_message('/rml63_apriltag/tag_detections', AprilTagDetectionArray, timeout=20)
    parameters = calibration['parameters']
    frames = [parameters[k] for k in ('robot_base_frame', 'robot_effector_frame', 'tracking_base_frame', 'tracking_marker_frame')]
    xml = rospy.get_param('/robot_description')
    chain = RobotChain(xml, frames[0], frames[1])
    history, camera = JointHistory(), CameraSnapshot()
    buffer = tf2_ros.Buffer(rospy.Duration(30))
    listener = tf2_ros.TransformListener(buffer)
    directory = Path.home()/'.ros/rml63_joint_calibration_runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    (directory/'robot.urdf').write_text(xml)
    dataset = {'schema': 1, 'role': args.role, 'joint_names': JOINT_NAMES,
               'initial_calibration': calibration, 'tag_config': rospy.get_param('/rml63_apriltag/detector/standalone_tags'),
               'frames': frames, 'samples': []}
    print('只读采集：自己示教运动，停稳且 Tag 可见后按 Enter；q 结束。\n'
          '本工具直接保存同步关节角与视觉数据，不需要点击 easy_handeye 的 Take sample。\n'
          '训练建议 30～40 姿态；验证另开 --role validation 采 8～12 个新姿态。\n目录：'+str(directory), flush=True)
    try:
        while not rospy.is_shutdown():
            command = input('已保存 %d 个姿态；Enter 采集 / r 重复性检查 / q 结束 > ' % len(dataset['samples'])).strip().lower()
            if command == 'q':
                break
            if command not in ('', 'r'):
                print('请输入 Enter、r 或 q')
                continue
            try:
                index = len(dataset['samples'])+1
                sample = capture_joint_sample(buffer, frames, x, chain, history, camera, directory, index)
                q = np.asarray(sample['joint_positions_rad'])
                if command != 'r' and any(np.max(np.abs(q-np.asarray(s['joint_positions_rad']))) < np.radians(2)
                       for s in dataset['samples']):
                    raise RuntimeError('关节配置与已有样本过近，请换一个姿态')
                sample['used_for_fit'] = command != 'r'
                dataset['samples'].append(sample)
                temporary = directory/'dataset.yaml.tmp'
                temporary.write_text(yaml.safe_dump(dataset, sort_keys=False))
                temporary.replace(directory/'dataset.yaml')
                print('已保存第 %d 个姿态；关节角(度)：%s；原图：%s' %
                      (index, np.round(np.degrees(q), 2).tolist(), sample['raw_image_saved']), flush=True)
                active = [s['joint_positions_rad'] for s in dataset['samples'] if s['used_for_fit']]
                if active:
                    print('有效拟合姿态 %d；各关节跨度(度)：%s' %
                          (len(active), np.round(np.degrees(np.ptp(active, axis=0)), 1).tolist()), flush=True)
            except (RuntimeError, ValueError) as error:
                print('本次未保存：'+str(error), flush=True)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rospy.signal_shutdown('Manual collection finished')
    print('数据：'+str(directory/'dataset.yaml'))


if __name__ == '__main__':
    main()
