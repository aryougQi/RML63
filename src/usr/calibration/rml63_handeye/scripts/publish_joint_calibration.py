#!/usr/bin/env python3
"""Publish measured q + calibration offsets as a separate FK branch, with the matched camera X."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix, quaternion_from_matrix

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joint_calibration_model import RobotChain, JOINT_NAMES


def chain_signature(chain):
    payload = [(origin.tolist(), axis.tolist(), index) for origin, axis, index in chain.chain]
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def load_config(path, xml):
    config = yaml.safe_load(Path(path).expanduser().read_text())
    if config.get('schema') != 1 or config.get('joint_names') != JOINT_NAMES:
        raise ValueError('Unsupported calibration config or joint order')
    frames = [config[k] for k in ('base_frame', 'nominal_effector_frame', 'calibrated_effector_frame',
                                  'camera_link_frame', 'camera_optical_frame')]
    if len(set(frames)) != len(frames) or any(not f or f.startswith('/') for f in frames):
        raise ValueError('Frame names must be distinct and have no leading slash')
    chain = RobotChain(xml, config['base_frame'], config['nominal_effector_frame'])
    if chain_signature(chain) != config['robot_chain_sha256']:
        raise ValueError('Running robot kinematic chain differs from calibrated chain; review model before applying')
    delta = np.radians([config['joint_offsets_deg'][n] for n in JOINT_NAMES])
    if not np.isfinite(delta).all() or np.max(np.abs(delta)) > np.radians(3.000001):
        raise ValueError('Joint corrections must be finite degrees within the fitted +/-3 degree bound')
    if abs(delta[0]) > 1e-12 or abs(delta[5]) > 1e-12:
        raise ValueError('This calibration fixes J1/J6 gauge offsets to zero; changing these needs a new consistent calibration')
    transform = config['effector_to_camera_optical']
    q, t = np.asarray(transform['quaternion_xyzw'], dtype=float), np.asarray(transform['translation_m'], dtype=float)
    if q.shape != (4,) or t.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(t).all():
        raise ValueError('Invalid camera transform shape/value')
    if abs(np.linalg.norm(q)-1) > 1e-5:
        raise ValueError('Camera quaternion must have unit length')
    x = quaternion_matrix(q)
    x[:3, 3] = t
    return config, chain, delta, x


def message_matrix(transform):
    q, t = transform.rotation, transform.translation
    value = quaternion_matrix([q.x, q.y, q.z, q.w])
    value[:3, 3] = [t.x, t.y, t.z]
    return value


def stamped(parent, child, value, stamp):
    result = TransformStamped()
    result.header.frame_id, result.child_frame_id = parent, child
    result.header.stamp = stamp
    result.transform.translation.x, result.transform.translation.y, result.transform.translation.z = value[:3, 3]
    q = quaternion_from_matrix(value)
    result.transform.rotation.x, result.transform.rotation.y, result.transform.rotation.z, result.transform.rotation.w = q
    return result


class CalibratedCameraPublisher:
    def __init__(self):
        path = rospy.get_param('~calibration_file')
        content_hash = hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest()
        self.config, self.chain, self.delta, x = load_config(path, rospy.get_param('/robot_description'))
        if hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest() != content_hash:
            raise RuntimeError('Calibration file changed while loading; restart the publisher')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        cfg = self.config
        native = self.buffer.lookup_transform(cfg['camera_link_frame'], cfg['camera_optical_frame'],
                                              rospy.Time(0), rospy.Duration(10))
        existing = yaml.safe_load(self.buffer.all_frames_as_yaml()) or {}
        if cfg['camera_link_frame'] in existing:
            raise RuntimeError('camera_link already has a TF parent; stop the existing hand-eye publisher first')
        if cfg['calibrated_effector_frame'] in existing:
            raise RuntimeError('Calibrated effector frame already exists; stop its other publisher first')
        # Native RealSense chain is preserved: X_optical must be converted, not renamed.
        mount = x @ np.linalg.inv(message_matrix(native.transform))
        self.static = tf2_ros.StaticTransformBroadcaster()
        self.dynamic = tf2_ros.TransformBroadcaster()
        self.static.sendTransform(stamped(cfg['calibrated_effector_frame'], cfg['camera_link_frame'], mount, rospy.Time.now()))
        self.last_stamp = rospy.Time(0)
        self.max_age = float(rospy.get_param('~max_joint_age_sec', .5))
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.watchdog)
        # Consumers can reject mixing a newly edited X with an older FK offset set.
        rospy.set_param('~calibration_sha256', content_hash)
        rospy.loginfo('Active joint calibration: %s', path)
        rospy.loginfo('TF: %s -> %s -> %s -> native camera optical frames; controller/model untouched',
                      cfg['base_frame'], cfg['calibrated_effector_frame'], cfg['camera_link_frame'])

    def callback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(n in values for n in JOINT_NAMES):
            rospy.logwarn_throttle(5, 'Missing arm joints in joint_states; no calibrated TF published')
            return
        age = (rospy.Time.now()-msg.header.stamp).to_sec()
        if not -.05 <= age <= self.max_age or msg.header.stamp <= self.last_stamp:
            rospy.logwarn_throttle(5, 'Joint timestamp stale/out of order; no calibrated TF published')
            return
        q = np.asarray([values[n] for n in JOINT_NAMES])
        if not np.isfinite(q).all():
            rospy.logerr_throttle(5, 'Non-finite measured joint angles')
            return
        result = self.chain.fk(q, self.delta)
        self.dynamic.sendTransform(stamped(self.config['base_frame'], self.config['calibrated_effector_frame'],
                                           result, msg.header.stamp))
        self.last_stamp = msg.header.stamp

    def watchdog(self, _):
        if (rospy.Time.now()-self.last_stamp).to_sec() > self.max_age:
            rospy.logwarn_throttle(5, 'No fresh joint feedback: calibrated camera TF is not being refreshed')


if __name__ == '__main__':
    rospy.init_node('rml63_handeye_tf')
    try:
        publisher = CalibratedCameraPublisher()
        rospy.spin()
    except Exception as error:
        rospy.logfatal('Calibrated camera publisher stopped: %s', error)
        sys.exit(1)
