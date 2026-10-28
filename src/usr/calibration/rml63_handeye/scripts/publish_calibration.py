#!/usr/bin/env python3
"""Publish link6 -> camera_link while retaining RealSense's native optical-frame chain."""
import numpy as np
import rospy
import tf2_ros
from easy_handeye.handeye_calibration import HandeyeCalibration
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def matrix(transform):
    q = transform.rotation
    t = transform.translation
    result = quaternion_matrix([q.x, q.y, q.z, q.w])
    result[:3, 3] = [t.x, t.y, t.z]
    return result


if __name__ == '__main__':
    rospy.init_node('rml63_handeye_tf')
    namespace = rospy.get_param('~calibration_namespace', '/rml63_l515_eye_on_hand')
    calibration = HandeyeCalibration.from_file(namespace)
    if not calibration.parameters.eye_on_hand:
        raise RuntimeError('Expected an eye-in-hand calibration.')
    transform = calibration.transformation
    if transform.header.frame_id != 'link6' or transform.child_frame_id != 'camera_color_optical_frame':
        raise RuntimeError('Saved calibration frames differ from link6 -> camera_color_optical_frame.')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    optical = buffer.lookup_transform('camera_link', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(10))
    # T_link6_camera_link = T_link6_optical * inverse(T_camera_link_optical).
    result = matrix(transform.transform) @ np.linalg.inv(matrix(optical.transform))
    output = TransformStamped()
    output.header.stamp = rospy.Time.now()
    output.header.frame_id = 'link6'
    output.child_frame_id = 'camera_link'
    output.transform.translation.x, output.transform.translation.y, output.transform.translation.z = result[:3, 3]
    q = quaternion_from_matrix(result)
    output.transform.rotation.x, output.transform.rotation.y, output.transform.rotation.z, output.transform.rotation.w = q
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(output)
    rospy.loginfo('Published calibrated link6 -> camera_link; RealSense optical transforms preserved.')
    rospy.spin()
