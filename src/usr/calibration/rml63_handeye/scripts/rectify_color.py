#!/usr/bin/env python3
"""Supply rectified images and matching intrinsics to the upstream AprilTag detector."""
from copy import deepcopy
import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


class Rectifier:
    def __init__(self):
        self.bridge = CvBridge()
        self.key = None
        self.image_pub = rospy.Publisher('image_rect_color', Image, queue_size=1)
        self.info_pub = rospy.Publisher('camera_info_rect', CameraInfo, queue_size=1)
        self.image_sub = message_filters.Subscriber('image_raw', Image, queue_size=1, buff_size=2**24)
        self.info_sub = message_filters.Subscriber('camera_info_raw', CameraInfo, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.info_sub], 10, .02)
        self.sync.registerCallback(self.callback)

    def callback(self, image, info):
        if (image.width, image.height) != (info.width, info.height) or image.header.frame_id != info.header.frame_id:
            rospy.logerr_throttle(2, 'Image and CameraInfo size/frame mismatch')
            return
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            rospy.logerr_throttle(2, 'Unsupported camera distortion model: %s', info.distortion_model)
            return
        key = (image.width, image.height, tuple(info.K), tuple(info.D))
        if key != self.key:
            k = np.asarray(info.K).reshape(3, 3)
            d = np.asarray(info.D)
            if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
                return
            size = (image.width, image.height)
            self.new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, size, 0, size)
            self.maps = cv2.initUndistortRectifyMap(k, d, np.eye(3), self.new_k, size, cv2.CV_32FC1)
            self.key = key
        raw = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        rectified = cv2.remap(raw, *self.maps, cv2.INTER_LINEAR)
        output = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
        output.header = image.header
        new_info = deepcopy(info)
        new_info.header = image.header
        new_info.K = self.new_k.flatten().tolist()
        new_info.D = [0.0] * 5
        new_info.R = np.eye(3).flatten().tolist()
        new_info.P = np.column_stack((self.new_k, np.zeros(3))).flatten().tolist()
        new_info.distortion_model = 'plumb_bob'
        new_info.roi.x_offset = new_info.roi.y_offset = 0
        new_info.roi.width = new_info.roi.height = 0
        new_info.roi.do_rectify = False
        new_info.binning_x = new_info.binning_y = 0
        self.info_pub.publish(new_info)
        self.image_pub.publish(output)


if __name__ == '__main__':
    rospy.init_node('rectify_color')
    Rectifier()
    rospy.spin()
