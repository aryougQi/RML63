#!/usr/bin/env python3
"""Display the actual rectified input and upstream detector overlay side by side."""
import sys
import threading
import time

import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget
from sensor_msgs.msg import Image


class DebugView(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('L515 手眼标定：去畸变图像 / AprilTag 识别')
        self.resize(1250, 620)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {}
        self.last_displayed = {}
        self.pixmaps = {}
        self.detections = None
        self.detection_received = 0
        self.panels = {}
        self.subscribers = []
        outer = QVBoxLayout(self)
        self.status = QLabel('等待相机与 AprilTag 检测数据……')
        self.status.setWordWrap(True)
        self.status.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        outer.addWidget(self.status)
        images = QHBoxLayout()
        for key, title, topic in [
                ('rectified', '去畸变图像（Tag 检测器的输入）', 'image_rect_color'),
                ('debug', 'AprilTag Debug（识别框、角点和 ID）', 'tag_detections_image')]:
            panel = QVBoxLayout()
            panel.addWidget(QLabel(title))
            label = QLabel('等待图像')
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(320, 240)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            label.setStyleSheet('background: #202020; color: white;')
            panel.addWidget(label, 1)
            panel.addWidget(QLabel(rospy.resolve_name(topic)))
            images.addLayout(panel)
            self.panels[key] = label
            self.subscribers.append(rospy.Subscriber(
                topic, Image, self.receive_frame, callback_args=key,
                queue_size=1, buff_size=2**24))
        outer.addLayout(images, 1)
        self.subscribers.append(rospy.Subscriber(
            'tag_detections', AprilTagDetectionArray, self.receive_detections, queue_size=1))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def receive_frame(self, message, key):
        with self.lock:
            self.frames[key] = (message, time.monotonic())

    def receive_detections(self, message):
        with self.lock:
            self.detections = message
            self.detection_received = time.monotonic()

    def refresh(self):
        if rospy.is_shutdown():
            self.close()
            return
        with self.lock:
            frames = dict(self.frames)
            detections = self.detections
            detection_age = time.monotonic()-self.detection_received
        for key, (message, received) in frames.items():
            if time.monotonic()-received > 2:
                self.panels[key].setText('图像已停止更新')
                continue
            if self.last_displayed.get(key) is not message:
                try:
                    pixels = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(message, 'rgb8'))
                    height, width = pixels.shape[:2]
                    image = QImage(pixels.data, width, height, pixels.strides[0], QImage.Format_RGB888).copy()
                    self.pixmaps[key] = QPixmap.fromImage(image)
                    self.last_displayed[key] = message
                except (CvBridgeError, ValueError) as error:
                    self.panels[key].setText(str(error))
                    continue
            if key in self.pixmaps:
                self.panels[key].setPixmap(self.pixmaps[key].scaled(
                    self.panels[key].size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        if detections is None or detection_age > 2:
            self.status.setText('等待新的 Tag 检测结果；请检查相机和检测节点是否运行。')
        elif not detections.detections:
            self.status.setText('图像正常，尚未识别到 Tag。请让整个标签及白边清晰地出现在画面中。')
        else:
            items = []
            for detection in detections.detections:
                distance = detection.pose.pose.pose.position.z
                items.append('ID %s，边长 %s m，Z≈%.3f m' % (detection.id, detection.size, distance))
            self.status.setText('已识别：'+'；'.join(items))


if __name__ == '__main__':
    rospy.init_node('image_debug', disable_signals=True)
    app = QApplication(sys.argv)
    window = DebugView()
    window.show()
    app.exec_()
    rospy.signal_shutdown('Debug image window closed')
