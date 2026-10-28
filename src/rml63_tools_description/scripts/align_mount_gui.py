#!/usr/bin/env python3
"""Adjust only the virtual gripper mounting yaw on a dedicated local ROS master."""
import math
import os
import socket
from pathlib import Path
import sys
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QApplication, QDoubleSpinBox, QHBoxLayout, QLabel,
                            QPushButton, QSlider, QVBoxLayout, QWidget)
from PyQt5.QtCore import Qt
import rospy
from sensor_msgs.msg import JointState
import yaml

ARM_DEGREES = [0.0, 50.0, -139.997, -0.001, -89.998, -30.673]


class AlignmentWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('夹爪安装角核对（仅虚拟预览）')
        root = ET.fromstring(rospy.get_param('/robot_description'))
        mount = next(j for j in root.findall('joint') if j.attrib['name'] == 'eg2_mount_joint')
        self.original_yaw = float(mount.find('origin').attrib['rpy'].split()[2])
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('机械臂固定为照片姿态，夹爪闭合。此窗口不会控制真机。'))
        layout.addWidget(QLabel('J1=0°  J2=50°  J3=-139.997°  J4=-0.001°\nJ5=-89.998°  J6=-30.673°'))
        layout.addWidget(QLabel('夹爪相对 link6 绕 Z 轴的安装角（度）：'))
        self.angle = QDoubleSpinBox()
        self.angle.setRange(-180.0, 180.0)
        self.angle.setDecimals(2)
        self.angle.setSingleStep(1.0)
        self.angle.setSuffix(' °')
        self.angle.setValue(math.degrees(self.original_yaw))
        layout.addWidget(self.angle)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-18000, 18000)
        slider.setValue(round(self.angle.value() * 100))
        slider.valueChanged.connect(lambda value: self.angle.setValue(value / 100.0))
        self.angle.valueChanged.connect(lambda value: slider.setValue(round(value * 100)))
        layout.addWidget(slider)
        presets = QHBoxLayout()
        for value in [-150, -60, -30, 0, 30, 60, 120]:
            button = QPushButton('%+d°' % value)
            button.clicked.connect(lambda checked=False, v=value: self.angle.setValue(v))
            presets.addWidget(button)
        layout.addLayout(presets)
        save = QPushButton('保存当前角度记录（不自动修改正式模型）')
        save.clicked.connect(self.save)
        layout.addWidget(save)
        self.status = QLabel('先比较两指开合方向；若只是观察视角不同，请旋转 RViz 视角。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish)
        self.timer.start(50)

    def publish(self):
        if rospy.is_shutdown():
            self.close()
            return
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = ['joint%d' % i for i in range(1, 7)] + ['mount_alignment_joint']
        message.position = [math.radians(v) for v in ARM_DEGREES] + [
            math.radians(self.angle.value()) - self.original_yaw]
        self.publisher.publish(message)

    def save(self):
        path = Path(rospy.get_param('~output')).expanduser()
        result = {
            'mount_yaw_deg': self.angle.value(),
            'mount_rpy': [0.0, 0.0, math.radians(self.angle.value())],
            'arm_reference_deg': ARM_DEGREES,
            'gripper_closed': True,
            'status': 'user_visual_alignment_pending_physical_validation',
        }
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix('.tmp')
            temporary.write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))
            temporary.replace(path)
            self.status.setText('已保存角度：%.2f°\n%s' % (self.angle.value(), path))
        except OSError as error:
            self.status.setText('保存失败：%s' % error)


def main():
    master = urlparse(os.environ.get('ROS_MASTER_URI', ''))
    local_names = {'localhost', '127.0.0.1', '::1', socket.gethostname().lower(), socket.getfqdn().lower()}
    if master.hostname not in local_names or master.port != 11313:
        raise SystemExit('请使用独立预览 master：ROS_MASTER_URI=http://localhost:11313 roslaunch -p 11313 rml63_tools_description align_mount.launch')
    rospy.init_node('mount_alignment_gui', disable_signals=True)
    app = QApplication(sys.argv)
    window = AlignmentWindow()
    window.show()
    app.exec_()
    rospy.signal_shutdown('Alignment window closed')


if __name__ == '__main__':
    main()
