# RML63 / RealSense L515 / AprilTag 手眼标定

所有标定相关源码、配置和打印文件统一放在本目录：

```text
src/usr/calibration/
  apriltag/           # https://github.com/AprilRobotics/apriltag
  apriltag_ros/       # https://github.com/AprilRobotics/apriltag_ros
  easy_handeye/       # https://github.com/IFL-CAMP/easy_handeye
  rml63_handeye/      # 本机 ROS1 配置、图像校正和自动采样入口
  targets/           # 原来工作空间外层 calibration/targets 的打印文件
  SOURCES.yaml       # 下载来源及固定提交
```

详见 [启动与采样说明](rml63_handeye/README.md)。
夹爪/机械臂模型包仍在 src/rml63_tools_description，属于真机模型，不在此重复复制。
