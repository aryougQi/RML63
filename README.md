# RML63 核心代码

ROS Noetic / MoveIt 1 下的 RML63 真机驱动、视觉检测、手眼标定与抓取代码。

## 内容

- `src/rm_robot`：驱动、控制、消息定义、RML63 模型和 MoveIt 配置。
- `src/rml63_tools_description`：夹爪模型、安装配置和真机启动入口。
- `src/usr/depth_handle_detect`、`src/usr/pnp_detect`：深度与 PnP 视觉检测。
- `src/usr/calibration`：AprilTag、easy_handeye 及本地标定封装。
- `src/usr/demo`：真机抓取与标记触碰流程。

## 构建与使用

在已安装 ROS Noetic、MoveIt 和所需相机驱动的环境中：

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

Python 视觉依赖见各模块的 `requirements.txt`。具体流程见各模块 README
及 `src/usr/demo/depth_handle_grasp/ROS_USAGE.md`。

本仓库不包含仿真包、Docker 仿真环境、采集数据、运行报告、构建产物或
设备生成的标定结果。厂商机器人模型所引用的 Gazebo xacro 扩展作为模型依赖保留。
第三方源码和许可证保留在原目录中。

`active_joint_calibration.yaml` 为设备生成结果，需在自己的设备上完成标定后，
通过启动参数 `calibration_file` 指定结果文件。现有设备 IP、目标尺寸和安装配置
仅供参考，实际运行前必须按设备重新检查。模块文档中的历史数据和验证报告路径
不随本仓库提供。

本次只整理发布范围，未进行真机执行或完整 ROS 构建验证。
