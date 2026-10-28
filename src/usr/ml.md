# 查看相机

~/.local/bin/l515-camera-ros
~/.local/bin/l515-rqt /camera/color/image_raw

# 启动ros
source ~/realman_RML63/devel/setup.bash

ROS_MASTER_URI=http://localhost:11311 \
roslaunch rml63_tools_description realrobot.launch \
allow_execution:=true

# 打开标定（辨识零点偏移）

source ~/realman_RML63/devel/setup.bash
roslaunch rml63_handeye calibrate.launch tag_size:=0.151 show_debug:=true

# 自动标定程序（会启动机械臂）

source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

roslaunch rml63_handeye auto_collect_joints.launch \
  rotation_delta_degrees:=10 \
  translation_delta_meters:=0.025 \
  execute:=true \
  tag_size_verified:=true

# 标定精度验证

source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/collect_joint_calibration.py \
  --role validation

# 触碰tag demo

roslaunch rml63_tag_touch touch.launch \
  execute:=true \
  clearance_m:=0.0 \
  velocity_scaling:=0.20 \
  max_target_distance_m:=0.70 \
  retract:=false

# 加载系统环境

source /opt/ros/noetic/setup.bash
source /home/pc/realman_RML63/devel/setup.bash
cd /home/pc/realman_RML63

# 抓取demo
## 启动抓取的相机ros节点
source /opt/ros/noetic/setup.bash
source /home/pc/realman_RML63/devel/setup.bash
roslaunch depth_handle_grasp camera.launch

## 启动moveit
source /opt/ros/noetic/setup.bash
source /home/pc/realman_RML63/devel/setup.bash
cd /home/pc/realman_RML63

roslaunch rml63_tools_description realrobot.launch allow_execution:=true

## ROI设定
python3 /home/pc/realman_RML63/src/usr/depth_handle_detect/scripts/select_roi.py \
  --live \
  --write-config \
  --config /home/pc/realman_RML63/src/usr/depth_handle_detect/configs/real_handle.json

## 检测路径
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py

## 抓取
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py \
  --config /tmp/rml63_side_place.yaml \
  --execute

## 返回
RUN_DIR=$(python3 -c 'import json; print(json.load(open("src/usr/demo/depth_handle_grasp/reports/latest_run.json"))["run_directory"])')

python3 src/usr/demo/depth_handle_grasp/scripts/return_demo.py \
  --run "$RUN_DIR" --execute


