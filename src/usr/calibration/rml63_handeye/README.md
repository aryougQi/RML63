# RML63 眼在手上标定

配置：L515 随机械臂末端运动，AprilTag 固定在桌面；tag36h11、ID 0，
按用户报告黑色外框边长 151 mm 设置 tag_size:=0.151（不含白边）。
不要依据 PDF 文件名或图片像素数推算打印后的物理长度。

## 坐标系和图像

- 机器人：base_link -> link6，MoveIt 组 arm。
- 视觉：camera_color_optical_frame -> rml63_apriltag_0。
- 原始输入：/camera/color/image_raw 与 /camera/color/camera_info。
- 校正输出：/rml63_apriltag/image_rect_color 与 /rml63_apriltag/camera_info_rect。
- 检测输出：/rml63_apriltag/tag_detections 和 /rml63_apriltag/tag_detections_image。

apriltag_ros 使用校正图像模型，因此先按 L515 CameraInfo 去畸变，并同步发布匹配的 K/P 和零畸变 D。
标定阶段 publish_dummy=false，不提前发布猜测的末端到相机变换。

## 1. 准备

把纸贴在平整、哑光硬板上，固定在桌面，整个采样过程不移动标定板和机器人底座。
相机与末端的连接也必须刚性固定；单纯把纸放在桌面上容易被气流或接触挪动。
核对黑框 151 mm，保持白边；保证整个黑框在彩色图像中完整清晰可见。

所有终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

相机和真机若已经运行，不要重复启动。真机标定时需要执行能力：

```bash
roslaunch rml63_tools_description realrobot.launch allow_execution:=true
```

该模型已有 -60° 夹爪安装角，夹爪模型固定闭合；实物夹爪在标定期间也保持闭合。
目前相机和支架尚未加入碰撞几何，因此 MoveIt 的无碰撞规划不代表相机/支架的运动空间已被验证。

## 2. 启动官方检测与 easy_handeye

```bash
roslaunch rml63_handeye calibrate.launch tag_size:=0.151
```

启动官方 AprilTag 检测器、easy_handeye 标定服务、采样 GUI 和官方自动姿态生成/逐步运动 GUI，
并自动打开双画面调试窗口：左侧去畸变输入，右侧带识别框/角点/ID 的 AprilTag Debug。
不会因为 launch 启动就自动运动。使用 L515 彩色画面检测，不需要深度图用于这一步标定。
检测图像可用现有工具查看：

```bash
~/.local/bin/l515-rqt /rml63_apriltag/tag_detections_image
```

只测试识别时可运行 `roslaunch rml63_handeye detect.launch`，但不要与 calibrate.launch 重复运行检测节点。
官方自动运动 GUI 可逐个检查初始姿态、规划、执行；执行后在采样 GUI 点击 Take Sample，最后 Compute/Save。
注意官方上游运动服务对执行结果检查较弱，本机自动脚本增加了实测到位检查。

## 3. 自动采样程序（本地封装，调用上游算法和服务）

在已运行第 2 步的另一个终端，先做只规划检查：

```bash
roslaunch rml63_handeye auto_collect.launch
```

这一步不运动、不采样。默认围绕当前 link6 姿态生成上游的 17 个目标位姿，
平移尺度 20 mm，旋转尺度 10°，速度和加速度比例 5%。过大关节绕转、空轨迹会被拒绝。
范围可通过 translation_delta_meters 和 rotation_delta_degrees 修改；较小范围用于调试，
最终精度仍需足够的多轴旋转激励和独立验证，不能只凭采样数量判断。

打印尺寸核对正确、所有姿态均可达且相机/支架空间检查完毕后，实际采样命令：

```bash
roslaunch rml63_handeye auto_collect.launch execute:=true tag_size_verified:=true
```

实际命令会移动真机。不要同时在官方运动 GUI 或示教器上发送运动命令。
自动脚本会：检查故障/使能 -> 规划 -> 执行 -> 核对真实关节到位 ->
检查机器人和标签连续停稳、TF 新鲜 -> 调用 easy_handeye 采样 -> 求解和保存。
遇到标签丢失、反馈停滞、执行失败或实际不到位会停止，不把 rm_control 的成功状态当作到位证明。
不自动清除已有样本；开始新自动运行前需要重新启动标定服务。
程序结束停留在最后采样姿态，不自动回起点。Ctrl+C 可以中止脚本，并请求 MoveIt 停止。

## 包含关节零偏的升级方案

见 [JOINT_CALIBRATION.md](JOINT_CALIBRATION.md)：同步记录原始关节角、视觉位姿、模型和原图，
联合诊断 J2～J5 零偏与手眼外参，通过留出及独立姿态验证判断有效性。仅离线计算，不修改机械零点。
原先只备份末端 TF 的样本不能直接用于这一方案。

## 手动示教采集

先在原 calibrate.launch 终端 Ctrl+C，保留相机和真机驱动。手动模式：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch rml63_handeye manual_collect.launch
```

该模式关闭自动运动后端和运动 GUI，保留官方采样 GUI 与 Tag 调试窗口。
用示教器点动或控制器支持的拖动示教移动，每个姿态停稳 1～2 秒、完整 Tag 持续检测后点击 Take sample。
建议先采 20～30 个有效姿态，覆盖实际使用距离，在 Tag 可见和机械空间允许范围内改变多个旋转轴，加入组合倾斜。
不要只平移、只转 J6 或密集重复接近的姿态。完成后点击 Compute，再点击 Save。
新结果保存到 `~/.ros/easy_handeye/rml63_l515_manual_eye_on_hand.yaml`，不会覆盖之前的自动标定结果。
只读备份节点每 0.5 秒检查官方样本列表，变化时原子保存完整列表：
`~/.ros/rml63_handeye_manual_runs/日期时间/samples.yaml`。
关闭前确认终端的 Manual samples backed up 数量与 GUI 一致。
独立验证时使用本轮结果：

```bash
python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/validate_calibration.py --live \
  --calibration ~/.ros/easy_handeye/rml63_l515_manual_eye_on_hand.yaml
```

## 4. 结果和发布

easy_handeye 结果：`~/.ros/easy_handeye/rml63_l515_eye_on_hand.yaml`。
自动运行原始样本与结果副本：`~/.ros/rml63_handeye_runs/日期时间/`。
保存之后，先用未参与求解的姿态检查固定标定板在 base_link 下的位置是否稳定，再用于抓取。
本次配置完成不代表已经采样、求解或通过精度验证。

### 验证保存结果

先检查最近一轮训练样本的内部残差（不会控制机械臂）：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/validate_calibration.py
```

独立验证：保留真机驱动、相机和 Tag 检测程序运行，标定板和相机安装保持固定。
不要再次运行自动采样；在新终端运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/validate_calibration.py --live
```

自己通过现有示教器或 MoveIt 换到未参与标定的新姿态，停稳且完整 Tag 可见后在验证终端按 Enter。
采集 5～6 个不同姿态，至少绕两条轴改变朝向，覆盖实际抓取使用的距离和视野。
每次自动检查停稳、等待新的图像并采集至少 15 帧和 0.6 秒；相邻观测用同一图像时间戳查询机器人 TF。
重复姿态（与任意已采姿态均小于 10 mm 且小于 5°）会被拒绝。输入 q 结束。
该脚本不发送任何运动命令，不需要先发布手眼 TF。

计算公式 `T_base_tag = T_base_link6 * T_link6_camera * T_camera_tag`，固定 Tag 的结果应保持一致。
报告同时显示相对均值的位置 RMS/最大值，以及任意两姿态间的最大位置差 `position_max_pair_mm`。
每个姿态平均后的变换参与姿态间误差统计；单姿态帧间抖动另存于 within_pose，不能当作手眼误差。
报告和全部原始观测自动保存到 `~/.ros/rml63_handeye_validations/日期时间/validation.yaml`。
是否达到要求须根据抓取容差判断；几毫米的散布不能证明绝对抓取精度，也不能排除标签尺寸或 TCP 偏差。
最后还应测量物体/标定点的独立基座坐标，或以低速、留间隙的接近测试核实相机定位与夹爪 TCP；先不直接闭爪抓取。

本机已部署联合补偿配置，启动相机定位 TF：

```bash
roslaunch rml63_handeye publish.launch
```

默认加载 config/active_joint_calibration.yaml，同时应用校正 FK 与匹配手眼外参，发布
base_link -> rml63_link6_calibrated -> camera_link，保留 RealSense 原生 optical 链。
名义机器人 TF、MoveIt 和控制器零点不变。修改/导出参数、重启及适用范围见
[JOINT_CALIBRATION.md](JOINT_CALIBRATION.md#已部署的相机定位补偿与修改方法)。
旧的纯手眼发布方法需显式 use_joint_offsets:=false，此时才使用 namespace_prefix 对应的 easy_handeye YAML。

## 源码与重编译

三个上游 Git 仓库未作源码修改，版本记录在 ../SOURCES.yaml。
本地封装的自动脚本使用上游 CalibrationMovements._compute_poses_around_state 生成姿态，
使用 easy_handeye 的采样/求解/保存服务；自动循环和到位/稳定性检查是本地实现。

```bash
cd ~/realman_RML63
catkin build apriltag_ros easy_handeye rqt_easy_handeye rml63_handeye -j4 -p2
```

## 图像发布与自动调试窗口

实际流程：原始彩色图像 + 原始 CameraInfo -> OpenCV 去畸变 -> 发布校正图像与匹配 CameraInfo ->
apriltag_ros 检测 -> 发布识别叠加图像和位姿。检测器不直接订阅原始畸变图像。

| 话题 | 内容 |
| --- | --- |
| /camera/color/image_raw | L515 原始彩色图像 |
| /rml63_apriltag/image_rect_color | 已去畸变的彩色图像，供检测器使用 |
| /rml63_apriltag/camera_info_rect | 校正后的 K/P，D 为零，与校正图像匹配 |
| /rml63_apriltag/tag_detections_image | 校正图像上的 Tag 识别叠加画面 |
| /rml63_apriltag/tag_detections | ID、物理尺寸及位姿 |

calibrate.launch 和 detect.launch 默认 show_debug:=true，自动显示双画面。
无显示器环境可传 show_debug:=false；所有图像话题仍继续发布。
调试窗口只是订阅图像，不改变图像处理、运动或采样流程。

## Tag 位姿延迟与等待策略

曾发现 L515 使用 hardware_clock 且 global_time_enabled=false 时，时间戳滞后会从约 0.6 s 增长到 1.4 s。
现 detect.launch/calibrate.launch 默认启动 sync_camera_time.py，通过 SDK dynamic_reconfigure
启用 RGB、深度和运动传感器的全局时间；RGB 配置失败会停止该标定 launch。
修复后实测原图时间戳延迟约 0.036 s，Tag 检测约 0.052 s，metadata clock_domain=global_time。
相机重启后请重新启动标定 launch，使时钟设置重新生效；改变时间域后必须开始新的采样轮次。
自动脚本默认 tf_max_age_sec:=0.8、tf_wait_timeout_sec:=5、settle_timeout_sec:=10。
不是直接使用缓存旧 TF：每次停稳检查先等到采集时间晚于检查开始的新图像，
再要求三个时间戳持续推进、机器人和标签都稳定的观测；冻结/失去更新仍会超时。
短暂超龄会等待新帧，抖动会重新累计连续稳定计数。

自动程序中断后，已有样本文件保留。若开始新一轮完整采样，先在运行 calibrate.launch 的终端 Ctrl+C，
重新启动标定服务以清空该轮的内存样本，再执行自动采样；仅重启 auto_collect.launch 不会清空旧样本。
等待策略测试：python3 tests/test_tf_sampling_gate.py（不连接或移动机械臂）。
