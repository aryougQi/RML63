# 与原有 RealSense ROS 相机节点统一

检测与抓取现在直接订阅你之前使用的 `realsense2_camera/rs_camera.launch`，不再使用独立的 SDK-to-ROS 发布节点。
`depth_handle_grasp/camera.launch` 只是原驱动的启动参数封装，不会再启动第二套相机实现。
驱动版本针对当前主机的 ROS Noetic realsense2_camera 2.3.2。相机一处启动，RViz、原有程序与横杆检测共享话题。

## 终端一：启动相机（保持运行）

```bash
source /opt/ros/noetic/setup.bash
source /home/pc/realman_RML63/devel/setup.bash
cd /home/pc/realman_RML63
roslaunch depth_handle_grasp camera.launch
```

已有原相机节点正常运行时跳过这一步，不重复启动。若旧相机 launch 终端还挂着但节点已退出，先在旧终端 Ctrl+C，再重新启动。
关闭直接占用相机的 Viewer 和 SDK live/capture 程序。

该封装实际 include 原来的 `realsense2_camera/rs_camera.launch`，保留以下原有设置：
L515、深度 320×240@30、彩色 640×480@30、enable_confidence=false，并指定相机序列号。
它将 `/camera/l500_depth_sensor/min_distance` 设为 250 mm，让约 400 mm 处的横杆获得有效深度。
使用标准驱动管理相机设置，不再执行先前自定义相机节点的退出恢复逻辑。

也可以继续使用你之前的原命令（与上面的封装二选一）：

```bash
roslaunch realsense2_camera rs_camera.launch \
  device_type:=l515 depth_width:=320 depth_height:=240 depth_fps:=30 \
  color_width:=640 color_height:=480 color_fps:=30 enable_confidence:=false
```

如果直接使用原命令，且最小距离仍是 490 mm，请在相机启动后另一个已 source ROS 的终端中执行：

```bash
python3 -c "import rospy; from dynamic_reconfigure.client import Client; rospy.init_node('configure_camera_min'); Client('/camera/l500_depth_sensor', timeout=5).update_configuration({'min_distance': 250})"
```

深度订阅使用原生 `/camera/depth/image_rect_raw`，不是 `/camera/aligned_depth_to_color/...`。
深度后处理 filters 保持空，不能用补洞深度绕过无效测量。

## 原有话题与服务

| 接口 | 用途 |
|---|---|
| /camera/depth/image_rect_raw | 原生深度像素网格，16UC1，驱动已转换为毫米 |
| /camera/depth/camera_info | 同帧深度内参 |
| /camera/depth/metadata | realsense2_camera/Metadata，原始帧号、硬件时间域和时间戳 |
| /camera/extrinsics/depth_to_color | 工厂深度到彩色外参，SDK 数组按列优先解析 |
| /camera/realsense2_camera/device_info | 核验 L515 型号和序列号 |
| /camera/color/image_raw | 彩色图像，供 RViz 与原有程序查看 |

重要：标准驱动的 uint16 深度单位是 **0.001 米/单位（毫米）**，不能套用 SDK 原生 Z16 的单位。
保存的 depth_raw.npy 是 ROS 收到的数值原样保存，不是灰度截图，也不是恢复出的 SDK 原始 Z16。
驱动可能已将 SDK 的亚毫米单位量化到毫米，因此不承诺与旧 SDK 采集具有完全相同的测量精度。
每帧 metadata.json 明确记录 ROS 存储单位和来源，原始 RealSense Metadata 也保留。

## 终端二：机械臂与 MoveIt（已运行则跳过）

```bash
source /opt/ros/noetic/setup.bash
source /home/pc/realman_RML63/devel/setup.bash
cd /home/pc/realman_RML63
roslaunch rml63_tools_description realrobot.launch allow_execution:=true
```

## 终端三：验证检测和路径

```bash
source /opt/ros/noetic/setup.bash
source /home/pc/realman_RML63/devel/setup.bash
cd /home/pc/realman_RML63

# 看帧率，之后 Ctrl+C 结束此监视命令
rostopic hz /camera/depth/image_rect_raw

# 查看设备和一份帧元数据
rosservice call /camera/realsense2_camera/device_info
rostopic echo -n 1 /camera/depth/metadata

# 只做视觉检测，不控制机械臂
python3 src/usr/demo/depth_handle_grasp/scripts/detect_ros.py

# 发布位姿，不规划/不运动
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py --publish-only --hold-seconds 120

# 完整规划，不运动
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py
```

RViz 添加 Image，选择 `/camera/color/image_raw`，可在检测时同时看彩色画面。
效果图在 `src/usr/depth_handle_detect/outputs/ros_<时间>/handle_detection.png`；
抓取入口的图在 `outputs/robot_grasp/<时间>/handle_detection.png`。数值结果在 reports，采集数据在 data。
保持横杆在已配置的 ROI 和深度范围内；明显移动后需要重新检查 ROI、深度范围和桌面区域。

## 实际抓取和返回

先通过上述检测和路径检查。当前速度为 15%、加速度为 10%，这套标准 ROS 相机入口尚未实机抓取验证。
现场人员看护、篮子静止、运动区域清空后，手动执行：

```bash
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py --execute

# 成功完成小幅抬升且仍保持夹持时，放回篮子并返回：
# 抓取与返回之间不要运行其他 grasp_demo.py，避免覆盖最新记录索引。
RUN_DIR=$(python3 -c 'import json; print(json.load(open("src/usr/demo/depth_handle_grasp/reports/latest_run.json"))["run_directory"])')
python3 src/usr/demo/depth_handle_grasp/scripts/return_demo.py --run "$RUN_DIR" --execute
```

## 数据有效性与验证范围

- 深度、CameraInfo、Metadata 按精确 ROS 时间戳配对；不用 Header.seq 冒充硬件帧号。
- 检查单一发布者、设备序列号、工厂外参、原生深度坐标系、数据新鲜度和连续硬件帧号。
- 抓取前持续采集真实关节状态，拒绝采集期间移动、反馈间隙和无法包围图像时刻的情况。
- 使用驱动转换后的 ROS 图像时间戳，不声称实现硬件同步；仅支持静止采集。
- SDK→ROS 单位、外参方向和原消息类型已通过合成 ROS 传输测试；真实 ROS 相机连续 30 帧检测及不运动位姿发布已验证，新的实机抓取仍待手动验证。
- 没有修改 pnp_detect，不发布虚拟关节状态，不关闭碰撞检查，不自动执行机械臂动作。

测试：

```bash
python3 -m unittest discover -s src/usr/demo/depth_handle_grasp/tests -v
# 需已有 ROS master；只发布测试命名空间中的合成图像与模拟设备服务
python3 src/usr/demo/depth_handle_grasp/tests/ros_transport_check.py
```

## 已修复：型号名称与时间戳漂移

设备型号检查兼容 `intel_realsense_l515`，序列号仍严格匹配。
根因是相机 `global_time_enabled=false` 时，旧驱动只使用启动时的一次硬件时钟偏移，运行中偏差会持续积累。
本项目 `camera.launch` 已在深度与 RGB 传感器中配置 `global_time_enabled=true`，由 SDK 持续校正硬件时钟到主机时钟。
采集入口要求实际帧 `clock_domain=global_time`，恢复 header 最大年龄 0.5 秒，同时检查主机到达时间年龄不超过 0.5 秒。
不是把接收时刻写入 header，也不继续扩大年龄门限。静止关节配对和连续帧检查仍生效。

如果直接使用你原来的 rs_camera.launch 命令启动相机，可在相机运行时执行以下命令开启同步，无需重启相机：

```bash
python3 src/usr/demo/depth_handle_grasp/scripts/configure_camera_clock.py
```

之后等待几秒，再运行不运动验证：

```bash
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py --publish-only --hold-seconds 300
```

修改了正在采集的相机时钟后，本次未完成的采集需要重新运行，不能跨时钟域混合帧。
重启 ROS master 后，继续使用本项目 camera.launch 可自动加载同步参数；直接使用原命令时重新执行配置脚本。

修复后实测记录：`reports/clock_sync/20261024_174700/summary.json`。连续 120 秒共 3604 帧均为 global_time，header 年龄 16.29–35.63 ms，平均 27.53 ms；首尾各 10 秒均值分别 28.95 / 28.43 ms，未见此前的持续累积趋势。相同配置已通过真实 30 帧检测与不运动位姿发布。本次没有发送机械臂动作。

首帧时间配对：订阅建立前可能已经生成了深度帧。现在正式采集的起点取请求时刻与第一份真实关节反馈时间的较晚者，自动跳过此前帧，再采集连续 30 帧；首末关节覆盖、静止性和新鲜度检查保持生效。已通过专门的 ROS 启动时序回归测试，以及连续三次真实相机不运动位姿发布（reports/20261024_175035、175040、175045）。

## 把手多候选与向旁边放置（已接入原入口）

继续使用原相机、标定、MoveIt 与 rm_driver。无需识别整个篮体，也不需要填写绝对放置目标。

```bash
source /home/pc/realman_RML63/devel/setup.bash
cd /home/pc/realman_RML63
/usr/bin/python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py
```

默认：原深度检测→沿把手拟合中心线采样→接触/几何评分→前 K 个位置规划→选择完整路径代价最低者。
完整路径为预抓取、下探、抬高 3 cm、沿 base_link −Y 侧移 5 cm、下降到原抓取高度、松爪后向上撤离 5 cm。
所有运动路径先规划，通过后才可能执行。默认不发送机械臂或夹爪动作。

- `config/grasp.yaml` 中 `side_place_offset_base_m`、`side_place_lift_m`、`side_place_retreat_m` 控制相对放置。
- 采样、评分及前 K 个位置数量沿用 `config/multigrasp.yaml` 的相关参数，其篮体/绝对放置字段不参与此流程。
- 只识别、输出候选和发布姿态：追加 `--publish-only`。
- 保留原小幅抬升模式：追加 `--lift-only`。
- 输出新增 `handle_candidates.json`、`handle_planning.json`、`side_place_targets.json` 和六段轨迹。

本入口继续使用原标定 RobotSession 规划器，不把它称为 MTC；独立 MTC 扩展仍见 `MULTIGRASP_REPORT.md`。
识别把手足以定义相对放置目标，但不会自动验证整篮随动后的碰撞包络或放置面的支撑情况。
真实侧移执行必须同时具备 `--execute` 和经过现场核实的 `side_place_scene_verified: true`；默认该项为 false。
抓取接触丢失、到位校验失败或超时会中止，不自动松爪/恢复。到达放下位姿且接触反馈正常后才松爪、核验张开并撤离。
本次仅运行无硬件的集成/回归测试，未运行真实抓放。

### 规划搜索耗时（2026-10-27 优化）

原入口默认 `handle_planning_top_k: 3`、`planning_total_timeout_sec: 20`、
`planning_candidate_timeout_sec: 6`、`planning_max_approach_attempts: 4`。
这些是搜索预算，不保证在预算内一定有可行解；采集、候选评分和场景构建耗时另计。
预算在 IK、轨迹状态检查及调用规划器前后检查；正在等待的 ROS 服务调用不能被该预算强行中断。
所有保留的解仍须通过完整碰撞/轨迹检查。服务断开会报错退出，不自动发送运动或重试。

新增后续位姿预检查，避免已知搬运/放置终点不可行时仍重复规划预抓取和下探；
碰撞服务采用持久连接减少重复建连。终端实时打印 `[planning ...]`，
`planning_progress.json` 保存当前阶段，`planning_search.json` 记录尝试数、耗时和非穷举搜索范围。
优化后单元/集成测试 50 项通过，构建成功；当前实机场景的新版耗时仍待重新运行验证。

### 闭合反馈误判后的明确续行

2026-10-27 已修复：`mode=2` 也会出现在张开到位时，不能单独表示空夹。
闭合现在要求连续 3 次稳定 `mode=6` 接触；空夹判据则同时要求连续 3 次到位且实际开度接近零。
没有接触或反馈异常仍中止，不自动松开或重试。

`resume_side_place.py` 仅支持“已完成预抓取和下探、随后报告旧闭合误判”的记录。
需要该记录中的操作员确认 `operator_contact_confirmation.json`，且当前关节必须仍匹配下探终点。
它核对标定/TCP、实际夹持开度、静止状态，使用原采集关节重建原观测场景，并重新验证全部剩余轨迹。
默认只验证，不重新开闭夹爪；显式 `--execute` 才从抬升继续，到位后松开并撤离。
它不能用于未知接触、移动过的机器人或已经抬升/搬运过的记录。

本次 `20261027_164248` 操作员已确认夹住横杆且篮子仍有支撑；
纯验证记录 `reports/20261027_164248/resume_20261027_164938/` 四段剩余轨迹全部通过，无运动。

```bash
python3 src/usr/demo/depth_handle_grasp/scripts/resume_side_place.py \
  --run src/usr/demo/depth_handle_grasp/reports/20261027_164248 \
  --config /tmp/rml63_side_place.yaml

# 当前状态未改变且现场条件仍满足时，显式执行剩余阶段：
python3 src/usr/demo/depth_handle_grasp/scripts/resume_side_place.py \
  --run src/usr/demo/depth_handle_grasp/reports/20261027_164248 \
  --config /tmp/rml63_side_place.yaml --execute
```
