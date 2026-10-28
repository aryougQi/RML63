# RML63 TCP 触碰 AprilTag 中心（ROS1）

通过 ROS MoveIt 的 `arm` 组和现有 `rm_control` 执行，不使用厂商 Python SDK，
也不直接向控制器发布未经规划的笛卡尔命令。

触碰点是 **夹爪闭合时两指尖中心**，用户确认位于 `link6` 的 **+Z 150 mm**。
程序不会开合夹爪。运行前保持夹爪闭合、Tag 和相机安装固定。

当前默认流程：停稳采集 Tag → 规划到距表面 100 mm 的接近点 → 沿法线到中心上方 50 mm → 停下。
默认只预览；`execute:=true` 才通过 ROS 执行。`retract` 默认 false，保持最终位置。
这里“上方”沿 Tag 表面法线朝相机的一侧；Tag 平放在水平桌面时才等同于基座竖直向上。
这是位置控制到达几何目标，不带接触/力检测，不会因碰到纸面而自动停止或补偿误差。

## 构建

```bash
cd ~/realman_RML63
catkin build rml63_tag_touch
source devel/setup.bash
```

## 启动条件

保留 RML63 真机驱动、模型、MoveIt、L515 和 AprilTag 检测节点。
标定检测入口为 `roslaunch rml63_handeye detect.launch`，使用当前实测 Tag 边长配置。
手动退出自动标定/自动采样运动程序后再执行本演示；程序不会替你终止其他节点。
本包不自动启动或重新配置机器人控制器，不改变控制器工具坐标系、零点或夹爪开口。

默认读取 `rml63_handeye/config/active_joint_calibration.yaml`，与
`roslaunch rml63_handeye publish.launch` 使用同一份关节零偏和相机外参。
先重启该发布器，再运行接近程序。两者必须加载同一文件内容；文件指纹不一致会拒绝运行。
手工修改 YAML 后，两者都要重启。不需要重新编译。

程序读取这份配置中的相机外参，按检测图像的时间戳查询校正 FK 的 TF：

```text
T_base_tag = T_base_rml63_link6_calibrated(t) × X_calibrated_tool_optical × T_optical_tag(t)
T_base_link6_goal = T_base_tcp_goal × inverse(T_link6_tcp)
```

关节补偿由发布器以 FK(q+delta) 应用一次；本程序不会再把零偏叠加到反馈或控制命令中，
也不会把新相机外参与未校正的 link6 TF 混用。Tag 检测帧仍是 `rml63_apriltag_0`。
MoveIt 与控制器继续使用原来的名义模型，接近程序的末端规划与到位检查仍基于名义 link6。
本次接入改进的是 Tag 的基座坐标定位，并不等同于已经校正真实 TCP 的运动控制误差；
相机定位的独立验证精度也不能作为真实 5 cm 停靠精度的证明。
Tag 连续采集至少 15 帧和 0.6 秒，检查机器人停稳、检测抖动和时间戳新鲜度。
程序将目标冻结在基座系，运动中不再追踪 Tag；相机被夹爪遮挡不会改变已采集的目标。

## 从头启动（不同终端，已有节点不要重复启动）

每个终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

终端 1，真机驱动、模型、MoveIt 和控制接口：

```bash
roslaunch rml63_tools_description realrobot.launch allow_execution:=true
```

终端 2，L515 彩色流：

```bash
~/.local/bin/l515-camera-ros enable_depth:=false
```

终端 3，Tag 检测与相机画面（当前黑框实测 151 mm）：

```bash
roslaunch rml63_handeye detect.launch tag_size:=0.151 show_debug:=true
```

终端 4，应用 active_joint_calibration.yaml 的成套参数：

```bash
roslaunch rml63_handeye publish.launch
```

终端 5，先预览：

```bash
roslaunch rml63_tag_touch touch.launch
```

退出终端 5 的预览，再执行同一目标（5 cm 停下）：

```bash
roslaunch rml63_tag_touch touch.launch execute:=true
```

兼容旧的纯手眼文件时必须显式设置 `use_joint_offsets:=false` 并指定对应文件：

```bash
roslaunch rml63_tag_touch touch.launch use_joint_offsets:=false \
  calibration_file:=$HOME/.ros/easy_handeye/rml63_l515_manual_eye_on_hand.yaml
```

## 先预览

```bash
source ~/realman_RML63/devel/setup.bash
roslaunch rml63_tag_touch touch.launch
```

默认 `clearance_m:=0.050`，终点是 Tag 中心上方 5 cm。TCP 的 +Z 自动对准表面，
接近点放在相机所在的板面一侧，不依赖 AprilTag 法线正负号。
可设 `align_tcp_z:=false` 保持当前 TCP 朝向。

RViz 的 Fixed Frame 使用 `base_link`。添加：

- MotionPlanning/Trajectory 或 DisplayTrajectory，话题 `/rml63_touch_tag/display_planned_path`。
- MarkerArray，话题 `/rml63_touch_tag/markers`：绿色是接近点，红色是终点。
- Pose，话题 `/rml63_touch_tag/target_tcp`，检查 TCP 目标坐标轴。

节点默认保持运行以便查看。看完按 Ctrl+C 退出预览，再启动执行命令。
执行时会重新采集并规划；预览窗口里的结果并不是下一次运行直接复用的缓存轨迹。

## 执行

到 Tag 中心上方 5 cm 并停下：

```bash
roslaunch rml63_tag_touch touch.launch execute:=true
```

如以后需要真正触碰：当前手眼结果曾报告最大位置残差 9.30 mm，先核对目标、
闭合指尖 TCP 和实际间隙。显式设为零间隙，到达 Tag 中心并退回：

```bash
roslaunch rml63_tag_touch touch.launch execute:=true clearance_m:=0.0 retract:=true
```

若要停在 Tag 中心而不退回：

```bash
roslaunch rml63_tag_touch touch.launch execute:=true clearance_m:=0.0
```

速度、加速度比例默认均为 0.02，配置在 `config/touch.yaml`。
启动参数 `velocity_scaling` 可覆盖速度比例，例如以 20% 速度到达 Tag 中心并停住：

```bash
roslaunch rml63_tag_touch touch.launch execute:=true clearance_m:=0.0 velocity_scaling:=0.20
```

先预览时去掉 `execute:=true`。这仍会先到 10 cm 接近点，再沿法线到中心。
`0.20` 是 MoveIt 关节速度限值的比例，并非固定的 TCP 线速度；加速度仍为 0.02。

`max_target_distance_m` 默认 0.40 m，限制当前 TCP 到接近点和终点的直线距离，
不是机器人基座到 Tag 的距离。超限时报出两个实际距离，尚未开始规划或发送运动命令。
若当前 TCP 确实距离 Tag 约 0.61 m，核对坐标后可以先以 0.70 m 限制预览：

```bash
roslaunch rml63_tag_touch touch.launch clearance_m:=0.0 \
  velocity_scaling:=0.20 max_target_distance_m:=0.70
```

确认目标和轨迹后，再在该命令中添加 `execute:=true`。提高距离限制不会跳过碰撞、
关节跨度、直线完成度和机器人状态检查；也不保证该位置可达。
两指尖中心有横向偏置时，可用 `tcp_x`、`tcp_y`、`tcp_z`（单位米）指定实际 TCP。
原模型的夹爪安装 yaw 为 -60°，但本演示的 TCP 轴定义与 link6 平行，
其原点仅沿 link6 +Z 平移 150 mm；无需把安装 yaw 误加到这个轴向偏移上。

全部运动段规划成功后才会开始执行。直线段要求 100% 完成且启用模型碰撞检查；
会拒绝关节跳变、超出目标距离限制、陈旧 TF、机器人未到位或其他自动采样程序正在运行。
运动失败不会重试、继续触碰或自动回撤。预览节点退出不会向其他运动程序发送停止指令。
碰撞检查只覆盖当前规划场景中已建模的物体，相机、支架、纸板和线缆仍需实物核对。

## 验证（不连接或移动机器人）

```bash
source ~/realman_RML63/devel/setup.bash
python3 -m unittest discover -s src/usr/demo/rml63_tag_touch/tests -v
```
