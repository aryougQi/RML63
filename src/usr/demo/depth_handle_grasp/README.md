# depth_handle_grasp

RML63 + L515 + 原装 EG2-4C2 的实机抓取 demo，视觉代码保持在独立 depth_handle_detect 项目。
默认只做新采集、发布位姿和规划；只有显式 `--execute` 才发送夹爪及机械臂动作。

RobotSession 默认使用 `first_feasible`，完整路径验证通过即返回；可配置 `best_within_budget` 保留预算内最佳完整解。性能统计、基线对比和现存限制见 [PLANNER_OPTIMIZATION.md](PLANNER_OPTIMIZATION.md)。

## 环境和运行

保持现有唯一 rm_driver、rm_control、MoveIt 与 robot_state_publisher 运行；不要同时运行其他自动运动程序。
L515 统一使用原有 `realsense2_camera/rs_camera.launch`；本 demo 直接订阅其深度、内参、Metadata 与工厂外参，不再打开相机。
请按 [ROS_USAGE.md](ROS_USAGE.md) 启动原相机节点。`camera.launch` 仅封装原驱动参数；已有原节点时无需再启动。
机器人状态、故障、使能和关节反馈来自已有真实驱动，不发布虚拟 joint_states。

```bash
source /opt/ros/noetic/setup.bash
source /home/pc/realman_RML63/devel/setup.bash
cd /home/pc/realman_RML63

# 只采集/验证/发布位姿；不修改规划场景、不运动。保持话题 120 秒供 RViz 查看
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py --publish-only --hold-seconds 120

# 新采集并检查完整六阶段路径；不会控制夹爪/机械臂
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py

# 首次低速试抓：现场看护、运动区域清空、空篮条件下使用
python3 src/usr/demo/depth_handle_grasp/scripts/grasp_demo.py --execute
```

每次运行从 ROS 新接收 30 帧原始深度并验证连续性、重复性；不从昨天的 JSON 重放机器人运动。
系统 Python 运行 ROS，相机使用原有 RealSense ROS 驱动，离线检测由 depth_handle_detect/.venv 子进程运行，避免混用 ROS 二进制依赖。

## 发布的话题

| 话题 | 类型 | 含义 |
|---|---|---|
| /depth_handle_grasp/pose_camera | geometry_msgs/PoseStamped | 本次深度采集坐标系中的抓取位姿 |
| /depth_handle_grasp/pose_base | geometry_msgs/PoseStamped | base_link 中的抓取位姿 |
| /depth_handle_grasp/pregrasp_base | geometry_msgs/PoseStamped | base_link 中的预抓取位姿 |
| /depth_handle_grasp/markers | visualization_msgs/MarkerArray | RViz 目标标记 |
| /depth_handle_grasp/display_planned_path | moveit_msgs/DisplayTrajectory | 已检查的规划轨迹 |

抓取快照相机帧是 `depth_handle_grasp/captured_depth_optical`，通过静态快照 TF 与 base_link 连接。
它明确代表采集时的坐标系，不冒充机械臂运动后的实时相机帧。
消息时间戳取静止采集区间最后一帧深度图像的 ROS 时间，原始硬件时间戳另存元数据；没有把硬件时钟直接当 ROS 时间。
节点退出后不再发布；需要持续在 RViz 查看时使用 --hold-seconds。

## 坐标链与 TCP

```
T_base_depth = FK(q_reported + calibrated_offsets)
               * T_calibrated_flange_color_optical
               * T_color_optical_depth_optical
T_base_tcp = T_base_depth * T_depth_grasp
T_base_flange_goal = T_base_tcp * inverse(T_flange_tcp)
```

手眼文件读取 calibration/rml63_handeye/config/active_joint_calibration.yaml，并校验运行 URDF 运动链哈希。
该标定对应彩色光学坐标；深度到彩色外参来自当前相机驱动的工厂外参话题，并随采集数据保存，不能把两种光学坐标混用。
采集全程关节反馈范围超过 0.1°、反馈中断或无法包围图像时间时拒绝该次静态配对；采集后会检查关节是否改变。

控制器和 MoveIt 没有应用这套关节零偏，因此目标 IK 用配套校正 FK 求解实际报告关节目标，
再通过原有真实轨迹 action 执行报告关节角；不修改控制器零点或全局 URDF。

原装夹指 CAD 与 measured_mount.yaml 给出：闭合指尖到法兰约 150 mm；
16 mm 夹持开口时夹持面中心约 137.15 mm，32 mm 张开时位置也会变化。
代码从两指内侧面的 CAD 中心计算 TCP，不把 150 mm 指尖长度当成夹持中心。
这是 CAD + 已记录安装尺寸的 TCP 估计，不替代接触式 TCP 精标定。

抓取工具轴定义：X 沿木条，Y 为两指闭合方向，Z 为接近方向。
夹爪相差 180° 的两种等价姿态都检查，选关节变化更小的可达分支；优先竖直接近。
`inward_grasp_tilt_candidates_deg` 可配置朝机械臂底座方向的内倾备选角度，当前为 `[0, 5, 10, 15]`。
`prefer_inward_grasp: true` 优先内倾 5/10/15°，再试竖直；都失败后才按
`grasp_approach_azimuth_candidates_deg` 尝试相对内倾方向的 ±45° 斜向和 ±90° 侧向接近。
保持抓取中心不变，检查实测表面/木条轴向约束、两种夹爪方向和完整的预抓取、下探、抬升路径。
只有整条路径通过才选用该角度；候选失败原因记录在 `approach_ik_candidates.json`，选中角度记录在 `selected_targets.json`。
这不会跳过当前状态碰撞检查、增大关节行程、降低间隙要求或自动执行动作。
本次初始姿态的夹爪需转约 80° 对齐，因此预抓取段允许不超过 90° 的单轴行程，首次试抓采用 2% 速度；当前默认参数为 15% 速度、10% 加速度。

## 检查与首次动作

- 默认 15% 速度、10% 加速度，预抓取在杆上方 100 mm；抬升验证只做 10 mm。
- 完整预抓取、校正 IK 直线下探和抬升均先规划，不会边试 IK 边移动。
- 检查名义与校正关节状态的 MoveIt 碰撞、开夹爪 CAD、桌面、观测篮体体素、相机保守包络。
- URDF 中固定闭合的两个夹指与篮体之间的重复检查，由当前阶段的 CAD 替代：接近/下探使用张开宽度，夹持后抬升使用预期接触宽度。实际夹持宽度反馈到达后，再核验抬升路径。其余碰撞对保持原有设置，替代 CAD 对篮体的检查强制启用，原始观测点云 4 mm 间隙检查仍执行。
- 接触区域保留原始障碍点位置，以半径 4 mm 的球表示；较远区域使用带余量体素。所有非目标观测点均保留，避免粗体素占据原本存在的夹持间隙。
- 先用当前机器人模型剔除深度中的自身点，避免把自己的机械臂当成静态篮子障碍。
- 每个轨迹状态检查，预抓取稠密化到不超过 2° 的相邻关节步长后重新限速；直线 IK 检查分支跳变。
- 开夹爪表面与观测点云要求至少 4 mm 间隙；桌面和篮体几何带余量。
- 正式动作时监测新鲜关节反馈、关节故障、对规划路径的偏离及控制 action 结果；异常取消自有目标并发送停止。
- 夹爪通过 ROS 驱动控制，读状态使用只读 JSON 查询；不绕开 rm_control 直接发送机械臂运动 JSON。
- 开爪必须核验实际位置和在线状态；夹持必须出现力控接触状态且开口与横杆宽度相容，才允许小幅抬升。

本项目没有完整观察单视角背面、支架和线缆，当前相机采用保守球形包络，桌面只建模抓取局部范围。
这些检查不是全环境安全认证，首次测试需要现场人员能立即急停；不要关闭实际检查来强行通过。
空篮重量未知，当前低夹持力用于短程试抓，不代表负载能力已验证。

抬升后会核对夹爪接触是否保持；这仍不能独立证明篮子被抬起，需要相机/现场观察确认。
失败后不自动重试、退回、释放或继续抬升，以免在未知接触状态下增加动作。

## 文件

- config/grasp.yaml：首轮保守参数和标定来源。
- scripts/grasp_geometry.py：纯几何、坐标变换和夹指连杆/TCP。
- scripts/grasp_ros.py：ROS 发布、校正 IK、场景与路径检查、真实接口。
- scripts/grasp_demo.py：新采集、规划和可选执行。
- reports/<运行时间>/：采集配对、变换、目标、规划轨迹、执行反馈、夹爪状态与最终状态。
- reports/latest_run.json：最近运行记录索引。

视觉效果图仍只放在 depth_handle_detect/outputs/robot_grasp/<运行名>/handle_detection.png；
原始深度在 depth_handle_detect/data/robot_grasp/，视觉 JSON/CSV 在其 reports/runs/robot_grasp/。

```bash
python3 -m unittest discover -s src/usr/demo/depth_handle_grasp/tests -v
```

真实抓取状态以每次 reports/.../status.json 和现场观察为准；不把“规划通过”或“夹爪命令成功”写成实物抓取成功。

## 已完成的实机试抓（2026-10-24）

运行记录：`reports/20261024_165942/`。原装 EG2-4C2、空篮，使用 32 mm 张开量、竖直下探、力控闭合和 10 mm 小幅抬升。
下探后与抬升后均为夹爪 mode=6（力控接触），无夹爪故障；现场人员明确确认“夹住横杆，篮子已抬起”。
这证明本场景的一次实机试抓成功，不代表不同摆放下的成功率或长时间承载能力已经验证。

该次抓取前 30 帧全部检测成功，中心 XYZ 标准差为 0.515 / 0.109 / 0.243 mm，最大轴线偏差 0.318°。
检测计算平均 15.75 ms/帧（不含采集和文件读写）；该结果是重复性，不是绝对定位精度。
抬升目标为 10 mm，实际离桌高度由现场目视确认，未独立测量。
现场确认记录为 `operator_confirmation.json`，关节反馈推算的 TCP 执行偏差为 `execution_feedback.json`。
此次没有自动放下或松爪；后续动作需要单独规划。

## 放回篮子并返回抓取前位置

仅适用于机器人仍位于指定成功抓取记录的 10 mm 抬升终点、篮子仍保持夹持的情况。
程序核对当前关节、夹爪接触、标定和原规划场景，检查全部反向路径后，
保持夹持下降回原抓取高度，松爪，再退至预抓取点并沿原路径返回采集前关节位置。
它不会在空中松开篮子；若当前位置或夹持状态不符则停止。

```bash
# 在前述 ROS 环境中运行；替换为需要返回的已完成抓取记录
python3 src/usr/demo/depth_handle_grasp/scripts/return_demo.py \
  --run src/usr/demo/depth_handle_grasp/reports/20261024_165942 --execute
```

不带 `--execute` 只核验与规划。返回记录保存在原运行目录的 `return_<时间>/`。

速度参数更新：后续抓取和返回轨迹都会按当前配置重新计时；已完成的 2% 和 10% 速度实验记录保持原样。15% 速度及加速度尚未进行实机试抓验证。

原有 RealSense ROS 相机兼容入口已通过模拟消息传输与拒绝跳帧/重启测试；真实 ROS 相机与 15% 速度抓取组合待手动验证。此前成功实机试抓使用 SDK 独立采集和 2% 速度，不能视作本次修改已验证。

标准 RealSense ROS 驱动将深度图转换为 uint16 毫米，已显式按 0.001 米/单位保存与计算，不再使用此前自定义发布节点的 SDK 原始单位。

时间同步：`camera.launch` 已启用深度与 RGB 的 `global_time_enabled`。使用原相机启动命令时，可运行 `scripts/configure_camera_clock.py` 配置现有节点；采集要求实际 global_time 帧，时间年龄门限保持 0.5 秒。

## 多候选抓取与 MTC 扩展

候选采样、接触来源标记、完整 MTC 规划、局部依赖和实际测试结果见 [MULTIGRASP_REPORT.md](MULTIGRASP_REPORT.md)。新入口默认仅规划，完整硬件执行尚未验证并保持禁用；原小幅抬升流程保留。

## 已接入原流程的侧移放置

直接运行 `scripts/grasp_demo.py`：把手多候选评分→比较完整路径→抓取→抬高 3 cm→沿 base_link −Y 侧移 5 cm→放下→撤离。默认只规划，无需篮体位姿或绝对放置目标；`--publish-only` 只识别，`--lift-only` 保留原抬升模式。参数和执行条件见 [ROS_USAGE.md](ROS_USAGE.md)。
