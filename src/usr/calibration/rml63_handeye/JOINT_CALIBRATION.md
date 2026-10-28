# 关节零偏与手眼外参联合诊断

本方案只采集数据并离线计算，不修改控制器零点、URDF、joint_states 或 MoveIt。
现有 easy_handeye 备份没有对应的关节角和时间戳，不能可靠恢复原机械臂关节分支，因此不能直接用于零偏估计。
不要用逆运动学猜出的关节角替代原始编码器读数。

## 模型和可观测性

拟合 `FK(q_reported + delta) * X * T_camera_tag = Y`。
X 为 link6 到彩色光学坐标系，Y 为固定 Tag 在基座坐标系中的位姿。
URDF 使用采集时真实运行的 robot_description 快照，不更改连杆长度。

单个固定板和末端相机无法独立区分：

- J1 零偏与 Y 中绕首关节轴的固定变换。
- J6 零偏与 X 中末关节轴的固定变换。

因此固定 delta1=delta6=0 是选择坐标规范，绝不表示已经测出这两轴的真实物理零点。
只联合估计 delta2～delta5、X 和 Y，共 16 个参数。
要独立标定六个真实机械零点，需额外已知的板到基座关系、相机到法兰关系、机械基准或外部测量。
相关研究讨论参数可观测性与联合估计：[KIT 博士研究](https://publikationen.bibliothek.kit.edu/1000174827/154979691)、
[机器人标定教材](https://motion.cs.illinois.edu/RoboticSystems/Calibration.html)。

默认参数限制为每轴 ±3°，零均值先验标准差 1°；这是保守的诊断设置，不是制造商给定的机械零偏范围。
测量权重默认为位置 1 mm、旋转 0.5°，不是经认证的传感器标准差。拟合使用 soft_l1 鲁棒损失。
报告数据项雅可比的秩和缩放后条件数，先验不参与可观测性计算。
参数碰到边界、不同留出折估计不一致或数据秩不足时，不能解释为可靠的关节零偏。
视觉畸变、板不平、柔性和连杆几何误差仍可能被零偏参数吸收。

## 采样步骤

保留真机驱动、相机和 manual_collect.launch 运行，由后者提供矫正图像、Tag 检测和调试窗口。
新工具不调用官方 Take sample，不需要在 easy_handeye GUI 中点击采样或求解。
直接运行采集脚本会自动补启动缺失的检测/调试画面；无图形环境用 --no-view 并事先单独启动检测。

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/collect_joint_calibration.py
```

自己示教移动，每姿态停稳 1～2 秒后在终端按 Enter。
每姿态至少记录 15 帧/0.6 秒，新视觉时间戳和 TF 检查复用现有验证工具；
每帧用前后两条 joint_states 插值获得相机曝光时刻的关节角，间隔超过 0.15 秒或关节跳变则拒绝。
整个姿态关节变化超过 0.2°则拒绝；名义 FK 与同时间的 TF 相差超过 0.5 mm 或 0.1°也拒绝。
原图尽量与中间一帧匹配（不超过 40 ms），连同原始内参一起保存；原图未保存会明确显示。

建议训练 30～40 个有效姿态，重点让 J2～J5 分别发生独立变化，组合不同肩、肘、腕配置。
在机械空间、关节限位和 Tag 可见范围内，争取这些轴各有约 20～30°以上跨度；这是采样目标，不是自动运动命令。
末端朝向不同并不保证每个待估关节都受到独立激励。终端会逐次打印六轴跨度。
先覆盖实际抓取区域，可从相机到板约 0.35～0.60 m 开始；保持 Tag 完整且有清晰边缘，避免接近掠视角。
例如：几组不同位置分别向左右、前后倾斜，并加入组合腕姿态；不要只转 J6。
不要为了追求大角度而扩大到实际不用、检测明显退化的区域。

每采约 5～10 个姿态，可示教回到同一参考关节配置，输入 r 保存重复性检查数据。
r 数据保留但不参与拟合；后续可比较相同配置下的漂移，判断安装、回差或检测重复性。
普通 Enter 拒绝与已有样本六轴均相差不足 2°的重复配置。
输入 q 结束，数据在 `~/.ros/rml63_joint_calibration_runs/日期时间/dataset.yaml`。
同目录保存 robot.urdf、原图和 CameraInfo。不要编辑角度，单位为弧度。

固定 Tag 和相机安装不变，再单独采 8～12 个未用于拟合的验证姿态：

```bash
python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/collect_joint_calibration.py --role validation
```

验证姿态覆盖实际作业范围，验证数据不参与拟合。程序按姿态分组，绝不会把同姿态不同帧拆到训练和验证。

## 离线优化

将下列两个路径替换为各次采集结束后打印的实际路径：

```bash
python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/optimize_joint_calibration.py \
  ~/.ros/rml63_joint_calibration_runs/训练目录/dataset.yaml \
  --validation ~/.ros/rml63_joint_calibration_runs/验证目录/dataset.yaml
```

可以先不传 --validation，检查五折留出诊断，但不能据此替代新姿态的独立验证。
对照组与联合组使用相同数据、位置/旋转权重、鲁棒损失及 X/Y 自由度，唯一差别是是否开放中间四轴零偏。
每折从头只拟合训练姿态；验证误差相对训练时拟合的 Y 计算，不重新平移对齐或拟合验证集。
报告为训练目录下 joint_optimization_report.yaml，包含：

- 两组的训练和按姿态五折留出误差，完整拟合结果和零偏角度。
- 各折零偏估计及标准差、边界状态、数据雅可比秩/条件数。
- 若提供独立数据，则列出各组独立验证误差。
- 数据和模型哈希、关节范围、参数权重，便于复核。

先看留出及独立验证是否持续改善，再看拟合误差；若只改善训练误差，不能认定关节零偏是根因。
即使改善，也只能称为本配置下的有效补偿参数。物理零点仍需独立机械基准确认。

## 应用限制

本工具不会自动应用任何结果。带零偏的 X 绑定的是校正后的 FK，
不能单独把该 X 写进 easy_handeye YAML 再用现有 publish.launch 发布，
否则未校正的 link6 与校正后的相机外参会混用。
经独立验证后，使用下文的校正分支发布器一致使用 `FK(q+delta)` 和对应 X。
规划、控制器和夹爪 TCP 仍需单独核对；不得在驱动和模型中重复补偿。

## 已部署的相机定位补偿与修改方法

当前配置文件：`config/active_joint_calibration.yaml`。
来源为 20261022_193925 的 33 姿态联合拟合，以及 20261022_194846_839381 的 8 姿态独立验证。
独立位置 RMS 1.671 mm、最大 2.408 mm；姿态 RMS 1.422°。这些是固定板一致性指标，不是已测得的绝对抓取精度。

启动顺序：先真机驱动/robot_state_publisher 和 L515 相机，再启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch rml63_handeye publish.launch
```

publish.launch 现在默认 use_joint_offsets:=true，读取上述配置而非 ~/.ros/easy_handeye 的旧矩阵。
也可以启动 calibrated_camera.launch，二者选一个。节点名 /rml63_handeye_tf。
没有修改启动登录/系统服务；重启电脑或关闭该进程后需重新运行启动命令。

发布坐标链为：

```
base_link -> rml63_link6_calibrated -> camera_link -> RealSense 原有相机坐标链
```

第一段动态使用实机 /joint_states 的原始时间戳和 FK(q+delta)，反馈过期不会伪装成实时更新。
第二段为固定的 `X_optical * inverse(T_camera_link_optical)`。
既保留原相机光学坐标定义，也保留名义 base_link -> link6 机器人 TF 分支。
原始 /joint_states、MoveIt 运动学和控制器指令均未改变；这是相机定位补偿的部署，
不是机器人规划/控制全链的零点标定。后续抓取须核对 TCP 并验证实际接近误差。

程序启动时校验基座到 link6 的 URDF 运动链签名；结构变更会拒绝启动。
不允许已有其它 TF 发布器给 camera_link 或校正末端坐标系再加父节点。
勿同时运行旧 static_transform_publisher 或旧手眼发布器。

配置字段：

- joint_offsets_deg：度，按 `q_model = q_measured + delta`，当前 J1/J6 为规范固定零。
- effector_to_camera_optical.translation_m：米，校正末端到彩色光学坐标系的平移。
- effector_to_camera_optical.quaternion_xyzw：同一外参的单位四元数，顺序 x/y/z/w。
- provenance：这套原始参数对应的验证来源。手工编辑参数后，原验证指标不再代表编辑后的结果。

重新标定后推荐成套导出，避免把新零偏和旧外参混用。示例为本次数据，替换路径可导出其它已验证结果：

```bash
python3 ~/realman_RML63/src/usr/calibration/rml63_handeye/scripts/export_joint_calibration.py \
  ~/.ros/rml63_joint_calibration_runs/20261022_193925/joint_optimization_report.yaml \
  ~/.ros/rml63_joint_calibration_runs/20261022_194846_839381/dataset.yaml \
  --output ~/realman_RML63/src/usr/calibration/rml63_handeye/config/active_joint_calibration.yaml
```

导出工具重新读取验证样本并计算独立误差，要求拟合收敛、数据满秩、不触边界且独立位置 RMS 优于对照组。
这些检查不代替具体任务的精度要求。已有配置自动备份到 ~/.ros/rml63_handeye_deployments/日期时间/previous_config.yaml。
更换相机、安装位置或实际机械零点后，应重新采集和验证，不靠手调抵消未知误差。

YAML 只在启动时读取。编辑或导出后重启发布节点：

```bash
rosnode kill /rml63_handeye_tf
roslaunch rml63_handeye publish.launch
```

若需另存配置，启动时可指定 `calibration_file:=配置文件绝对路径`。
停止该节点只停止相机校正 TF 更新，不停止驱动、相机和 MoveIt。
旧的纯手眼方法仍可显式传 use_joint_offsets:=false namespace_prefix:=对应名称，但不会应用本次关节补偿。

测试：`python3 tests/test_joint_calibration.py`。
合成测试验证偏差正负号、保留姿态精度、首尾关节规范自由度，以及先验不能消除真实数据退化。
它们验证软件数学逻辑，不代表本机械臂已测得零偏或提高实机精度。

## 自动采集关节角和视觉数据

真机驱动与 MoveIt（allow_execution:=true）、相机保持运行。已有检测节点时直接复用，
缺失检测或画面时会补启动。初值默认读取已经保存的手动手眼结果，它仅用于优化初值，不是已验证的正确答案。
没有该文件时可用 initial_calibration:=实际文件路径指定另一套相同坐标系的保存结果。

先只规划，不运动、不采样：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch rml63_handeye auto_collect_joints.launch
```

确认当前 Tag 完整可见、附近运动空间合适后，实际运行：

```bash
roslaunch rml63_handeye auto_collect_joints.launch execute:=true tag_size_verified:=true
```

默认 20°、0.04 m，保留上游 17 姿态并增加 12 个组合旋转及 4 个三维平移，共 33 姿态。
2026-10-22 在当时起始姿态实测：20°触发某轴轨迹跨度超过 30°而被拒绝；
显式指定 `rotation_delta_degrees:=15 translation_delta_meters:=0.03` 时 33 个目标全部通过只规划检查。
需要时在上述两条命令末尾都加上这两个参数；起始姿态改变后必须重新检查，不能把本次规划结果当作运动保证。
之后一次运行停在首个采样姿态，当前位置作为新中心时，15°触发 joint4 的 30.18°跨度保护。
修复往返路线预检查后，用 `rotation_delta_degrees:=10 translation_delta_meters:=0.025`
从当时当前位置完成 33/33 往返只规划检查。该检查没有移动真机，也不代表零偏估计精度已验证。
速度/加速度限制 5%，每轴轨迹跨度超过 30°仍拒绝（检查包含起始反馈角）。
使用“中心→采样姿态→中心→下一采样姿态”的往返路线，预检查每个目标的去程和返程，
保存预检查选定的关节目标；实际执行按这些固定关节目标从最新反馈重新规划，不重新选末端 IK 分支。
每次执行仍检查轨迹跨度和实际到位，不通过就停止；前置规划不保证 Tag 可见性或现场未建模物体的安全距离。
预检查期间若机械臂位置改变，也会拒绝开始执行。preflight_plan.yaml 保存中心和各采样关节目标。
实际不到位、故障、Tag 丢失、同步/停稳检查失败会停止。结束停在最后姿态，不自动回起点。
不同时运行手动采集或其它运动程序。相机和线缆几何未完整纳入碰撞模型，需人工核对工作空间。
默认完成后自动运行离线优化，报告仍是诊断候选值，不修改 ROS 模型或控制器。
参数 optimize_after_collection:=false 可只采集，之后手动运行离线优化。
数据和报告在 ~/.ros/rml63_joint_calibration_runs/日期时间/。
即使 33 姿态采完，也需查看各关节跨度、可观测性和留出误差，不能仅凭姿态数量认为零偏可靠。
独立验证仍应另采 8～12 个新姿态，使用 --role validation，再按前述 --validation 路径比较。
