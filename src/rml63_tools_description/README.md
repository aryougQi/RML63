# RML63 + 官方 EG2-4C2 模型

## 接入依据

- 官方教程：[RM65-B 添加夹持器和转接板 URDF 使用说明](https://bbs.realman-robotics.cn/question/160.html)。
  采用 `link6 -> fixed_plate_link -> 夹爪底座` 的固定安装链，根据实际装配核对 xyz/rpy。
  教程示例为 RM65-B；不能复制其偏移量到 RML63。
- 官方模型：[RealManRobot/rm_models](https://github.com/RealManRobot/rm_models/tree/main/thirdparty)。
  使用官方分件 STL 和 URDF，保留原 link、关节轴、限位和五个 mimic 关系。
  原文件、固定提交和 Apache-2.0 许可保存在 upstream/。
- 开源交叉参考：[fiveages-sim/inspire_description](https://github.com/fiveages-sim/robot-descriptions-common/tree/main/gripper/inspire_description)。
  其 EG2-4C2 也使用主关节和 mimic 联动；该项目为 ROS2，本包使用 ROS1 catkin，无 ROS2 Control 依赖。
- 官方转接板图纸：upstream/RMU01020_EG2-4C2_adapter.pdf。尚未确认该板与你实物一致。

这是按官方资料制作的本地集成包，不是厂商发布的一键 RML63+夹爪启动包。

## 独立可动预览

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
ROS_MASTER_URI=http://localhost:11312 roslaunch -p 11312 rml63_tools_description display.launch
```

独立 ROS master 防止虚拟关节角进入真机环境。
GUI 的 `4C2_Joint1` 滑块控制模型开合，五个 mimic 关节随动；不会发送实体运动指令。
主关节角范围 0..0.82 弧度，不是毫米或驱动开口位置值。

## 安装参数

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| plate_xyz, plate_rpy | link6 到转接板坐标系，米/弧度 | 0 0 0 |
| mount_xyz | 转接板到夹爪底座的有效平移（米） | 0 0 0.006909 |
| mount_rpy | 夹爪安装旋转（弧度） | 0 0 -1.0471975511965976（-60°） |
| RM63_type | 机械臂本体 II/III 型 | RM63II；III 型传 RM63III |
| link6_type | 普通末端/一体化六维力末端 | Link6；后者传 Link6_6fb |
| fixed_gripper | 预览是否固定开合姿态 | false |
| gripper_angle | 固定模式官方连杆角，弧度 | 0 |

默认安装值已按用户测量更新（见下节），仍需实物核对。RViz 勾选 TF 后查看 link6、
fixed_plate_link 和 4C2_baselink 坐标轴，以实际测量或装配 CAD 确定偏移与旋转。
机械臂本体 II/III 型不等同于控制器第三/第四代。

例如传入 `plate_xyz:="0 0 0.01" mount_rpy:="0 0 1.57079632679"` 可演示厚度和旋转参数；
这些只是语法示例，不能直接作为你的实物安装值。

## 真机反馈与 MoveIt

停止旧 demo/bringup/MoveIt 和旧 robot_state_publisher，只保留一个 rm_driver。
若旧 bringup 同时管理驱动，退出后单独重启 rm_driver。在正常 master 上执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
roslaunch rml63_tools_description moveit.launch
```

传入与预览核对后的相同安装参数。默认 allow_execution:=false，仅显示、规划和标定 TF。
MoveIt 模式固定夹爪开口，由 gripper_angle 指定。现有 rm_driver 的 /joint_states 只有六轴机械臂反馈，
不能把夹爪命令或成功返回当作真实开口测量。动态模型与真机同步还须接入实际反馈并验证连杆角映射。
需要执行时须完成装配、碰撞和控制器负载核对，再显式设置 allow_execution:=true，并单独启动 rm_control。

## 未完成的实物核对

- 转接板当前仅有坐标系，没有 visual/collision 几何。
- 用户确认使用原装夹指；尚未建模 L515、相机支架和线缆，不能据此宣称避障模型完整。
- 未设置假定 TCP；眼在手上标定可以用 link6，不必先把 TCP 作为末端标定帧。
- URDF 不会自动设置控制器负载参数；上游惯性不代表你实物总重量和重心。
- 本包不修改真机关节零位或官方限位。
- SRDF 排除夹爪内部接触及安装接触，保留夹爪与其余机械臂碰撞检测。

重新生成适配宏：`python3 scripts/generate_gripper_xacro.py`。
可动分支保留上游关节数据；固定分支用于保持开口不动的标定。

## 当前实测装配参数

用户确认：中心对中心安装、原装夹指、闭合时法兰安装面到指尖 150 mm；
用户在 RViz 与实物核对后确认，相对 link6 的安装绕 Z 角为 -60°，现已应用到正式模型。
J6 角度和夹爪模型本身的轴定义不能简单混为同一个零方向；没有修改 J6 编码器零位。

对官方原始 URDF 和 STL 做前向运动学与网格顶点检查：
- 4C2_Joint1=0 rad 时两指内侧位于 X=0，属于闭合姿态；0.82 rad 时张开。
- 闭合时夹爪底座到最前端的轴向长度为 143.091 mm。
- 因而当前有效安装偏移为 150 - 143.091 = 6.909 mm，沿 link6 的 +Z。

默认 plate_xyz/plate_rpy 为零，mount_xyz 为 0 0 0.006909，mount_rpy 为 0 0 -π/3。
6.909 mm 是从总长反推的有效轴向间距，不是转接板厚度的独立测量。
没有据此生成转接板实体或添加精确抓取 TCP。测量精度和实物轴向对齐仍需检查。
改变夹爪开口时，连杆结构会改变轴向总长度，150 mm 只对应闭合姿态。

## 用户照片参考姿态

独立预览默认使用照片中示教器角度（度）：
J1=0，J2=50，J3=-139.997，J4=-0.001，J5=-89.998，J6=-30.673。
GUI 启动值通过私有 zeros 参数给出，单位仍为弧度；这只是虚拟模型初始位置，不是硬件零点校准。
MoveIt 的 arm 组增加命名目标 photo_reference；选择该目标只设定规划目标，不会自动运动。
照片参考姿态中 J6 的 -30.673° 与模型安装角 -60° 分别记录；安装角由用户目视核对确认。

## 用度数交互核对安装角

当前正式模型已采用用户确认的 -60°。如需重新核对，先关闭旧预览，然后运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
ROS_MASTER_URI=http://localhost:11313 roslaunch -p 11313 rml63_tools_description align_mount.launch
```

新窗口以度数显示夹爪相对 link6 的绝对安装 yaw；六轴保持用户照片角度、夹爪闭合。
输入数值或拖动滑块即可实时调整；可以用快捷按钮比较不同开合朝向。
窗口只在独立本机 11313 master 运行，虚拟安装关节不出现在正常 MoveIt 模型中。
保存按钮仅写 config/mount_alignment_result.yaml，不会自行覆盖正式装配参数。
如果绕 Z 旋转始终不能匹配，应核查夹爪轴线、法兰原点、本体型号，而不是继续改 J6 零位。

角度记录：`config/mount_alignment_result.yaml`。该目视安装方向确认不等同于相机手眼标定或精确 TCP 标定。

## 一条命令接入真机与夹爪模型

```bash
source /opt/ros/noetic/setup.bash
source ~/realman_RML63/devel/setup.bash
ROS_MASTER_URI=http://localhost:11311 roslaunch rml63_tools_description realrobot.launch
```

默认机器人 192.168.1.18，电脑机械臂网口 192.168.1.10，第三代控制器，模型安装角 -60°。
需按本体配置选择 RM63_type 与 link6_type，不根据控制器代际猜本体型号。
统一启动 rm_driver、真实反馈的 robot_state_publisher、MoveIt 和 RViz；不启动虚拟关节发布器。
已有驱动时传 start_driver:=false，避免启动重复节点。不要与旧 bringup/MoveIt 同时运行。
默认禁止运动执行，夹爪模型保持闭合，并非实时开口反馈。相机驱动可在同一 master 独立运行。
当前手眼标定尚未完成，未把未求解的相机安装变换写入机械臂模型。
