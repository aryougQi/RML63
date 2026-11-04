# fixed_grasp 精简运行工程

适用于当前 RML63 + EG2-4C2 装配及已示教的两套点位，ROS 1 Noetic / Ubuntu 20.04 / 系统 Python 3。
仅包含 fixed_grasp 和必需的驱动、消息、控制器、RML63 模型、MoveIt 配置。
不包含相机驱动、视觉识别、AprilTag、easy_handeye、标定采集/拟合程序、历史报告和编译缓存。
现有 M20 正式/手动测试接口和点位示教工具保留。
随包保留 `src/usr/ml.md` 操作指令笔记和 `src/usr/fixed_grasp/ARM_SIDE_FOR_M20.md` 对接说明。
指令文档中的原工程路径已改为 `$FIXED_GRASP_WS`，在工程根目录执行 `export FIXED_GRASP_WS="$PWD"` 后使用。
本次只整理和打包源码，没有执行编译、单元测试或实机运行。

## 编译

解压后进入本目录，在仅加载 ROS Noetic 的新终端执行：

```bash
source /opt/ros/noetic/setup.bash
# 新电脑缺少依赖时，先用已初始化的 rosdep 安装：
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
bash build.sh
```

源码包不包含 ROS、系统依赖或旧 devel/build，需在目标电脑重新编译。

## 启动与抓取

终端一，在工程根目录运行；启动会给末端上 24V 并打开空载夹爪：

```bash
bash start.sh --confirm-empty
# 无显示器可添加 --no-rviz
# 本机网卡 IP 不同可添加 --udp-ip 192.168.1.10
```

终端二，在同一个工程根目录运行：

```bash
bash client.sh --grasp 1 --full-transfer            # 规划预览
bash client.sh --grasp 1 --full-transfer --execute  # 第一块完整抓放并回 Home
bash client.sh --grasp 2 --full-transfer --execute  # 第二块完整抓放并回 Home
bash client.sh --home                             # 回 Home 规划预览
bash client.sh --home --execute                   # 实际回 Home
```

M20 模式：终端一用 `bash start.sh --confirm-empty --m20`；
手动测试用 `bash start.sh --confirm-empty --m20-test`。
详细话题和事务规则见 `src/usr/fixed_grasp/M20_INTERFACE.md`。
使用底层脚本前，在当前终端设置 `export FIXED_GRASP_WS="$PWD"`，再加载 `source devel/setup.bash`。

## 配置

- 速度、加速度、夹爪力：`src/usr/fixed_grasp/config/fixed.yaml`，修改后重启服务。
- 机械臂 IP：`src/usr/fixed_grasp/config/runtime_defaults.yaml` 的 `robot_ip`。
- 两套点位：`src/usr/fixed_grasp/config/waypoints.yaml`、`waypoints_2.yaml`。
- 当前关节修正：`src/usr/fixed_grasp/config/runtime_kinematics.yaml`。
- 夹爪安装参数：`src/rml63_tools_description/config/measured_mount.yaml`。

关节修正、安装参数与点位、模型一起校验，保留了原文件字节和哈希。
这些是当前抓取所需的既有参数，不需要启动标定或相机；其中历史来源路径仅是记录，不会读取。
换目录后仍可用。模型使用 `src/rml63_tools_description/urdf/fixed_runtime.urdf`，
它冻结了已示教的精确 XML；文件头中的旧路径只是 xacro 注释，不是运行依赖。
保留的 xacro 用于离线对照，不会在启动时重新生成模型。
更换机械臂、夹爪装配或实际工作位置后，需要核对模型并重新示教。

## 离线验证

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
/usr/bin/python3 verify_package.py
/usr/bin/python3 -m unittest discover -s src/usr/fixed_grasp/tests -p 'test_*.py'
```

验证脚本不启动 ROS 节点、不连接机械臂；单元测试使用模拟对象。
打包时不复制 `~/.ros` 中的 M20 事务数据库，目标电脑会使用自己的状态。
同机运行仍使用现有 `~/.ros/fixed_grasp` 数据库。
