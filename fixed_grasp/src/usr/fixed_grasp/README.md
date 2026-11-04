```bash
# 终端一：空载启动（ROS、MoveIt、夹爪、抓取服务）
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty
```

```bash
# 不连接机械狗，手动place测试真机械臂；先回Home、停旧服务，准备好承托面
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20-test
```

```bash
# 每执行一次实际搬一块；完成后再运行同一条搬第二块
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_manual_test.py place --execute
```

```bash
# 机械狗联动：先回Home并停止旧服务；两次新的place事务依次搬1、2
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20
```

```bash
rostopic echo /m20_cooperation/arm/status
```

```bash
rostopic echo /m20_cooperation/arm/feedback
```

```bash
# 终端二：加载环境
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
cd $FIXED_GRASP_WS
```

```bash
# Home 预览
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home
```

```bash
# Home 执行
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home --execute
```

```bash
# 第一方块：预览
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 1 --full-transfer
```

```bash
# 第一方块：执行
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 1 --execute --full-transfer
```

```bash
# 第二方块：预览
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 2 --full-transfer
```

```bash
# 第二方块：执行
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 2 --execute --full-transfer
```

```bash
# 人工确认空载且机械臂停止后：恢复 Home
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home --confirm-empty
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home --confirm-empty --execute
```

```bash
# 停止旧抓取服务、手动到位后：示教第一方块
/usr/bin/python3 src/usr/fixed_grasp/scripts/teach.py Grasp --grasp 1 --interactive
```

```bash
# 实测确认 Place 能稳定支撑并松爪后：更新放置确认
/usr/bin/python3 src/usr/fixed_grasp/scripts/confirm_place.py \
  --confirm-supported \
  --source '操作者实测确认：当前物体在已示教 Place 能稳定支撑并松爪'
```

```bash
# 手动到位并停止后：示教第二方块
/usr/bin/python3 src/usr/fixed_grasp/scripts/teach.py Grasp --grasp 2 --replace --interactive
```

```bash
# 重新启动抓取服务
roslaunch fixed_grasp fixed_grasp.launch allow_execution:=true
```

```bash
# 离线IK性能对比，不连接机械臂
/usr/bin/python3 src/usr/fixed_grasp/scripts/benchmark_ik.py
```

```bash
# 独立ROS完整规划对比，不启动驱动、不执行运动
/usr/bin/python3 src/usr/fixed_grasp/scripts/benchmark_planning.py
```
