```bash
# 终端一：空载一键启动；更新后先关闭旧服务，会开启24V并开爪到50mm
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty
```

```bash
# 不连机械狗的手动真机测试：先回Home并停止旧服务，准备好Place承托面和两块
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20-test
```

```bash
# 另开终端：每执行一次实际搬一块，成功后再运行同一条搬第二块
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
cd $FIXED_GRASP_WS
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_manual_test.py place --execute
```

```bash
# 再测一轮：先停止旧服务并补回两块，重置独立测试计数
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_ledger.py \
  --database ~/.ros/fixed_grasp/m20_test_transactions.sqlite3 --reset-sequence --confirm-reloaded
```

```bash
# 机械狗联动：先在手动模式回Home并停止旧服务，与上方启动二选一；IP按实际互通网口修改
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.10
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20
```

```bash
# 机械狗两次新place事务依次放1、2；看反馈与实际收臂状态
rostopic echo /m20_cooperation/arm/feedback
```

```bash
rostopic echo /m20_cooperation/arm/status
```

```bash
# 两块完成后：停止旧服务、补回两块，再重置顺序；保留旧事务去重
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_ledger.py --reset-sequence --confirm-reloaded
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20
```

```bash
# 手动启动方案：只启动驱动和MoveIt，与一键启动二选一
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
cd $FIXED_GRASP_WS
roslaunch rml63_tools_description realrobot.launch allow_execution:=true
```

```bash
# 关闭抓取服务、手动到位并停止后：示教第一方块，已有记录输入r重录
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
cd $FIXED_GRASP_WS
/usr/bin/python3 src/usr/fixed_grasp/scripts/teach.py Grasp --grasp 1 --interactive
```

```bash
# 实测确认Place能稳定支撑并松爪后：更新放置确认
/usr/bin/python3 src/usr/fixed_grasp/scripts/confirm_place.py \
  --confirm-supported \
  --source '操作者实测确认：当前物体在已示教 Place 能稳定支撑并松爪'
```

```bash
# 手动到位并停止后：示教第二方块；第一套更新后需重新示教第二套
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
cd $FIXED_GRASP_WS
/usr/bin/python3 src/usr/fixed_grasp/scripts/teach.py Grasp --grasp 2 --replace --interactive
```

```bash
# 手动启动方案：启动抓取服务；一键启动后无需重复执行
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
cd $FIXED_GRASP_WS
roslaunch fixed_grasp fixed_grasp.launch allow_execution:=true
```

```bash
# 终端二：加载环境并预览第一方块，不运动
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
cd $FIXED_GRASP_WS
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 1 --full-transfer
```

```bash
# 第一方块：预览通过后执行，完成搬运并回Home
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 1 --execute --full-transfer
```

```bash
# 第二方块：预览，不运动
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 2 --full-transfer
```

```bash
# 第二方块：预览通过且上一任务结束后执行
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --grasp 2 --execute --full-transfer
```

```bash
# 单独Home：预览，不运动
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home
```

```bash
# 单独Home：预览通过后执行
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home --execute
```

```bash
# 人工确认空载、机械臂停止后：恢复Home，执行成功后清除恢复锁
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home --confirm-empty
/usr/bin/python3 src/usr/fixed_grasp/scripts/client.py --home --confirm-empty --execute
```

```bash
# 查看第一方块结果
rostopic echo /fixed_grasp/task/result
```

```bash
# 查看第二方块结果
rostopic echo /fixed_grasp/task_2/result
```

```bash
# 离线IK性能对比，不连接机械臂
/usr/bin/python3 src/usr/fixed_grasp/scripts/benchmark_ik.py
```

```bash
# 独立ROS完整规划对比，不启动驱动、不执行运动
/usr/bin/python3 src/usr/fixed_grasp/scripts/benchmark_planning.py
```
