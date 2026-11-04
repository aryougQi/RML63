# M20 机械狗对接

只启用 `task_type="place"`。**第一次新的事务搬运第一块，成功后，第二次新的事务搬运第二块**。每次都执行原来的完整抓取、放置、回 Home；不会收到一次请求就连续搬两块。

同一 `transaction_id` 重发只返回原状态/结果，不动作、不跳号。失败/取消不推进顺序。两块完成后停止受理新作业，补料并显式重置后才能开始下一轮。重启保留顺序和事务历史。

## 启动

不连接机械狗、手动输入 `place` 测试真机械臂时，使用独立测试模式。先准备好两块与 Place 承托面，回 Home 并停止旧服务：

```bash
# 终端一：测试模式，夹爪需空载；与正式 --m20 二选一
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20-test
```

```bash
# 终端二：这条会实际搬运一块；完成后再执行同一条搬第二块
source /opt/ros/noetic/setup.bash
source $FIXED_GRASP_WS/devel/setup.bash
cd $FIXED_GRASP_WS
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_manual_test.py place --execute
```

客户端只向 `/m20_cooperation_test/*` 发送模拟趴稳/授权心跳及一次请求，实际机械臂反馈仍需通过检查；Ctrl+C 会取消自己的任务。测试计数独立保存在 `~/.ros/fixed_grasp/m20_test_transactions.sqlite3`，不会推进正式机械狗计数。正式 `--m20` 不接收这些测试话题。进入正式联调前需补回测试搬走的方块。

```bash
# 测试完成需再来一轮：停止旧服务、补回两块，再重置测试计数
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_ledger.py \
  --database ~/.ros/fixed_grasp/m20_test_transactions.sqlite3 --reset-sequence --confirm-reloaded
```

测试中若失败，先人工恢复空载，再用 `client.py --home --confirm-empty --execute` 回 Home。此手动恢复仅在测试模式开放；正式联动仍需退出联动模式后处理。

先用手动模式使机械臂回到已示教 Home，停止旧启动终端。Home 是本接口的收臂位置，首次联调应实测它不妨碍机械狗起立和行走。Place 沿用当前示教位置，狗趴下停靠后需与承托位置匹配。

```bash
# 机械臂电脑：192.168.1.10 替换为两端互通的实际电脑IP
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.10
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20
```

```bash
# 已独立启动驱动和MoveIt时：只替换原抓取服务，与一键启动二选一
roslaunch fixed_grasp fixed_grasp.launch allow_execution:=true m20_enabled:=true
```

机械狗 ROS 主机使用同一个 `ROS_MASTER_URI`，其 `ROS_IP` 设置为机械臂电脑可反向访问的机械狗 ROS 主机 IP。两端需时间同步、ROS TCP 双向可达。联动模式仅接受 M20 授权的执行请求，手动客户端仍可预览；手动执行/恢复需先退出联动模式。

## 机械狗发送什么

沿用原接口 v1 的完整请求字段，固定：

| 字段 | 设置 |
| --- | --- |
| `operation` | `execute` |
| `task_type` | `place` |
| `source` | `task` |
| `payload` | `{}` 即可；其他内容只用于事务一致性校验 |
| `request_id` | 每次新请求使用新 ID |
| `transaction_id` | 每次新的物理作业使用新 ID；重发同一作业保持不变 |
| `robot_state_ref` | M20 已发布的当前 `boot_id`、`seq` |
| `issued_at/expires_at` | 当前 ROS 秒时间及首次受理期限 |

无需传 `payload.grasp`。第一次和第二次由机械臂持久化计数决定。`grasp` 类型会被拒绝。

执行前，M20 必须持续发布真实的 `healthy=true, stationary=true, posture="crouched"`；任务状态使用 `state="ARM_EXECUTING"`（也支持 `phase`），并匹配本次 `run_id/transaction_id`。状态需有 `schema_version=1, boot_id, seq, sample_stamp`；过期、未来时间或重复/倒退序号不作为有效心跳。暴露 `basic_age_s/motion_age_s` 时还会检查它们；原始反馈与停稳判据仍由 M20 的聚合器确认。

引用样本必须被机械臂观察到，允许最多 0.3 秒跨话题等待，不会延长请求有效期。执行中姿态变化、授权/心跳丢失会取消并停止机械臂后续动作，保留夹持，不自动松爪或继续下一块。

## 查看结果

```bash
rostopic echo /m20_cooperation/arm/feedback
```

```bash
rostopic echo /m20_cooperation/arm/status
```

`arm/status` 即使空闲也按 10 Hz 发布，含新 `boot_id`、递增 `seq` 和当前 `sample_stamp`。`next_grasp` 为下一块编号，两块完成后为 `null`。

机械狗先确认匹配请求的 `status="succeeded", task_success=true`，再等待同一事务的新鲜状态满足 `active_request_id=null, arm_safe_to_move=true`。收臂状态根据真实 Home 关节到位、静止、夹爪空载及硬件健康确认；成功反馈本身不授权起立。最后事务 ID 会持续保留。

支持原协议的 `cancel/query`，需新请求 ID、原 `run_id/transaction_id/target_request_id` 及有效时间。取消回执的 `succeeded` 只代表取消请求已处理，`task_success=false`，不等于放置成功或已收臂。query 会返回持久化结果及当前安全状态。M20 默认执行超时为 30 秒；若实测整套搬运更长，应在 M20 侧增大 `arm_timeout`。

## 补料、重置和未知结果

数据库默认 `~/.ros/fixed_grasp/m20_transactions.sqlite3`，不要删除它来复位。重置仅复位 1→2 顺序，保留旧事务去重。

```bash
# 先停止旧启动终端，将两块补回原示教位置
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_ledger.py --reset-sequence --confirm-reloaded
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20
```

进程中断时仍在执行的事务恢复为 `RESULT_UNKNOWN`，旧事务绝不自动重放。若核实某个未知/失败事务实际上已完整放置成功，先停止服务并同步机械臂记录，再重新启动，最后在 M20 侧人工核实并处理原协议的 `task/resolve_arm_result`：

```bash
# TX_ID 和确认说明替换为实际值；不发送任何机械臂动作
/usr/bin/python3 src/usr/fixed_grasp/scripts/m20_ledger.py \
  --resolve-completed TX_ID --confirm-completed --source '操作者实测确认该事务已经完成放置并收臂'
```

本接口只发布机械臂反馈/状态，不控制机械狗的速度、姿态或 TCP/UDP。接口原文为用户提供的 `ARM_ROS_INTERFACE.md`。
