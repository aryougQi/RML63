# 机械臂侧对接说明（供 M20 机械狗端使用）

日期：2026-11-03  
适用程序：`fixed_grasp` 正式 M20 联动模式（`--m20`）  
通信协议：ROS1 Noetic，JSON 协议 `schema_version=1`

本文说明机械臂侧当前代码已提供的行为与接口。协议桥接已经实现，跨主机通信、实物停靠位置、收臂空间和整套动作时长仍需双方现场联调确认。

## 1. 机械臂这边负责什么

机械臂收到一次有效的 `place` 请求后，执行一次完整搬运：

```text
检查前提与规划
  → Home
  → 抓取接近点 Pregrasp
  → 抓取点 Grasp、闭爪
  → 抬起 Lift、检查夹持
  → 放置接近点 Preplace
  → 放置点 Place、检查放置条件、松爪
  → 撤离 Retreat
  → 回 Home（ReturnHome）
  → 返回任务结果，并持续报告实际收臂状态
```

抓取点、放置点和 Home 使用机械臂侧预先示教的固定位置。狗端不需要发送机械臂关节角、目标位姿或轨迹；狗趴下后的停靠位置需要与已示教 Place 的承托位置匹配。

机械臂只控制自身和夹爪。导航、停稳、停止导航/速度输出、趴下、起立及恢复行走由狗端负责。

### 当前任务约定

| 项目 | 当前行为 |
| --- | --- |
| 接收的执行类型 | 仅 `operation="execute"`、`source="task"`、`task_type="place"` |
| 一次请求的动作 | 搬一块，完成抓取、放置和回 Home |
| 第一笔新事务 | 搬第一块 |
| 第一块成功后的下一笔新事务 | 搬第二块 |
| 是否需要指定方块 | 不需要，机械臂侧持久化记录决定编号 |
| `payload` | 传 `{}` 即可，不需要 `payload.grasp` |
| `grasp` 类型 | 当前拒绝，即使原始 v1 协议示例使用了该类型 |
| 两块完成后 | 拒绝新作业，等待机械臂侧补料并显式重置顺序 |

失败、取消、结果未知均不自动推进编号。两块完成并补料重置后，下一轮必须使用新的事务 ID；旧事务去重记录仍会保留。

## 2. 通信通道

双方使用同一个 ROS Master。下列消息类型全部是 **`std_msgs/String`**，`data` 内容为 UTF-8 JSON 对象。

| 话题 | 发布方 | 订阅方 | 用途与频率 |
| --- | --- | --- | --- |
| `/m20_cooperation/arm/request` | 狗端 | 机械臂端 | 执行、取消、查询；按需发送，不 latch |
| `/m20_cooperation/arm/feedback` | 机械臂端 | 狗端 | 受理、阶段进度、终态；事件触发，不 latch |
| `/m20_cooperation/arm/status` | 机械臂端 | 狗端 | 就绪、事务、实际收臂状态；10 Hz，空闲也发布 |
| `/m20_cooperation/robot/status` | 狗端 | 机械臂端 | 底盘健康、停稳、姿态；按双方协议 20 Hz |
| `/m20_cooperation/task/status` | 狗端 | 机械臂端 | 本次任务执行授权；按双方协议 20 Hz |

正式联动不接收 `/m20_cooperation_test/*`。该前缀仅用于机械臂侧独立测试，与正式事务记录分开。

## 3. 狗端发送执行请求前的条件

机械臂仅在以下条件同时满足时受理新作业：

1. 请求格式合法，处于 `issued_at` 到 `expires_at` 的受理时间范围内，没有事务冲突。
2. 机械臂本机健康、空闲、实际位于 Home、夹爪空载，没有待恢复锁，且仍有下一块可搬。
3. 已收到狗端新鲜的 `robot/status`，满足 `healthy=true`、`stationary=true`、`posture="crouched"`。
4. 请求引用的底盘状态样本已被机械臂观察到，且从该样本至当前状态之间没有安全条件失效。
5. 已收到新鲜的 `task/status`，其 `state="ARM_EXECUTING"`，`run_id`、`transaction_id` 与请求一致。

因此，狗端应先确认实际趴稳，并开始发布底盘状态和本次任务授权，再发送 execute。机械臂只对底盘引用样本跨话题晚到提供最多 **0.3 秒**等待，该等待不会延长请求有效期，也不能代替提前发布任务授权。

### 状态公共字段

狗端两路状态均需携带：

| 字段 | 要求 |
| --- | --- |
| `schema_version` | 整数 `1` |
| `boot_id` | 非空字符串；发布会话重启后更换 |
| `seq` | 非负整数；同一状态流、同一 boot 下严格递增 |
| `sample_stamp` | 当前 ROS 秒时间，数值类型；同一状态流严格递增 |

机械臂拒绝使用超过 **0.5 秒**、未来时间、重复/倒退序号或缺字段的状态维持授权。双方需同步时钟。

下面所有 JSON 时间均为格式示例，实际必须填写当前 ROS 时间。

`robot/status` 示例：

```json
{
  "schema_version": 1,
  "boot_id": "dog-session-001",
  "seq": 321,
  "sample_stamp": 1793620800.0,
  "healthy": true,
  "stationary": true,
  "posture": "crouched",
  "basic_age_s": 0.2,
  "motion_age_s": 0.03
}
```

`healthy`、`stationary`、`posture` 应由狗端真实反馈确认，不能使用趴下指令的 ACK 代替。当提供原始反馈年龄字段时，机械臂还会检查 `basic_age_s ≤ 1.5`、`motion_age_s ≤ 0.5`，且均不能为负。

`task/status` 示例：

```json
{
  "schema_version": 1,
  "boot_id": "task-session-001",
  "seq": 100,
  "sample_stamp": 1793620800.0,
  "state": "ARM_EXECUTING",
  "run_id": "run-001",
  "transaction_id": "tx-001"
}
```

任务状态也支持用 `phase` 表示阶段；同时发送 `state` 和 `phase` 时，两者必须一致。执行全程需持续保持对应事务的 `ARM_EXECUTING` 授权与趴稳状态。

## 4. 执行请求格式

向 `/m20_cooperation/arm/request` 发布：

```json
{
  "schema_version": 1,
  "request_id": "req-001",
  "source": "task",
  "run_id": "run-001",
  "mission_id": "mission-001",
  "waypoint_id": "point-01",
  "transaction_id": "tx-001",
  "operation": "execute",
  "task_type": "place",
  "issued_at": 1793620800.0,
  "expires_at": 1793620810.0,
  "robot_state_ref": {
    "boot_id": "dog-session-001",
    "seq": 321
  },
  "payload": {}
}
```

| 字段 | 说明 |
| --- | --- |
| `request_id` | 一次请求的唯一 ID |
| `transaction_id` | 一次物理作业的唯一 ID；作业重发时保持不变 |
| `run_id / mission_id / waypoint_id` | 狗端运行、任务、任务点标识；反馈会回显 |
| `issued_at / expires_at` | 数值类型 ROS 秒时间；限制首次受理，不限制执行完成时间 |
| `robot_state_ref` | 已发布的真实底盘状态的 `boot_id` 与 `seq`，不能随意填写 |
| `payload` | JSON 对象；目前用 `{}`，内容参与事务一致性校验 |

狗端创建第二次物理作业时，应使用新的 `request_id` 和 `transaction_id`，同步更新任务授权及底盘引用。不要通过修改 `payload` 选择第二块。

## 5. 机械臂返回什么

### 5.1 阶段与终态：arm/feedback

受理后发布 `accepted`，执行阶段变化时发布 `running`，最后发布 `succeeded`、`failed` 或 `cancelled`。`running` 是阶段事件，不保证固定频率；在线状态请看持续发布的 `arm/status`。

成功反馈示例：

```json
{
  "schema_version": 1,
  "request_id": "req-001",
  "run_id": "run-001",
  "mission_id": "mission-001",
  "waypoint_id": "point-01",
  "transaction_id": "tx-001",
  "operation": "execute",
  "task_type": "place",
  "status": "succeeded",
  "task_success": true,
  "arm_safe_to_move": false,
  "error_code": "OK",
  "message": "placed, retreated and returned Home",
  "grasp": 1,
  "transaction_status": "succeeded",
  "stage": "DONE"
}
```

当前程序仅在完整流程完成、规划与执行成功、夹持已验证且阶段为 `DONE` 后，才返回 execute 成功。该结果包括放置、撤离和返回 Home。

当前 execute 反馈中的 `arm_safe_to_move` 保持 `false`；狗端必须另外等待实时 `arm/status` 的实际收臂确认。`grasp` 为此次搬运编号，`stage`、`message` 用于进度展示和排查，不能单独作为完成判据。

### 5.2 实时状态：arm/status

第一块完成并确认实际收臂后的示例：

```json
{
  "schema_version": 1,
  "boot_id": "arm-boot-001",
  "seq": 500,
  "sample_stamp": 1793620825.0,
  "healthy": true,
  "ready": true,
  "transaction_id": "tx-001",
  "active_request_id": null,
  "arm_safe_to_move": true,
  "next_grasp": 2,
  "manual_test": false
}
```

| 字段 | 机械臂侧含义 |
| --- | --- |
| `boot_id / seq / sample_stamp` | 本次机械臂进程标识、递增序号、当前 ROS 时间 |
| `healthy` | 执行权限及实际关节、故障、使能、夹爪反馈满足健康检查 |
| `ready` | 当前可以受理新作业；包含空载 Home、无恢复锁及顺序未完成等条件 |
| `transaction_id` | 当前或最后一次事务 ID；完成后持续保留，用于狗端匹配 |
| `active_request_id` | 活动执行请求 ID；无活动事务时为 `null` |
| `arm_safe_to_move` | 当前实际收臂检查通过，允许底盘移动 |
| `next_grasp` | 下一块编号 `1` 或 `2`；两块完成后为 `null` |

收臂判据使用实际 Home 关节到位、静止、夹爪空载和硬件健康反馈，不只依赖程序已执行到某一行。Home 的实际外形是否满足狗起立和行走空间，仍需现场确认。

两块完成后 `ready=false`、`next_grasp=null`；若实际收臂检查通过，`arm_safe_to_move` 仍可为 `true`。狗端不要用 `ready` 代替收臂判据。

## 6. 狗端如何确认可以起立

狗端应依次确认以下两步：

1. 收到与本次 execute 的身份字段匹配的反馈：`operation="execute"`、`status="succeeded"`、`task_success=true`。
2. 收到新鲜的 `arm/status`：`transaction_id` 与本次作业一致、`active_request_id=null`、`arm_safe_to_move=true`。

状态还应符合协议的新鲜度和会话要求：不超过 0.5 秒、无未来时间、序号有效；机械臂重启更换 `boot_id` 时进入异常核实流程。

只收到 `accepted`、只收到成功反馈、收到取消 ACK、看到阶段为 Home 或看到 `active_request_id=null`，都不足以单独授权起立。

建议狗端流程：

```text
导航到点 → 停稳并停止导航/速度输出 → 实际趴下
  → 持续发布 robot/status 与本次 task/status
  → 确认机械臂 ready、healthy → 发送一次 place execute
  → 等待匹配的成功反馈
  → 等待同事务新鲜的安全收臂状态
  → 起立、恢复导航并前往下一点
```

## 7. 重发、取消、查询和异常

### 重发与持久化

- 机械臂在下发物理动作前持久化受理事务，使用 SQLite 保存结果与方块顺序。
- 同一 `transaction_id`、同一任务参数重发，只返回原状态或结果，不再次执行、不重复计数。
- 同一事务的 `run_id`、`mission_id`、`waypoint_id`、`task_type` 或 `payload` 改变，会拒绝并返回 `TRANSACTION_CONFLICT`。
- 同一 `request_id` 的完整内容必须一致；若更新时间或引用等内容，需使用新 `request_id`，同一作业仍沿用原事务 ID。
- 失败/取消的旧事务重发仍返回原结果。确需重新执行时，应先人工确认物理状态并恢复，再创建新事务，不自动重抓。

### 取消与查询

两者均向同一 `arm/request` 话题发送。取消示例：

```json
{
  "schema_version": 1,
  "request_id": "cancel-001",
  "run_id": "run-001",
  "transaction_id": "tx-001",
  "target_request_id": "req-001",
  "operation": "cancel",
  "issued_at": 1793620805.0,
  "expires_at": 1793620815.0
}
```

查询使用相同关联字段，改为 `operation="query"`，并使用新的请求 ID 和有效时间。查询结果仍通过 `arm/feedback` 返回，包含 `transaction_status`、`task_success`、阶段及安全状态。

取消回执的 `status="succeeded"` 只代表取消请求已处理，`task_success=false`，不代表原作业成功，也不代表已经停止或收臂。查询回执的 `succeeded` 只表示查询成功；原事务是否完成看其结果字段。狗端应按 `operation` 区分回复，不能将它们当成 execute 成功。

### 执行中失联、姿态变化或授权丢失

机械臂会请求取消并停止后续动作，保持已有夹持，不自动松爪、继续搬下一块或自动回 Home。故障发生前可能已有物理动作，不能把失败解释为物体完全未移动。

进程在受理后、终态落盘前中断时，事务恢复为结果未知。重发 execute 会返回 `status="failed"`、`error_code="RESULT_UNKNOWN"`，并携带 `result_state="unknown"`，不会自动重放。双方人工核实后，由机械臂侧核对持久化记录，狗端再按原协议处理 `task/resolve_arm_result`；该接口不由本机械臂程序提供。

常见错误码：

| 错误码 | 含义 |
| --- | --- |
| `UNSUPPORTED_TASK` | 不是 `source=task`、`task_type=place` |
| `ARM_NOT_READY` | 本机未满足空载 Home、健康或恢复条件 |
| `BUSY` | 已有活动事务或正在等待其他请求引用 |
| `REQUEST_EXPIRED` | 新请求已过期或签发时间在未来 |
| `REFERENCE_TIMEOUT / REFERENCE_INVALIDATED` | 引用样本未及时观察到或安全连续性失效 |
| `TASK_AUTHORIZATION_LOST / ROBOT_HEARTBEAT_LOST` | 任务授权或底盘状态失效 |
| `TRANSACTION_CONFLICT / REQUEST_ID_CONFLICT` | ID 复用但内容不一致 |
| `SEQUENCE_COMPLETE` | 两块已完成，等待补料重置 |
| `RESULT_UNKNOWN` | 中断后物理结果不确定，需人工核实 |

## 8. 联调连接与时间约定

双方先确认实际 IP 和 ROS Master 所在机器。以下假设 Master 在机械臂电脑 `192.168.1.10`，该地址仅为示例。

机械臂端由机械臂操作人员执行，先回 Home、确认空载并停止旧服务：

```bash
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.10
bash $FIXED_GRASP_WS/src/usr/start_fixed_grasp.sh --confirm-empty --m20
```

狗端加载自身 ROS 环境后设置：

```bash
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=<机械臂电脑能够反向访问的狗端IP>
```

上面的 IP 占位符需替换后再执行。双方需保证 ROS 节点双向可达，不能只检查 Master 的 11311 端口；同时同步时钟。正式联动不需要狗端调用机械臂手动 `client.py`。

可用以下只读命令检查连接与状态：

```bash
rostopic info /m20_cooperation/arm/request
rostopic hz /m20_cooperation/arm/status
rostopic echo /m20_cooperation/arm/status
rostopic echo /m20_cooperation/arm/feedback
```

原始狗端协议默认机械臂执行超时 30 秒，收臂等待另计 30 秒。当前机械臂 execute 的成功结果包含规划、搬运、放置和返回 Home，双方应实测完整耗时，并在狗端设置足够的 `arm_timeout`。请求的 `expires_at` 不替代执行超时。

首次联调需确认：实际停靠后 Place 对齐、Home 不影响狗起立行走、正常两次搬运、重复事务不重复动作，以及取消、断线、重启后的结果核实流程。

## 9. 对应实现文件

- `scripts/m20_bridge.py`：ROS 话题收发、接入机械臂任务服务、实际硬件状态观察。
- `scripts/m20_protocol.py`：请求校验、授权与心跳监控、反馈和状态生成。
- `scripts/m20_ledger.py`：事务持久化、去重、第一块到第二块的顺序。
- `scripts/core.py`：完整抓取、放置与回 Home 的动作流程。
- `config/fixed.yaml`：机械臂侧规划、速度、夹爪和放置条件配置；狗端无需修改。
