# 11 错误模型

> 状态：实现（`episode_sdk/exceptions.py`）+ 测试。

## 设计目标

1. 每个错误有稳定的机器可读 `code`，上层（ROS 节点、日志分析）按 code 分支，不解析文本。
2. 区分“没发出去”（校验错误）、“发出去了但不知道结果”（传输超时）、“对方明确失败”（远端错误）、“被安全层拒绝”。这四类的处理方式完全不同。
3. `retryable` 只表示“同一请求重发可能成功”；**运动命令无论如何不自动重试**。

## 层级

| 类别 | code 示例 | 机械臂是否可能已动作 | 处理 |
|---|---|---|---|
| `ValidationError` | `JOINT_LIMIT_VIOLATION`, `NON_FINITE_VALUE`, `SPEED_RATIO_INVALID`, `JOINT_COUNT_INVALID`, `PARAMETER_TYPE_INVALID` | 否（未发送） | 修正参数 |
| `ProtocolError` | `FRAME_TOO_LARGE`, `FRAME_TRUNCATED`, `PAYLOAD_MALFORMED`, `REQUEST_ID_MISMATCH`, `PROTOCOL_VERSION_MISMATCH` | **未知** | 丢弃连接；读请求可重试 |
| `TransportError` | `CONNECT_TIMEOUT`, `RECEIVE_TIMEOUT`, `COMMAND_TIMEOUT`, `CONNECTION_CLOSED`, `CONNECTION_FAILED` | 连接前失败：否；之后：**未知** | 读可重试；运动 → 读反馈确认状态 |
| `RemoteError` | `NO_IK_SOLUTION`, `REMOTE_COMMAND_FAILED` | 否（官方报告失败） | 换目标 |
| `SafetyError` | `NOT_HOMED`, `STATE_REJECTED`, `ESTOP_ACTIVE`, `FAULT_LATCHED`, `STATE_STALE` | 否 | 按状态恢复 |
| `ArbitrationError` | `COMMAND_REJECTED`, `COMMAND_PREEMPTED`, `COMMAND_CANCELLED` | 否 / 被停止 | 等待或换策略 |
| `HardwareError` | `HARDWARE_READ_FAILED`, `HARDWARE_COMMAND_FAILED` | 未知 | 计数 → 故障 |

所有异常 `to_dict()` 输出 `{code, type, message, retryable, details}`，可直接进日志或 ROS 结果字符串。

## “不知道结果”是最危险的状态

运动命令超时后，客户端不知道：命令没到？到了没执行？执行了回复丢了？本项目的处理：

1. 不重试运动命令。
2. 在控制器层：命令下发失败 → 锁存 `COMMAND_FAILED` 故障 → 停止 → 需要操作员确认。
3. 恢复时以**实测反馈**为准重新建立认知，而不是以“最后一次发出的目标”为准。

## 故障码（安全监督器锁存）

| code | 触发 | 证据/对应现象 |
|---|---|---|
| `READ_FAILURES` | 连续读失败（含部分 None）≥ 3 | FAQ tid=1932（None） |
| `STALE_DURING_MOTION` | 运动/停止/回零期间反馈过期 | — |
| `POSITION_OUT_OF_RANGE` | 实测角超出物理范围 ± 3° | 回零零点漂移（tid=1996） |
| `MOTION_TIMEOUT` | Monitor 超时 / 仲裁器票据超时 | 堵转、过慢 |
| `FINAL_ERROR_EXCEEDED` | 停止但终点误差 > 容差 | 皮带打滑（tid=1996） |
| `TRACKING_ERROR` | 可选：跟踪误差上限 | 默认关闭（期望轨迹未知） |
| `HOMING_FAILED` | 回零超时 | tid=1260（60 s） |
| `COMMAND_FAILED` | 硬件拒绝/传输失败 | — |
