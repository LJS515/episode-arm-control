# 06 TCP / JSON 协议

> 状态：协议实现与测试完成（Mock）。与官方服务器的兼容性：**待真机/真实上位机验证**。

## 帧格式（B：当前官方；C：历史版本同一帧头）

```
| 8 bytes unsigned big-endian N | N bytes UTF-8 JSON |
```

请求与响应格式相同。历史版本 payload 为 `pickle`（C），当前为 JSON（B）。

## 消息

### Legacy 模式（默认，`legacy_compatible=True`）

请求只包含官方字段：

```json
{"action": "angle_mode", "params": [[180.0, 90.0, 83.0, 30.0, 110.0, 30.0], 0.3]}
```

响应是“原始结果值”：

| action | params | 结果 | 证据 |
|---|---|---|---|
| `get_motor_angles` | `null` | `[q1..q6]`，元素可为 `null` | B/C |
| `angle_mode` | `[[q1..q6], speed_ratio]` | 预计耗时（s） | B/C |
| `move_xyz_rotation` | `[x,y,z,r1,r2,r3,"zyx"]` | 预计耗时或 `-1`（无解） | B/C |
| `move_linear_xyz_rotation` | 同上 | 总耗时或 `-1` | B/C |
| `home` | `null` | 历史版本硬编码 `5` | C |
| `gripper_on/off` | `null` | `0.05` | C |
| `servo_gripper` | 角度 | `1` | C |
| `set_free_mode` | `0/1` | 未知，可能不回复 | C/D |
| `robodk_simu` | `0/1` | `0.05` | C |
| `emergency_stop` | 未知（SDK 发 `null`） | 未知 | B（存在）/E（格式） |
| `dynamic_move` | 未知（SDK 发 `[[q]]` 或 `[[q], r]`） | 未知 | B/E |
| `get_T` | `null` | 4×4（格式 E） | B/E |
| `get_pose` | `null` | `[x,y,z,rx,ry,rz]`（格式 E） | B/E |

### Enhanced 模式（仅本项目 Mock / 未来自有服务器）

```json
{"protocol_version":"1.0","request_id":"req-123-...","action":"get_motor_angles","params":null}
{"protocol_version":"1.0","request_id":"req-123-...","ok":true,"result":[...]}
{"protocol_version":"1.0","request_id":"req-124-...","ok":false,"error":{"code":"NO_IK_SOLUTION","message":"..."}}
```

客户端校验主版本号与 `request_id`，不匹配即 `RequestIdMismatchError`（防止错位回复被当成本请求的结果）。**不假设官方服务器支持这些字段。**

## 加固点

| 问题 | 措施 | 测试 |
|---|---|---|
| 短读（`recv` 返回少于请求字节） | `recv_exact` 循环 | 头部逐字节、payload 任意分片 |
| 对端中途关闭 | 头部未完整 → `TruncatedFrameError`；未开始 → `ConnectionClosedError` | 3 个用例 |
| 超大长度头（内存耗尽） | 读 body 前比较 `max_message_size`（默认 1 MiB） | `1<<40` 长度头 |
| 非法 JSON / 非 UTF-8 / 空 payload | `MalformedPayloadError` | 3 个用例 |
| NaN / Infinity | 编码 `allow_nan=False`，解码拒绝常量 | 6 个用例 |
| 慢速滴灌 | 截止时间是**绝对**的，不会每个分片重置 | `test_deadline_spans_multiple_chunks` |
| 无回复 | 接收超时 + 命令总超时 | `test_unknown_action_legacy_times_out` |

## 超时体系

| 超时 | 默认 | 含义 |
|---|---|---|
| `connect_s` | 2.0 | 建连 |
| `send_s` | 2.0 | 发送整帧 |
| `receive_s` | 5.0 | 接收整帧 |
| `command_total_s` | 10.0 | 一次请求的总上限（含以上全部） |

> 轮询路径必须用更短的读超时（ROS 节点默认 0.3 s），且 **读超时 < 状态过期阈值**，否则一次挂起的读会掩盖通信中断（`examples/mock_demo.py` 中已体现）。

## 重试策略

只重试幂等请求：`get_motor_angles`、`get_T`、`get_pose`、`emergency_stop`。**运动命令绝不自动重试**：超时后无法知道机械臂是否已收到，重发可能执行两次（`test_motion_is_never_retried`）。

## 连接策略

* `per_command`（默认）：每次请求新连接，与历史官方客户端一致，兼容性最好。Mock 本机测得 p50≈1.0 ms / p95≈15.9 ms（**Synthetic / Mock Data**，Windows）。
* `persistent`：长连接，Mock 本机 p50≈0.15 ms（**Synthetic / Mock Data**）。出错后丢弃连接重建，避免流错位。
* 急停：**始终新开专用连接**，不排队在其他请求之后（`test_estop_not_blocked_by_hanging_command`）。

## 待真机验证

1. 当前官方服务器对 `{"action","params"}` 的接受情况与回复格式（用 `tools/tcp_trace.py` 抓官方示例流量对照）。
2. `emergency_stop`、`dynamic_move`、`get_T`、`get_pose` 的参数格式。
3. 服务器是否支持同连接多请求；是否仍按连接分线程。
