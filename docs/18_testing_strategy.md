# 18 测试策略

> 状态：自动化测试已在 Windows 11 / Python 3.12.8 与 WSL2 Ubuntu 24.04 / Python 3.12.3 运行；ROS2 Jazzy 运行测试见 `reports/ros2_runtime_wsl.md`。当前测试数量以 `python -m pytest -q` 的输出为准。

## 金字塔

| 层 | 工具 | 依赖 | 文件 |
|---|---|---|---|
| 单元 | pytest + FakeClock | 无网络 | validation, joint_mapping, state_cache, state_machine, command_arbiter, safety_supervisor, trajectory_monitor |
| 协议 | `socket.socketpair` | 本机 | test_protocol |
| SDK ↔ Mock | 真实 TCP（127.0.0.1，随机端口） | 本机 | test_client |
| 集成 | FakeClock + SimulatedArm；以及真实线程 + TCP Mock | 本机 | test_integration_mock |
| ROS 逻辑 | stub rclpy 模块 | 无 ROS | test_ros_node_stubbed |
| 工具/配置 | 合成输入 | 无 | test_tools |
| ROS2 运行 | colcon + launch + rclpy 客户端 | ROS2 Jazzy（WSL2） | `episode_ros2/test/ros2_runtime_smoke.py`：17/17 |
| C++ 单元 | colcon test + gmock | ROS2 Jazzy（WSL2） | `ros2_control/test/test_episode_system_hardware.cpp`：5/5 |
| ros2_control 运行 | controller_manager + JTC + JSB | ROS2 Jazzy（WSL2） | mock 与 vendor_tcp→TCP Mock 两种后端，轨迹 SUCCEEDED |
| MoveIt | move_group + MoveGroup action | MoveIt 2.12.4 + 官方 URDF（WSL2） | `moveit_config/test/moveit_smoke.py --collision-check`：两条执行栈各 11/11 |
| 硬件验收 | 真机 | **机械臂（缺）** | 未运行 |

## 关键技术

* **注入时钟（FakeClock）**：所有时间逻辑（过期、超时、稳定时间）都用注入的 `clock`，测试中手动推进，结果确定、不依赖机器速度。
* **Mock Server 复刻历史行为**：一连接一线程、回复=预计耗时、未知命令不回复、`set_free_mode` 不回复。
* **故障注入**：传输层（延迟、丢回复、截断、分片、垃圾、超大头、断开、None）+ 机械层（卡死关节、终点偏移、冻结反馈、读失败、命令拒绝、IK 失败、减速）。

## 需求覆盖矩阵（要求中的 21 项）

| # | 场景 | 测试 |
|---|---|---|
| 1 | 长度头分片 | test_protocol::test_header_fragmented_byte_by_byte；test_client::test_fragmented_reply_is_reassembled |
| 2 | JSON 分片 | test_protocol::test_json_payload_fragmented |
| 3 | 响应截断 | test_protocol::test_truncated_header/payload；test_client::test_truncated_reply |
| 4 | 非法 JSON | test_protocol::test_invalid_json/utf8；test_client::test_garbage_reply |
| 5 | 超大消息 | test_protocol::test_oversize_*；test_client::test_oversize_reply_header |
| 6 | 连接超时 | test_client::test_connect_timeout（monkeypatch），test_connection_refused |
| 7 | 响应超时 | test_client::test_response_timeout, test_command_total_deadline；test_protocol::test_receive_timeout |
| 8 | 服务器关闭 | test_client::test_server_closes_connection；test_protocol::test_peer_closed_before_anything |
| 9 | 错误数组长度 | test_validation::test_wrong_length |
| 10 | NaN/Inf | test_validation::test_non_finite；test_protocol::test_non_finite_* |
| 11 | 越过关节限位 | test_validation::test_joint_limits；test_client::test_validation_blocks_send |
| 12 | 非法速度 | test_validation::test_speed_ratio_invalid；test_safety_supervisor::test_speed_rejected |
| 13 | 未回零运动 | test_integration_mock::test_motion_rejected_before_homing；test_safety_supervisor::test_not_homed_rejected |
| 14 | 并发运动 | test_command_arbiter::test_concurrent_submit_single_owner；test_integration_mock::test_concurrent_motion_rejected |
| 15 | Action 取消 | test_ros_node_stubbed::test_execute_cancel_stops_motion；test_integration_mock::test_cancel_stops_physically |
| 16 | 软件停止 | test_integration_mock::test_soft_stop, test_stop_escalates_to_estop_when_soft_stop_unsupported |
| 17 | 急停 | test_integration_mock::test_emergency_stop_during_motion, test_estop_works_even_if_reads_hang；test_client::test_estop_not_blocked_by_hanging_command |
| 18 | 状态过期 | test_state_cache::test_becomes_stale_with_age；test_safety_supervisor::test_stale_*；test_integration_mock::test_stale_feedback_during_motion_faults |
| 19 | 连续读取失败 | test_state_cache::test_failures_counted_and_reset；test_integration_mock::test_consecutive_read_failures_fault_in_ready |
| 20 | 轨迹超时 | test_trajectory_monitor::test_timeout_*；test_integration_mock::test_stuck_joint_times_out, test_arbiter_timeout_latches_fault |
| 21 | 终点误差超限 | test_trajectory_monitor::test_final_error_exceeded_when_stopped_short；test_integration_mock::test_belt_slip_final_error |

## 运行

```bash
python -m pytest -q                       # 全部
python -m pytest -q tests/test_client.py  # 单文件
python tools/generate_mock_report.py --out reports/mock_fault_injection_report.md
```

## 平台差异（真实踩到的）

Windows 上连接一个无人监听的端口，TCP 栈会重试 SYN 约 2 s 才报“拒绝”，Linux 立即 RST。测试改为接受 `ConnectionFailedError` 或 `ConnectTimeoutError`。

## ROS2 Jazzy 中已执行的测试（WSL2，Mock）

| 测试 | 命令 | 结果 |
|---|---|---|
| ROS2 节点启动 + Action + 取消 + 急停 + 故障 + 诊断 | `ros2 launch episode_ros2 mock.launch.py` + `python3 episode_ros2/test/ros2_runtime_smoke.py` | 17/17 |
| C++ 编译与单元测试 | `colcon build` + `colcon test --packages-select episode_ros2_control` | 0 警告，gmock 5/5 |
| ros2_control 运行 | `ros2 launch episode_ros2_control mock_ros2_control.launch.py [backend:=vendor_tcp]` | JTC SUCCEEDED，50 Hz；断开 TCP Mock → 硬件停用 |

ROS2 环境中发现的两个问题（均已修复）：ros2_control 4.48 弃用旧 API；colcon 入口脚本使用系统 Python。

## 仍未运行的测试

| 测试 | 缺失依赖 | 命令 | 预期检查 |
|---|---|---|---|
| 官方上位机兼容 | 官方 Windows 上位机 | `tools/tcp_trace.py` + `examples/read_angles.py --host ... --real-robot` | JSON 字段与回复格式一致 |
| 硬件验收 | 机械臂 | 见 `12` 首次上电清单 | 全部为 A 级记录 |
