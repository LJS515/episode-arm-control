# 19 故障注入报告

> **Synthetic / Mock Data.** 全部结果来自 Mock（`SimulatedArm` + FakeClock，20 Hz 轮询），由 `tools/generate_mock_report.py` 生成（原始表：`reports/mock_fault_injection_report.md`）。时间为**仿真秒**，描述软件反应，不代表真机。真机对应结果：**TBD**。

## 结果

| ID | 场景 | 最终状态 | 票据 | 锁存故障 | 拒绝原因 | 检测/稳定（仿真 s） |
|---|---|---|---|---|---|---|
| F01 | 未回零即运动 | INACTIVE | — | — | NotHomedError | — |
| F02 | 正常关节运动 | READY | SUCCEEDED | — | — | 1.00（终点误差 0.000°，Mock 理想模型） |
| F03 | 运动中再下发运动 | MOVING | — | — | StateRejectedError | — |
| F04 | 运动中取消 | READY | CANCELLED | — | — | 0.40（J1 停在 183.00°，目标 200°） |
| F05 | 运动中软件急停 | ESTOP | ABORTED_ESTOP | — | 之后运动：EStopActiveError | — |
| F06 | READY 时连续 3 次读失败 | FAULT | — | READ_FAILURES | — | 0.15 |
| F07 | 运动中反馈丢失 | FAULT | FAILED | READ_FAILURES | — | 0.15（并已停止机械臂） |
| F08 | J3 返回 None（FAQ tid=1932 模式） | READY | — | — | StaleStateError | — |
| F09 | J5 终点偏 4°（皮带打滑，FAQ tid=1996 模式） | FAULT | FAILED | FINAL_ERROR_EXCEEDED | — | 1.10 |
| F10 | J1 卡死 + 整体变慢 | FAULT | FAILED | FINAL_ERROR_EXCEEDED | — | 0.70 |
| F11 | 目标越限 | READY | — | — | JointLimitError | — |
| F12 | speed_ratio = 2.0 | READY | — | — | SpeedRatioError | — |

## 解读

* **F03 拒绝来源**：第二条运动先被 Supervisor 以“非 READY”拒绝（StateRejectedError），仲裁器是第二道防线（直接调用仲裁器时为 CommandRejectedError，见 test_command_arbiter）。
* **F06/F07 检测时间 0.15 s** = 3 个轮询周期 × 50 ms，即阈值 `max_consecutive_read_failures=3` 的直接结果。真机上还要加上每次读的超时时间（`10` 节）。
* **F07 是故障注入发现的缺陷**：最初版本在运动中锁存故障后，票据仍是 ACTIVE、机械臂没有被要求停止。修复后（`_abort_on_fault`）票据 FAILED，并发出软停（Mock 支持）或急停（官方 TCP 后端不支持软停）。新增两条回归测试。
* **F10 判为 FINAL_ERROR_EXCEEDED 而非 TIMED_OUT**：在 Mock 的减速模型下，其余关节每个采样周期的位移小于静止阈值，被判“已静止且未到位”。这暴露了“按采样变化判静止”依赖轮询频率的局限（见 `15`）。两种判定都会锁存故障并停止，安全结果一致，但诊断信息不同。
* **F08 不进入 FAULT**：单次 None 只阻止运动；连续 3 次才锁存 READ_FAILURES（见 test_consecutive_read_failures_fault_in_ready 与 tid=1932 的“部分关节 None”场景）。

## TCP 层故障注入（test_client）

| 注入 | 客户端表现 |
|---|---|
| 回复延迟 1 s（接收超时 0.5 s） | ReceiveTimeout/CommandTimeout，< 1 s 返回 |
| 丢 1 次回复 | 读请求自动重试成功（服务器收到 2 次）；运动请求不重试（服务器只收到 1 次） |
| 回复截断到 12 字节 | TruncatedFrameError |
| 垃圾 payload | MalformedPayloadError；长连接下下一次请求自动重连成功 |
| 长度头 2^40 | FrameTooLargeError（未分配内存） |
| 读完请求即关闭 | ConnectionClosedError |
| 回复按 3 字节分片 | 正确重组 |
| 一个请求挂起时急停 | 急停 < 0.5 s 送达（专用连接） |

## 真机故障注入计划（TBD，安全前提见 `12`）

| 场景 | 方法 | 预期 |
|---|---|---|
| CAN 断线 | **断电后**拔末端关节 CAN，上电 | None → 3 次后 FAULT |
| 上位机关闭 | 运动中关闭上位机 | 读失败 → FAULT → 急停尝试失败 → 日志提示使用硬件急停 |
| 软件急停 | 低速运动中 `emergency_stop.py` | 记录到静止时间（A） |
| 终点误差 | 末端加载（≤ 标称 500 g）或人为阻挡（低速、低回零电流） | FINAL_ERROR_EXCEEDED |
