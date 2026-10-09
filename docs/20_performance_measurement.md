# 20 性能测量

> 状态：工具完成；仅有 **Mock 环境结果**。所有真机指标：**TBD / 待真机测量**。不得引用 Mock 数字作为机械臂性能。

## 指标定义与测量方法

| 指标 | 定义 | 工具 / 方法 | 当前值 |
|---|---|---|---|
| TCP RTT（读） | `get_motor_angles` 请求发出到回复解析完成 | `tools/latency_analyzer.py --env "..."` | 真机 TBD |
| 轮询有效频率 | 每秒成功样本数 | `/diagnostics` rate_hz；`diagnostics_node` | 真机 TBD |
| 反馈年龄 | 使用时刻 − 采样时刻 | `JointStateSnapshot.age_s` | 真机 TBD |
| 命令到首个 CAN 帧延迟 | TCP 请求时间戳 → 首帧 | `tcp_trace.py` + PCAN-View 同机时钟 | TBD |
| 软件急停到静止 | 急停发出 → 速度≈0 | 反馈速度 + 视频 240 fps 交叉验证 | TBD |
| 终点误差 | `goal − actual`（电机侧） | `TrajectoryMonitor.result()` | TBD |
| 重复定位精度 | 同一目标多次往返，末端位置离散度 | 外部测量（千分表/视觉标定板），参考 ISO 9283 思路 | TBD（不可引用厂商“高精度”作为实测） |
| 跟踪误差 | `desired − actual` | `trajectory_analyzer.py`（注意 desired 为构造值） | TBD |
| 回零重复性 | 多次回零后同一电机角对应末端位置差 | 外部测量 | TBD |

## Mock 结果（Synthetic / Mock Data）

`reports/mock_latency_localhost.md`，Windows 11 本机回环，n=300：

| 连接策略 | p50 | p95 | p99 | max |
|---|---|---|---|---|
| per_command | 1.00 ms | 15.91 ms | 16.30 ms | 24.63 ms |
| persistent | 0.15 ms | 0.26 ms | 0.31 ms | 0.48 ms |

只能说明：在本机上，**客户端建连开销**远大于 Mock 的处理开销；不代表上位机或 CAN 的耗时。

## 真机测量规程（模板）

1. 记录环境：上位机版本、驱动固件、PC 型号/OS、USB 口、是否同机。
2. 机械臂静止，测 RTT（两种连接策略 × 1000 次）。
3. 低速运动中再测一次（总线负载上升时的变化）。
4. 每组输出 JSON（`--json`），原始数据入 `reports/`，在 `EVIDENCE_MATRIX.md` 中把对应条目改为 A。
5. 报告中写明样本数、分位数、最大值，不写“提升 xx%”这类无基线的数字。

## 性能预算（设计目标，非测量）

| 环节 | 目标 | 理由 |
|---|---|---|
| 轮询周期 | 50 ms（20 Hz） | 足以检测秒级运动的到位与停滞；真机可达性 TBD |
| 读超时 | 0.3 s | < 过期阈值 0.5 s |
| 故障检测 | ≤ 3 周期 + 3 × 读超时 | 由阈值决定 |
