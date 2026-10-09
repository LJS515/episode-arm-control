# 14 命令仲裁

> 状态：实现 + 测试（`episode_hardware/command_arbiter.py`，`tests/test_command_arbiter.py`）。

## 问题

可能下发运动的来源：ROS Action、应用脚本、示教程序、操作员面板、安全层自身（停止）。官方服务器不做互斥（C/D）。需要一个地方回答：“现在谁拥有机械臂？”

## 模型

```
submit(kind, payload, source, priority, timeout) → CommandTicket
    command_id   唯一 ID（cmd-N）
    source       来源（ros_action / api / operator / safety ...）
    submitted_at 提交时间（注入时钟）
    timeout_s    超时（激活后计时；排队中按提交时间计时）
    status       PENDING / ACTIVE / SUCCEEDED / FAILED / CANCELLED / PREEMPTED / REJECTED / TIMED_OUT / ABORTED_ESTOP
    cancel_requested  物理执行者需要停止
```

## 忙时策略（`BusyPolicy`，可配置）

| 策略 | 行为 | 适用 |
|---|---|---|
| REJECT（默认） | 有 active 时直接拒绝 | 最安全；调用方显式重试 |
| QUEUE | 有界 FIFO（默认 4），active 完成后提升下一条 | 脚本化序列；提升时重新经过安全检查 |
| PREEMPT | 新命令来源在 `preempt_sources`（默认 operator/safety）且优先级 ≥ 当前 → 旧命令 PREEMPTED | 操作员接管 |

## 急停

`emergency()`：立即把 active 与全部排队票据标为 ABORTED_ESTOP、设置急停标志；之后任何 `submit` 抛 `EStopActiveError`，直到 `clear_emergency()`。急停从不抛异常、不等待队列。

## 取消 ≠ 停止

`cancel()` 只改变票据状态并设置 `cancel_requested`。**物理停止**由执行者（`SafeRobotController.cancel` → `request_stop`）负责。把两者分开是有意的：仲裁器是纯数据结构，可以被完全单元测试；“如何停”取决于硬件能力（软停或急停）。

## 线程安全

* 所有公共方法在 `RLock` 内修改状态。
* 回调（on_activate / on_finish）收集后在**锁外**触发，避免回调里再调用仲裁器造成死锁。
* 测试：20 线程 Barrier 竞争恰好一个胜出；急停与持续提交并发无异常、结束态干净。

## 排队命令的执行

仲裁器提升排队票据时设置 `from_queue=True` 并触发 on_activate；控制器把它放入待执行列表，在下一次 `poll_once` 中**重新做安全检查**（状态可能已变化）后执行。直接在回调中执行会因状态机仍在 MOVING 而被拒绝——这是开发中遇到并修正的问题。
