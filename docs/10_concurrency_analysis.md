# 10 并发分析

> 状态：分析 + 实现 + 多线程测试（Mock）。

## 共享资源清单

| 资源 | 访问者 | 风险 | 保护 |
|---|---|---|---|
| CAN 总线（经官方进程） | 轮询读、运动命令、急停、夹爪、free mode | 命令交错、读写互相阻塞 | `hardware_lock`（急停除外） |
| 运动所有权 | ROS Action、应用 API、示教 | 两个运动同时下发 | `CommandArbiter` 单一 owner |
| 关节状态 | 轮询线程写，多方读 | 撕裂读（部分新部分旧） | `JointStateCache` 内 `Lock`，整体快照 |
| 状态机 | 调用方、轮询、监督器 | 竞态转换 | `RLock` + 转换表 |
| TCP 长连接 | 多线程调用同一 client | 请求/响应错位 | client 内 `RLock`，一连接一请求在途 |

## 历史官方服务器的并发问题（C/D）

1. 每个客户端连接一个线程，命令分发无锁 → 两个客户端可交错发 CAN（D）。
2. `move_linear_xyz_rotation` 回复后在**同一连接线程**里阻塞执行；同一连接上的后续请求（包括可能的停止）要等执行完。客户端若想打断，只能开新连接——而新连接又落到另一个线程，与执行中的线程并发访问总线（D）。
3. 结论：在官方服务器之上，**本项目必须自己保证“同一时间只有一个运动命令”**，且急停走独立连接。

## MultiThreadedExecutor ≠ 线程安全

ROS2 的 `MultiThreadedExecutor` 只决定“回调能不能并行跑”。是否互斥由 CallbackGroup 决定：

| 组 | 类型 | 内容 | 理由 |
|---|---|---|---|
| `poll_group` | MutuallyExclusive | 反馈定时器、诊断定时器 | 轮询自身不重叠 |
| `action_group` | Reentrant | goal/cancel/execute | execute 长时间运行时，cancel 回调必须能进来 |
| `srv_group` | MutuallyExclusive | 普通服务 | 短事务串行即可 |
| `estop_group` | MutuallyExclusive（独立） | 急停服务 + 话题 | 不排在其他服务后面 |

即使回调互斥，不同组之间仍然并行——所以硬件访问必须在**数据层**加锁（`hardware_lock`），而不是寄希望于回调组配置。

## 锁顺序与死锁避免

* 规则：`hardware_lock` 内只做一次硬件调用，不调用状态机/仲裁器（它们可能触发回调）。
* `CommandArbiter` 在锁内只修改数据，收集需要通知的票据，**锁外**触发回调（`_fire`）。
* 状态机监听器在锁外调用；钩子异常被吞并记录，不破坏状态机。
* 急停：`hw.emergency_stop()` 不拿 `hardware_lock`（测试 `test_estop_works_even_if_reads_hang` 在锁被占用时仍能急停）。Mock/TCP 后端各自有内部锁或独立连接。

## 读超时与过期阈值的关系

轮询线程在 `hardware_lock` 内做一次读；若读超时 5 s，则：(1) 这 5 s 内命令无法下发；(2) 缓存在 5 s 后才记一次失败；连续 3 次失败需要 15 s 才进入 FAULT。因此：**轮询读超时 < `stale_after_s`**（ROS 节点默认 0.3 s < 0.5 s）。`examples/mock_demo.py` 第 5 步演示了用 0.3 s 超时时，断开反馈后约 1 s 内进入 FAULT（Mock）。

## 测试证据（Mock）

| 测试 | 内容 |
|---|---|
| `test_concurrent_submit_single_owner` | 20 线程用 Barrier 同时提交，恰好 1 个成功 |
| `test_estop_during_concurrent_submits` | 4 线程持续提交时急停，结束后无 active、队列空 |
| `test_thread_safe_snapshots` | 写线程持续写，3 个读线程 6000 次快照无撕裂 |
| `test_concurrent_calls_are_serialised` | 4 线程共用长连接客户端，无错位 |
| `test_estop_not_blocked_by_hanging_command` | 一个请求挂起时急停 < 0.5 s 送达 |
