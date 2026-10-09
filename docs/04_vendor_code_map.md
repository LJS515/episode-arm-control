# 04 官方代码地图

> 状态：静态分析。来源：公开仓库 `enpeizhao/episode_robot` @ `9ede794`（2025-02-20，证据 C）。当前官方 SDK 源码未获得，因此以下结论对“当前版本”最多是 D。详细发现见 `reports/vendor_source_review.md`。

## 仓库结构（历史公开版）

| 文件 | 作用 |
|---|---|
| `0.server.py` | 启动 `MotorControlServer(resume, usb_id)` 并监听 `--ip/--port`（默认 12345）；`--usb_id` 选择 PCAN 通道（README：最多 6 台机械臂） |
| `episodeServer.py` | `MotorControlServer`（服务器端，依赖未公开的 `MotorControl`）+ `EpisodeAPP`（客户端，含 DH 模型） |
| `1.demo_episode_with_gripper.py` | 6D 抓取（GraspNet + RealSense），调用 `EpisodeAPP` |
| `2.air_sucker.py` / `2.demo_VLM_grasp.py` / `3.demo_VLM_handler.py` | 3D 抓取、VLM/LLM 示例 |
| `3.teach_mode.py` | 示教：一台 free mode 读角度，另一台复现；用“窗口内角度变化 < 阈值”判稳 |

## `MotorControlServer.handle_client` 调用关系

```
accept() ─► thread(handle_client)
              loop:
                recv(8)                    ← 单次 recv，非 recv_exact
                recv 循环读 body           ← EOF 时 break 后仍反序列化
                pickle.loads(data)         ← 网络数据反序列化
                dispatch command['action']
                  home              → motor_control.move_home(); result = 5 (硬编码)
                  gripper_on/off    → result = 0.05
                  servo_gripper     → result = 1
                  robodk_simu       → result = 0.05
                  set_free_mode     → result = motor_control.set_free_mode(...)
                  get_motor_angles  → result = read_motor_angles()
                  angle_mode        → execute_motors_degrees_normal(angles, speed_ratio)
                  move_xyz_rotation → ik_normal_move → execute_motors_degrees_normal / -1
                  move_linear_...   → ik_linear_move → 先回复总时间，再阻塞执行全部步
                  tiktok_rotate_point (演示用)
                  其他               → result = None（不回复）
                if result is not None: sendall(len) + sendall(pickle)
```

## 对本项目的映射

| 官方行为 | 本项目对应 |
|---|---|
| 长度前缀帧 | `protocol.py`（同格式，JSON，加固） |
| 每连接一线程、无锁 | `CommandArbiter` + `hardware_lock`；Mock Server 复刻该线程模型以便测试 |
| 回复=预计耗时 | `MotionAck.estimated_duration_s` + `TrajectoryMonitor` |
| 未知命令不回复 | 客户端接收超时；legacy Mock 复刻 |
| `set_free_mode` 可能不回复 | `LEGACY_MAY_NOT_REPLY`，超时视为无回复并告警 |
| 回零后到竖直姿态 | `VERTICAL_MOTOR_DEG`，`declare_homed()` 的依据 |
| DH 模型 + 角度换算 | `VENDOR_DH` 映射 + `kinematics.py`；测试逐字对照官方公式 |
| 示教判稳（窗口阈值） | `TrajectoryMonitor` 的 settle 逻辑（时间窗 + 容差） |

## 未公开部分（不能推断实现）

`MotorControl` 类（CAN 收发、运动规划、回零序列）、`ik_normal_move/ik_linear_move` 实现、`execute_motors_for_all_steps` 的时序。这些属于官方二进制/未公开代码，本项目只把它们当黑盒。
