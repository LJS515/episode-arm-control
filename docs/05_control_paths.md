# 05 两条控制链

> 状态：静态分析（B/C/D）。图：`diagrams/windows_sdk_path.mmd`、`diagrams/ros2_control_path.mmd`。

## 路线 A：Windows SDK

```
Python / C++ 用户程序 → TCP → 8 字节大端长度 + UTF-8 JSON → 官方 Windows 上位机 → PCAN → CAN → 6 个驱动器
```

| 维度 | 说明 |
|---|---|
| 总线所有者 | 官方上位机进程 |
| 调用粒度 | 点到点命令（MoveJ `angle_mode` / 位姿 `move_xyz_rotation` / MoveL `move_linear_xyz_rotation`），每条命令内部自行规划 |
| 反馈 | 轮询 `get_motor_angles`；无推送 |
| 完成语义 | 回复 = 预计耗时（C），**非完成确认** |
| 停止语义 | 只有 `emergency_stop`（B），无文档化软停 |
| 并发 | 历史版本每连接一线程且无锁（C） |
| 安全 | 无认证；历史版本 pickle（C） |

## 路线 B：ROS2 直接控制

```
MoveIt 2 / 自定义 Client → ROS2 Action/Service/Topic → episode_controller 节点 → episode_controller_core(.so) → PCAN → CAN → 驱动器
```

| 维度 | 说明 |
|---|---|
| 总线所有者 | `episode_controller` 进程内的 control core |
| 接口 | `FollowJointTrajectory`、`AngleMode`、`MoveXyzRotation`、`LinearMove` Actions；`ReadMotorAngles`、`GripperControl`、`ServoGripper`、`SetFreeMode` Services；`joint_states`、`motor_angles` Topics（B） |
| 运行环境 | Ubuntu 24.04 + ROS2 Jazzy（B） |
| 不透明部分 | control core 与 IK 为二进制（B） |

## 两条路线不能同时用

两者都直接打开 PCAN 通道。PCAN 通道同一时刻通常只能被一个进程以独占方式使用（D，取决于驱动），即使能共享，两个独立规划器向同一组驱动器发命令也必然冲突。**规则：同一时刻只运行一条路线。** `hardware.launch.py` 只连接路线 A 的上位机。

## 本项目选择路线 A 作为主集成点的原因

1. 路线 A 的协议是**可观测**的（TCP 帧可被 `tools/tcp_trace.py` 记录），可以写 Mock 并做故障注入；路线 B 的 core 是黑盒 .so，接口不公开。
2. 路线 A 天然有进程边界：即使本项目代码崩溃，总线仍由官方进程管理。
3. ROS2 适配（`episode_ros2`）放在路线 A 之上，相当于用标准 ROS 接口 + 安全层“包裹”官方上位机。

代价：多一跳 TCP（延迟 TBD），点到点语义限制了连续轨迹跟踪（见 `15`、`16`）。

## 在路线 B 上能做什么（未实现，设计）

* 不改官方节点，在其前面加一个“安全代理”节点：订阅官方 `joint_states`/`motor_angles`，转发 Action 前做 Supervisor 检查，对官方 Action 做取消/超时监视。
* 需要官方 Action/Service 的字段定义（未获得），见 `episode_ros2/interfaces_proposal/README.md`。
