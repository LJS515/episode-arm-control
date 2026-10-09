# 03 软件清单

> 状态：资料整理（静态）。

## 官方软件（非本项目）

| 组件 | 形态 | 证据 | 本项目如何使用 |
|---|---|---|---|
| Windows 上位机 | 应用程序，内含 TCP JSON 服务器、IK、PCAN 访问 | B | 路线 A：`EpisodeClient` 通过 TCP 调用 |
| 校准软件 | 独立工具（回零参数、驱动参数） | B | 不使用；回零参数以官方为准 |
| 官方 Python SDK | `EpisodeAPP`（历史版本 pickle，当前 JSON） | C/B | 不依赖；接口形状参考它 |
| ROS2 包 | `episode_controller` 节点、Actions/Services/Topics、URDF/SDF/STL/Gazebo | B | 路线 B 的参照；本项目节点不替代它 |
| `episode_controller_core` | 编译后的动态库（控制核心） | B | 未使用（API 未公开） |
| IK 动态库 | 编译后 | B | 未使用；本项目参考 IK 仅用于 Mock |
| PCAN 驱动 | Windows 驱动 / Linux `pcan` 内核模块 + `pcaninfo` | B | 环境前置条件 |
| 驱动器固件 | V5.1.5D | B | 记录在每次测量中 |
| 夹爪 STM32 固件 | 由官方提供源码（需索取） | B | 不涉及 |

## 运行环境

| 路线 | OS | 关键依赖 |
|---|---|---|
| A | Windows（官方上位机） | PCAN 驱动；本项目 SDK 可在任意 OS 通过网络访问（建议仅 localhost） |
| B | Ubuntu 24.04 + ROS2 Jazzy（B） | `pcan` 内核模块（Secure Boot 可能阻止未签名驱动；换内核需重装，tid=1932） |

## 本项目软件

| 包 | 语言 | 依赖 | 状态 |
|---|---|---|---|
| `episode_sdk` | Python ≥3.10 | 仅标准库 | 实现 + 测试 |
| `episode_hardware` | Python ≥3.10 | 标准库；配置加载用 PyYAML | 实现 + 测试 |
| `episode_ros2` | Python / rclpy | control_msgs, sensor_msgs, std_srvs, diagnostic_msgs | Jazzy 构建运行 + 冒烟测试（Mock） |
| `episode_ros2_control` | C++17 | hardware_interface, pluginlib, rclcpp_lifecycle | Jazzy 编译 + 测试（Mock） |
| `tools/` | Python | 标准库 | 实现 + 合成数据测试 |

## 开发环境（本次实际使用）

Windows 11、Python 3.12.8、pytest 9.0.3、PyYAML 6.0.3；WSL2 Ubuntu 24.04 + ROS2 Jazzy（ros2_control 4.48、g++ 13.3、Python 3.12.3）。没有 MoveIt、没有 PCAN 硬件、没有机械臂。
