# 16 ros2_control 设计

> 状态：**已在 ROS2 Jazzy（WSL2，ros2_control 4.48）编译（0 警告）、gmock 5/5、controller_manager + JTC 运行通过**——仅 Mock 后端与 Python TCP Mock 上位机，见 `reports/ros2_runtime_wsl.md`。未接真机。

## ros2_control 解决什么问题

把“硬件驱动”与“控制算法”解耦：硬件只暴露状态接口（position/velocity…）与命令接口（position…），`controller_manager` 周期性调用 `read() → update(controllers) → write()`；`joint_trajectory_controller`（JTC）负责插值与跟踪，`joint_state_broadcaster` 发布 `joint_states`。MoveIt 只需与 JTC 的 `FollowJointTrajectory` 对接。

## 本实现

| 部分 | 内容 |
|---|---|
| 类 | `episode_ros2_control::EpisodeSystemHardware : hardware_interface::SystemInterface` |
| 状态接口 | 每关节 `position`、`velocity`（模型弧度） |
| 命令接口 | 每关节 `position` |
| 后端 | `MockBackend`（一阶限速跟踪）、`VendorTcpBackend`（官方上位机 TCP/JSON）、`VendorCoreBackend`（占位：官方 core API 未公开） |
| 反馈线程 | 独立线程按 `poll_rate_hz` 读取，`read()` 只拷贝缓存——避免在控制循环里阻塞 TCP |
| 过期检测 | `read()` 在样本 age > `stale_after_s` 或连续失败 ≥ 3 时返回 ERROR |
| 限位 | `write()` 中检查软限位，越界返回 ERROR |
| NaN | 尚无命令（JTC 初始化前）时跳过写 |
| 激活 | 等待新鲜样本，命令 = 当前位置，防止跳变 |
| 去激活/错误 | 运动中则 `emergency_stop`（无文档化软停） |
| 映射 | 符号/偏移/限位作为 `<hardware><param>` 传入（xacro） |

## 核心矛盾：流式接口 vs 点到点 API

JTC 每周期写一个插值后的位置；官方 `angle_mode` 每条命令都会重新规划一段带加减速的运动。逐周期转发 → 上位机被淹没、驱动不断重启运动曲线 → 抖动。参考实现用 **死区（0.2°）+ 最小发送间隔（0.1 s）** 限流，这能让慢速轨迹勉强跟随，但 JTC 的轨迹容差对快速轨迹没有意义。

可选方向（优先级）：

1. 官方是否提供流式入口？`dynamic_move` 名称暗示“动态/流式”（E）——需要官方文档或抓包确认参数与语义（是否覆盖当前目标、是否有超时保护）。
2. 若 `episode_controller_core` 提供 C API（如 `set_target`、`read_positions`），写 `VendorCoreBackend`，直接在路线 B 上实现流式控制。
3. 否则保留 Python `FollowJointTrajectory` 服务器（`episode_ros2`），明确“走走停停”的语义。

## 编译与验证计划（Jazzy，TBD）

1. `colcon build --packages-select episode_ros2_control`，`-Wall -Wextra -Wpedantic` 无警告。
2. 用 `ros2_control_test_assets` 写 gmock 测试：加载插件 → configure → activate → read/write 循环（mock 后端）。
3. 在官方 URDF 中 include xacro，`ros2 control list_hardware_interfaces` 检查接口。
4. Mock 后端下发 JTC 轨迹，`rqt_plot` 观察 `joint_states` 跟随（仿真结果）。
5. 真机：先 `speed_ratio 0.1` + 单关节小幅轨迹，记录跟踪误差（A）。

## 为什么同时保留 Python 节点

ros2_control 路线依赖未确认的流式能力；Python 节点在点到点 API 上语义正确、可测试、已有 stub 测试。两者共享同一套映射与限位参数（`tools/validate_configuration.py` 检查一致性）。
