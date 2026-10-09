# 证据矩阵

| 等级 | 含义 |
|---|---|
| A | 真机实测（附日期、环境、原始数据） |
| B | 官方当前资料或当前 SDK 源码确认（含项目方提供的官方信息、官方论坛公开帖） |
| C | 公开历史仓库确认（`enpeizhao/episode_robot` @ `9ede794`，2025-02-20） |
| D | 根据源码和系统结构作出的合理推断 |
| E | 尚未验证的假设 |

**当前 A 级条目数量：0。**

## 硬件与系统

| 结论 | 等级 | 来源 | 升级到 A 的方法 |
|---|---|---|---|
| 6 自由度、步进 + 行星减速、编码器、FOC 闭环驱动 | B | 项目方资料、官方 README | — |
| PCAN / USB2CAN 访问 CAN | B | 项目方资料、FAQ tid=1932 | — |
| 供电 12 V / 10 A；标称负载 500 g；标称半径 510 mm | B（厂商标称） | 项目方资料 | 实测负载/工作空间 |
| 重复定位精度 | 厂商标称 ±0.03 mm（“非国标测试方法，仅供参考”） | tid=1217（B，厂商声明，非本项目实测） | 外部测量 |
| 关节物理范围 J1 0–340 … J6 0–335 | B | 项目方资料 | 软限位逐关节确认 |
| ROS 偏移 [180,90,83,30,110,30] | B | 项目方资料 | 符号验证流程 |
| ROS 模型：`joint = motor − offset`，符号全部 +1 | **B** | 官方 `robot_interface_node.py`；官方 URDF 限位（如 J3 [-83,80]）独立印证 | 真机目视方向 |
| （已推翻）此前暂取 [+,−,−,+,−,+] | E → 错误 | 由官方源码否定 | — |
| 竖直姿态 = 回零后命令 [180,90,83,30,110,30] | C | `MotorControlServer.__init__` | 目视 + `get_motor_angles` |
| 竖直姿态 = URDF 零位 | B | 官方节点公式 `motor − offset` | RViz 对照 |
| DH 参数表 | B | 资料中心 tid=1217（与历史客户端一致） | `get_T` 对比 FK |
| 电机→DH 映射公式 | C | `calculate_T_based_on_degrees` | 同上 |
| 官方当前 DH θ 范围与电机范围映射完全一致；较窄 qlim 只存在于历史客户端 | B | tid=1217 | — |
| 减速比 [25,20,25,10,4,1] | C | `EpisodeAPP.motor_ratios` | — |
| 编码器在电机侧（磁铁 + 驱动板） | C | BOM 10077 | 拆机确认 |
| 回零为堵转电流检测，阈值 1500/1500/1500/400/400/1000 | B | FAQ tid=1953 | 校准软件读取 |
| 驱动固件 V5.1.5D | B | FAQ tid=1943 | 实机读取版本 |
| 校准软件回零超时 60 s | B | FAQ tid=1260 | — |
| None = 该关节 CAN 不通 | B | FAQ tid=1932 | 拔线实验 |
| J5 可能皮带打滑/提前回零 | B | FAQ tid=1996 | 观察 |
| 夹爪为 CAN→STM32 节点（C14 继电器，A1 PWM，7.4 V） | B | FAQ tid=1997 | — |
| CAN 波特率 | **未知（TBD）** | — | 抓包 |
| CAN ID 映射、帧格式 | **未知（TBD）** | — | 抓包 |
| 软件急停到静止时间 | **TBD** | — | 实测 |
| TCP RTT（真机） | **TBD** | — | `latency_analyzer` |

## 官方软件行为

| 结论 | 等级 | 来源 |
|---|---|---|
| 帧格式 8 字节大端长度 + JSON | B（当前）/ C（历史帧头相同，payload 为 pickle） | 项目方 / 源码 |
| 历史服务器 `pickle.loads` 网络数据 | C | `handle_client` |
| 历史服务器单次 `recv(8)` | C | 同上 |
| 每连接一线程、无锁 | C | `start_server` |
| 并发指令导致 CAN 阻塞/读失败 | B | tid=1217 原文；官方节点注释“否则会导致CAN堵塞” |
| 运动回复 = 预计耗时 | C | 同上 |
| MoveL 先回复后阻塞执行 | C | 同上 |
| 未知命令不回复 | C | 同上 |
| `home` 回复硬编码 5 | C | 同上 |
| 当前官方 API 含 emergency_stop / get_T / get_pose 等 | B | 项目方资料、tid=1217 |
| emergency_stop 1/0、get_pose(order)、get_T、读失败整体 None、move_xyz 8 参数、servo 0–110、set_free_mode 0.1 | B | tid=1217 API 参考 |
| dynamic_move `[{"1":q…}, pulses, max_speed]`，需驱动板 Response=None | B | 官方 SDK Demo V1.2.1 |
| 官方 Python SDK V1.2.1 仍单次 `recv(8)`、无超时、异常转 None | B | `sdk_demo.py` |
| 无软停；急停后上位机忽略后续指令直到恢复；急停不作用于舵机夹爪 | B | tid=1217、tid=1191 |
| 官方 ROS2 Actions/Services 字段定义 | B | student_ros V1.0 `robot_arm_interfaces` |
| 官方 ROS2 节点：进度为“模拟进度”、取消不停电机、FJT 反馈把计划点当 actual、服务返回无时效缓存 | B | `robot_interface_node.py` |
| 官方核心支持流式执行 `dynamic_move_with_velocity` | B | 同上 |
| 官方 `joint_states` 发布实测（stamp 为发布时刻） | B | `publish_joint_states()` |

## 本项目结果

| 结论 | 等级 | 说明 |
|---|---|---|
| SDK/控制层自动化测试 | Mock 环境结果 | 运行 `python -m pytest -q` |
| 故障注入矩阵 | Synthetic / Mock Data | `reports/mock_fault_injection_report.md` |
| Mock 本机 RTT | Synthetic / Mock Data | `reports/mock_latency_localhost.md` |
| ROS2 节点逻辑 | 桩测试结果 + ROS2 Jazzy 运行（WSL2） | Mock 环境结果，`reports/ros2_runtime_wsl.md` |
| ros2_control | Jazzy 编译 + gmock + controller_manager 运行 | Mock 环境结果，`reports/ros2_runtime_wsl.md` |
