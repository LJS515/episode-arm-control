# 09 ROS2 接口分析

> 状态：分析 + 本项目接口实现（`episode_ros2`，stub 测试 + **ROS2 Jazzy（WSL2）运行冒烟测试 17/17，Mock 环境**，见 `reports/ros2_runtime_wsl.md`）。官方 ROS2 包未测试。

## 官方 ROS2 接口（B，student_ros V1.0，字段定义已获得）

| 名称 | 类型 | 推测用途 |
|---|---|---|
| `FollowJointTrajectory` | Action | MoveIt 执行 |
| `AngleMode` | Action | 关节空间点到点 |
| `MoveXyzRotation` | Action | 位姿（IK）点到点 |
| `LinearMove` | Action | 直线 |
| `ReadMotorAngles` | Service | 读电机角 |
| `GripperControl` / `ServoGripper` / `SetFreeMode` | Service | 末端/自由模式 |
| `joint_states` / `motor_angles` | Topic | 状态 |

## 需要重点确认的问题 —— 已由官方源码回答（B，见 `reports/official_material_review.md`）

| # | 官方源码中的答案 |
|---|---|
| Q1 | `joint_states` 发布实测（`motor − offset`，stamp 为发布时刻）；但 `FollowJointTrajectory` 反馈把**计划点当 actual** |
| Q2 | `AngleMode`/`MoveXyzRotation` 是**按时间的模拟进度**，结果里“假设达到目标位置” |
| Q3 | 取消**不发停止**，直线运动注释“运动已开始，无法完全停止” |
| Q4 | Action 用 Reentrant 组并行调用控制核心；作者注释定时器若可重入“会导致 CAN 堵塞” |
| Q5 | 运动走 Action，服务做短事务；但读角度服务返回**无时效检查的缓存** |

原始问题清单：

| # | 问题 | 为什么重要 | 本项目做法 |
|---|---|---|---|
| Q1 | `joint_states` 发布的是实测还是计划点？ | 若发布计划点，RViz/MoveIt 看到的是“愿望”不是“事实”（D：历史 Python 客户端完成判定是 sleep，存在同类风险） | 只发布实测；样本过期/不完整时不发布；只在有**新**样本时发布 |
| Q2 | Action 反馈的进度是如何计算的？ | 如果按时间比例“模拟进度”，进度 100% 不代表到位 | 反馈中 desired/actual/error 分开，actual 来自缓存 |
| Q3 | 取消 Action 时是否真的停止电机？ | Action 取消只是 ROS 层状态，不自动停止硬件 | 取消 = `controller.cancel()` → 物理停止（不支持软停则急停） |
| Q4 | 多个 Action/Service 回调是否会并发访问 CAN？ | MultiThreadedExecutor 下回调可并行 | 硬件锁 + 单一运动所有权 |
| Q5 | Service 是否被用于长时间运动？ | 阻塞调用方，无取消、无反馈 | 运动只走 Action；Service 只做短事务 |

## 本项目的接口设计

| 接口 | 类型 | QoS / 组 | 语义 |
|---|---|---|---|
| `/joint_states` | Topic | reliable, depth 10, volatile | 实测，模型弧度，stamp = 采样时刻 |
| `/episode/motor_angles` | Topic | 同上 | 电机角原值 |
| `/episode/robot_state` | Topic | reliable + transient_local, depth 1 | 状态机状态，迟到订阅者也能拿到 |
| `/diagnostics` | Topic | 默认 | 新鲜度、状态、仲裁 |
| `/episode/estop` | Topic(sub) | reliable, `estop_group` | 软件急停按钮 |
| `follow_joint_trajectory` | Action | `action_group`（Reentrant） | 路点点到点执行，反馈 desired/actual/error，取消=物理停止 |
| `~/home` `~/stop` `~/clear_fault` `~/declare_homed` `~/emergency_stop` | Trigger | `srv_group` / `estop_group` | 短事务；`home` 只“启动回零” |
| `~/release_estop` `~/gripper` `~/set_free_mode` | SetBool | `srv_group` | `release_estop.data` = 操作员确认硬件已复位 |

### QoS 选择理由

* `joint_states` 用 reliable：`robot_state_publisher` 与 MoveIt 默认以 reliable 订阅，best-effort 发布者与 reliable 订阅者**不兼容**，会静默收不到数据。
* `robot_state` 用 transient_local：状态是“当前值”语义，后启动的诊断节点也要知道当前是否 ESTOP。
* 急停话题 reliable、depth 10：不能因队列太浅丢掉急停。

### 为什么不用官方自定义 Action

字段定义未获得。本项目只用标准接口，保证可编译；自定义 `MovePose.action` 写在 `interfaces_proposal/` 作为提案，正式实现前要与官方定义对齐（适配，不替换）。

## Goal 接受条件（`_on_goal`）

关节名集合一致、非空、所有点有限且映射后在电机限位内、当前无其他目标在执行、状态机允许运动（READY 且已回零）。任一不满足即 REJECT——**在接受之前拒绝**，比接受后再 abort 更清晰。
