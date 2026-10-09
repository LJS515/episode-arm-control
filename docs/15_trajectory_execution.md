# 15 轨迹执行

> 状态：实现 + 测试（`trajectory_monitor.py`）。真实轨迹误差：**TBD**。

## 三个量必须分开

| 量 | 定义 | 来源 | 可以用来判定完成吗 |
|---|---|---|---|
| desired | 参考轨迹在 t 时刻的值 | 本项目按“起点→终点在预计耗时内线性插值”构造（官方内部速度曲线未知，E） | 否，仅用于记录/可选跟踪误差 |
| actual | 实测编码器角 | `JointStateCache`（电机侧） | 是 |
| error | `goal − actual`（终点误差）与 `desired − actual`（跟踪误差） | 计算 | 终点误差是 |

**计划点不能当作 actual feedback。** 如果 `/joint_states` 发的是计划点，RViz 里机械臂永远“完美到位”，即使真机卡住。

## 完成判定（`TrajectoryMonitor`）

* SUCCEEDED：所有关节 `|goal − actual| ≤ tol`（默认 1°，TBD）**连续**保持 `settle_time_s`（默认 0.3 s）。中途离开容差则重新计时。
* FINAL_ERROR_EXCEEDED：已过预计耗时、机械臂已静止（相邻采样变化 ≤ eps 持续 settle 时间）、但仍在容差外——典型是皮带打滑或提前回零导致零点偏移（FAQ tid=1996）。
* TIMED_OUT：超过 `expected × 1.5 + 2 s`（可配）仍未稳定，或无有效反馈。
* TRACKING_ERROR（默认关闭）：期望曲线未知时启用会误报。
* CANCELLED：外部取消，终态粘滞。

## 已知局限

* “静止”用“每个采样的变化 ≤ eps”判定，依赖轮询频率。极慢的运动可能被误判为静止 → 提前报 FINAL_ERROR_EXCEEDED（开发中测试暴露过；仲裁器超时测试因此改用更短超时）。更好的做法是用速度阈值（deg/s）并结合预计耗时——列入 roadmap。
* 电机侧编码器无法反映 J5 皮带之后的误差（C/B），终点误差检查对皮带打滑只有**部分**检测能力：如果打滑发生在运动中，电机编码器读数可能“到位”而末端没到位。

## MoveJ、MoveL 与插值

| | MoveJ（`angle_mode` / `move_xyz_rotation`） | MoveL（`move_linear_xyz_rotation`） |
|---|---|---|
| 插值空间 | 关节空间 | 笛卡尔空间 |
| 末端路径 | 一般是曲线 | 直线 |
| 官方实现 | 一条命令 | IK 产生关节点列表 → 逐段执行；回复总时间后阻塞执行（C） |
| 风险 | 路径难预测（避障） | 经过奇异点附近时关节速度可能激增；任一点无解则整段无解（返回 -1） |

## FollowJointTrajectory 的执行方式（`episode_ros2`）

官方 TCP API 是点到点的，无法跟随带时间戳的连续轨迹。本项目做法：

1. 把轨迹点映射为电机角并检查全部点的限位（goal 回调中，接受前）。
2. 抽稀：相邻路点关节变化 ≥ `waypoint_min_spacing_deg`（默认 5°）才保留，终点必保留。
3. 每个路点作为一次 `move_joints` 执行，等待 Monitor 判定后进入下一个（**走走停停**，忽略 `time_from_start`）。
4. 反馈每 0.1 s 发布 desired（当前路点）/ actual / error。

这是诚实的折中：语义正确（每段都由实测判定），但不满足 MoveIt 的时间约束。改进路径见 `16`。
