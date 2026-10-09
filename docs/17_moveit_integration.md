# 17 MoveIt 集成

> 状态：**MoveIt 2.12.4 已在 ROS2 Jazzy（WSL2）中用官方 URDF/STL 运行**：两条执行栈各 11/11（含碰撞检查生效验证），Mock 后端。官方模型包 `episode1_urdf_1113` 来自官方 ROS2 包 student_ros V1.0，未随本仓库分发。早期占位模型（9/9）仍可用 `model:=placeholder`。未接真机。详见 `reports/ros2_runtime_wsl.md`。

## 配置包 `moveit_config/`（ament 包名 `episode_moveit_config`）

| 文件 | 内容 |
|---|---|
| `config/episode1_official.srdf` | 官方模型：规划组、`vertical` 姿态、虚拟关节 world→base_link、碰撞矩阵（6 对 Adjacent + 8 对 Never，工具生成） |
| `tools/generate_collision_matrix.py` | 按 Setup Assistant 方法经 `/check_state_validity` 采样 2000 次生成碰撞矩阵 |
| `config/episode1_mock.srdf` | 占位模型（无碰撞几何） |
| `config/kinematics.yaml` | KDL 数值 IK（仅占位模型；真机用官方 IK） |
| `config/joint_limits.yaml` | 规划限速 0.25 rad/s、加速度 0.5 rad/s²（按执行端能力设定，真实值 TBD） |
| `config/moveit_controllers.yaml` | `episode_arm_controller` → `follow_joint_trajectory`；执行时长放宽 ×3 |
| `config/ompl_planning.yaml` | OMPL RRTConnect + 2.12 的请求/响应适配器 |
| `launch/moveit_mock.launch.py` | `stack:=ros2_control` 或 `stack:=python` |
| `test/moveit_smoke.py` | MoveGroup action 冒烟测试（9 项） |
| `test/moveit_debug_plan.py` | 只规划并打印轨迹终点的调试工具 |

两条执行栈对外都是 `/episode_arm_controller/follow_joint_trajectory`，所以一份 MoveIt 配置覆盖两者。

## 官方模型运行结果（Synthetic / Mock Data）

| 检查 | ros2_control + JTC | Python 安全层节点 |
|---|---|---|
| 目标 A / B / 回竖直：规划+执行成功且实测到位 | ✅ 误差 ≤ 0.011 rad | ✅ 误差 ≤ 0.010 rad |
| 越限目标：截断到边界、不越限 | ✅ | ✅ |
| 采样找到自碰撞姿态（link4 ↔ base_link） | ✅ | ✅ |
| 规划到自碰撞目标被拒绝（OMPL FAILURE） | ✅ | ✅ |
| **合计** | **11/11** | **11/11** |

碰撞矩阵：零位无自碰撞（无 Default 对）；2000 次采样中从不接触的 8 对禁用为 Never；base_link↔link4/5/6、link1↔link4/5/6、link2↔link4 共 7 对保持启用（存在真实自碰撞可能）。

## 占位模型运行结果（早期，Synthetic / Mock Data）

| 检查 | ros2_control + JTC | Python 安全层节点 |
|---|---|---|
| move_group 启动、action 可用 | ✅ | ✅ |
| 目标 A（J1 0.4、J5 −0.3）规划+执行 | ✅ 3.8 s，终点误差 0.010 rad | ✅ 3.3 s，0.006 rad |
| 目标 B（6 关节） | ✅ 8.7 s，0.007 rad | ✅ 6.7 s，0.010 rad |
| 回竖直姿态 | ✅ 8.7 s，0.011 rad | ✅ 6.2 s，0.010 rad |
| 越限目标（J2=2.5 rad）规划终点不越过限位 | ✅ 截断到 1.5708 | ✅ 截断到 1.5708 |

Python 栈“走走停停”、不遵守 `time_from_start`，在 `allowed_execution_duration_scaling: 3.0` 下没有被 MoveIt 判超时。

## 运行中发现的问题

1. **规划速度超过执行端能力 → 执行中止。** 初版限速 J5 0.8 rad/s，而 Mock 后端在 speed_ratio 0.3 下只有 ≈0.31 rad/s，JTC 报 `State tolerances failed for joint 5`（误差 0.309 > 0.3），MoveIt 收到 `PATH_TOLERANCE_VIOLATED`。把规划限速降到 0.25 rad/s 后通过。**真机接入前必须先实测关节速度，再设 MoveIt 限速。**
2. **MoveIt 不拒绝越限的关节目标，而是截断到边界**（`Assuming maximum bounds instead`）。规划不会越限，但调用者的意图被静默修改。本项目的 Python 节点在 goal 回调里**拒绝**越限轨迹——两层行为不同，上层应用必须自行校验。
3. MoveIt 2.12 的响应适配器名是 `DisplayMotionPath`，不是 `DisplayMotionPlans`（写错会导致 move_group 启动即崩）。

## 仍然存在的问题（真机阶段）

1. **时间参数化**：Python 执行器不遵守时间；ros2_control 路线对官方点到点 API 只能限流，快速轨迹效果未知（`16`）。
2. **限位一致性**：URDF 关节限位必须与电机物理范围映射一致；`08` 中官方 DH qlim 不一致，接入官方 URDF 前要逐一核对。
3. **起点一致性**：MoveIt 起点来自 `/joint_states`，本项目只发布实测。
4. **取消**：MoveIt 停止执行时发送 cancel；Python 节点把它变成物理停止（无软停时急停），急停后需重新回零。
5. **映射符号**（B）：已与官方 ROS 2 节点和 URDF 对齐，真机使用前仍需逐关节目视复核。
6. **碰撞检查**：官方 URDF 的碰撞几何直接复用可视 STL（偏保守、计算量较大）；SRDF 碰撞矩阵已由采样工具生成，未在真机上校核。官方 URDF 关节 `velocity="0"`，必须依赖 `joint_limits.yaml`。

## 运行方法

```bash
# 需要官方包 episode1_urdf_1113（官方 student_ros.zip）放进工作区 src/
ros2 launch episode_moveit_config moveit_mock.launch.py stack:=ros2_control model:=official   # 或 stack:=python
python3 moveit_config/test/moveit_smoke.py --stack ros2_control --collision-check
```

## 真机阶段的集成步骤

1. 只运行 `robot_state_publisher` + 本项目节点，RViz 中逐关节核对方向（`08` 流程）。
2. 实测每个关节的最大速度/加速度（低速逐级提高），据此设置 `joint_limits.yaml`。
3. 官方 URDF + 生成的碰撞矩阵已就绪；真机前用 RViz 目视检查碰撞模型与实物一致。
4. 真机：`speed_ratio_cap ≤ 0.2`，单段小幅规划，记录每段终点误差（A 级）。
