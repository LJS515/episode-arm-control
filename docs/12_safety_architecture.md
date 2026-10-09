# 12 安全架构

> 状态：软件安全层实现 + Mock 测试。**本项目不具备任何功能安全认证，也不能替代硬件急停。**

## 分层防护（从外到内）

| 层 | 措施 | 归属 | 可否被软件故障绕过 |
|---|---|---|---|
| 1 物理 | 硬件急停/电源开关、机械限位、围栏/空旷工作区 | 用户/官方 | 否 |
| 2 驱动器 | 闭环、堵转检测、驱动器保护（固件） | 官方 | 否（独立 MCU） |
| 3 官方上位机/核心 | IK 可达性、qlim、规划 | 官方 | 部分 |
| 4 本项目安全层 | 状态机、监督、仲裁、监视、故障锁存 | 本项目 | **是**（本项目进程崩溃即失效） |
| 5 调用方 | MoveIt 碰撞检查、应用逻辑 | 用户 | 是 |

第 4 层的价值是把“明显的错误”挡在总线之外，并把异常转为可见的、需要人确认的状态；它不是最后一道防线。

## 安全规则（实现位置）

| 规则 | 实现 |
|---|---|
| 未回零禁止运动 | `SafetySupervisor.check_motion_request` + 状态机 `ACTIVATE` 守卫 |
| FAULT/ESTOP 禁止普通运动 | 同上 + 仲裁器急停标志 |
| 关节软限位（物理范围内缩 2°） | `validate_joint_vector(margin)` |
| speed_ratio ∈ [0.01, 1]，调试期默认 0.3 | `validate_speed_ratio` + `speed_ratio_default_cap` |
| 反馈过期/不完整禁止运动 | `_require_fresh` |
| 运动中反馈过期 → 故障 | `evaluate()` |
| 连续读失败 → 故障 | `evaluate()` |
| 运动超时 / 终点误差 → 故障 | `TrajectoryMonitor` + `on_motion_result` |
| 故障时运动中 → 停止（不支持软停则急停） | `SafeRobotController._abort_on_fault` |
| 故障锁存，受控清除 | `latch_fault` / `clear_fault`（守卫：操作员确认 + 新鲜 + 0 失败）→ INACTIVE |
| 急停后须重新回零 | 状态机 `require_rehome_after_estop` |
| 急停不受锁/队列阻塞 | 绕过 `hardware_lock`；TCP 专用连接；仲裁器立即清空 |
| 运动命令不自动重试 | `retry.py` |
| 真机示例需双重确认 | `examples/_common.py`：`--host` + `--real-robot` + 键入 yes |
| 真机 launch 需确认参数 | `hardware.launch.py confirm_real_robot:=true` |

## 软件急停 vs 硬件急停

| | 软件急停（本项目 `emergency_stop`） | 硬件急停 |
|---|---|---|
| 路径 | Python → TCP → 上位机 → PCAN → CAN 仲裁 → 驱动器固件 | 切断电机电源 |
| 失效点 | 任一进程崩溃、网络、USB、CAN 错误/Bus Off、驱动器无响应 | 开关本身 |
| 停止方式 | 上位机“立即停止并忽略后续指令”直到恢复（B）；减速还是急刹 TBD；**对舵机夹爪无效**（B） | 断电；步进电机失去保持力矩，**重力轴可能下落** |
| 适用 | 程序可感知的异常 | 一切紧急情况 |

注意断电后 J2/J3 等重力轴可能下坠（D：取决于减速器自锁性，行星减速器一般不自锁）。真机首测时需评估这一点。

## 首次上电检查清单（真机）

1. 工作区清空，机械臂在官方上位机中正常回零，竖直姿态目视正确（FAQ tid=1996 图示）。
2. 硬件急停在手边，已演练一次断电。
3. 记录：驱动固件版本（应为 V5.1.5D）、上位机版本、USB2CAN 终端档位（120R）。
4. 上位机 TCP 端口只绑定 localhost（官方手册：设为 0.0.0.0 即对局域网开放，B）。
5. 先运行只读：`python examples/read_angles.py --host 127.0.0.1 --real-robot`，确认 6 个值无 None、与上位机显示一致。
6. `config/safety.yaml`：`speed_ratio_default_cap ≤ 0.3`，`max_joint_step_deg: 10`。
7. 单关节 +5° 小步运动，观察方向与映射（`08` 的符号验证流程）。
8. 测试软件急停：运动中执行 `examples/emergency_stop.py`，记录到静止时间（TBD）。
9. 测试拔线：断电状态下拔掉末端关节 CAN，上电后确认出现 None → FAULT。

## 残余风险（未解决）

* 驱动器内部故障（丢步、过热）对本层不可见（无对应接口，E）。
* J5 皮带打滑只能在终点误差检查中**间接**发现，且电机编码器读数本身可能“正确”——真正检测需要输出侧传感器或视觉。
* 碰撞检测不存在（没有电流/力矩反馈接口）。
