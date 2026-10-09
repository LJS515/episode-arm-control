# Official material review — logged-in resource centre (evidence B)

> Static analysis, 2026-10-08. Accessed with the project owner's forum login
> (owner consented; no credentials were handled by the assistant). Downloads
> were made with the owner's explicit permission into an isolated scratch folder
> and only **read** — nothing from them was executed or installed, and none of
> the vendor files are redistributed in this repository.

## Sources

| ID | Source | Version / date |
|---|---|---|
| R1 | Forum tid=1217 "Episode1 资料中心（驱动、上位机、机械臂参数、API开发）" | posted 2025-03-13, read 2026-10-08 |
| R2 | Forum tid=1191 "开箱安装与上位机使用手册" | 2025-03-11 |
| R3 | `sdk_demo.py` — 二次开发 SDK Demo (Python) | V1.2.1, 17 828 B |
| R4 | `cpp_api_07101.zip` — SDK Demo (C++) | V1.1, 201 828 B |
| R5 | `student_ros.zip` — URDF / ROS2 Ubuntu package | V1.0, 22 226 484 B, sha256 `bbc42d6e…5d4e` |

Download list in R1 also names: USB2CAN driver, calibration tool V0.9.4, upper
computer V0.9.9.5.1 (Windows/macOS/Ubuntu/Jetson Xavier/RK3588), RoboDK driver
V1.1 / model / examples.

## 1. API reference (R1) — corrections applied to the SDK

| Item | Previous assumption | Official (B) | SDK change |
|---|---|---|---|
| `emergency_stop` | params `null` (E) | `1` stop / `0` release; returns 0.05 | `emergency_stop()` sends 1; new `release_emergency_stop()` sends 0; vendor adapter can release in software |
| `angle_mode` reply | duration or -1 | `0` no motion needed / `-1` no solution / `>0` duration; speed auto-clamped 0..1 | already handled |
| `move_xyz_rotation` | 7 params | **8 params** `[x,y,z,r1,r2,r3,order,speed_ratio]` | sends 8 by default (`speed_ratio=None` → historical 7-param form) |
| `move_linear_xyz_rotation` | 7 params | 7 params, speed not adjustable, replies first then executes | unchanged |
| Euler orders | 6 orders | only `"zyx"` and `"xyz"` | validation restricted |
| `servo_gripper` | 0–180 (E) | integer 0–110, server clamps; larger = closes; 3 s after power-up resets to 90° | validation 0–110, int |
| `set_free_mode` | may not reply (C) | returns 0.1 | mock replies 0.1; tolerance for old servers kept |
| reads (`get_motor_angles`/`get_T`/`get_pose`) | per-joint None | **whole reply None** "due to CAN bus blocking, e.g. while other control commands run" | new `ReadFailedError` (retryable) |
| `get_pose` | params null | params = order `"zyx"`/`"xyz"` | `get_pose(order)` |
| client guidance | — | "wait the returned time before sending the next command" | completion still judged from feedback |
| parameterless commands | sent `"params": null` | official SDK omits `params` | legacy envelope omits it |

The CAN-blocking note confirms the project's premise (evidence D → B) that
concurrent commands and reads interfere on the shared bus.

## 2. Robot parameters (R1, R2)

* DH table equals the historical client's (C → B). **DH θ ranges** J1 [-180,160], J2 [0,180], J3 [-80,83], J4 [-30,305], J5 [-110,110], J6 [-30,305] equal the motor ranges mapped through the DH mapping → the narrower qlim found earlier exists **only in the historical client**.
* Vendor-stated: payload 500 g, reach 510 mm, **repeatability ±0.03 mm "non-national-standard test method, for reference only"**, weight 8.3 kg, 12 V/10 A, 0–45 °C, < 55 dB, software limits + software E-stop. These are vendor claims, not project measurements.
* Reducers (R2): J1 2-stage 25:1, J2 20:1, J3 25:1, J4 1-stage 10:1, **J5/J6 no reducer** (J5 timing belt 4:1 per BOM/SDK gear list). Backlash notes: J2/J3 backlash compensated by gravity in normal postures; J1 has backlash in one direction.
* Homing (R2): mechanical stop + encoder, ~1 min after every power-up; "recovery mode" skips homing.
* E-stop (R2): stops the arm and **ignores further commands** until "resume service"; **does not stop the servo gripper**.
* Upper computer polls joint angles every 5 s; IP `0.0.0.0` exposes the API to the LAN (needs firewall rule).

## 3. Official Python SDK V1.2.1 (R3)

* JSON over the 8-byte big-endian frame — confirms the wire format.
* **Still reads the length header with a single `recv(8)`** → the short-read issue remains in the current official SDK.
* No socket timeouts; every exception is caught and turned into `None`, so a network error is indistinguishable from the server's "read failed" `None`.
* One TCP connection per command.
* `dynamic_move` (missing from the API page): params `[{"1":q1,…,"6":q6}, current_joint_pulses[6], motors_max_speed[6]]`, `pulses = int(3200 * gear * angle / 360)` with gear `[25,20,25,10,4,1]`, reply 0.05; demo streams at 30 Hz with `motors_max_speed = [90,40,100,100,100,10] * 0.5`; prerequisites: **driver-board "Response" = None and upper-computer status refresh off**. SDK `dynamic_move()` now uses this format.

## 4. Official C++ SDK V1.1 (R4)

nlohmann/json; `send_all`/`recv_all` loop correctly; `htonll` 8-byte header;
no timeouts and **no upper bound on the response length** (`resize(response_length)`).

## 5. Official ROS2 package V1.0 (R5)

Contents: `episode_controller` (Python node), `robot_arm_interfaces`
(AngleMode/LinearMove/MoveXyzRotation actions, GripperControl/ReadMotorAngles/
ServoGripper/SetFreeMode services, MotorAngles msg), `episode1_urdf_1113`
(URDF, SDF, visual+collision STL, Gazebo world/bridge), and
`episode_controller_core` wheels (cp38–cp312, x86_64 + aarch64) containing
compiled `controller_core`, `PCANBasic` and `ik_wrapper` extension modules.
README: Ubuntu 24.04 + Jazzy; "no upper computer needed" (route B).

### Findings in `robot_interface_node.py` (previous evidence → now B)

| # | Finding | Earlier status in this project |
|---|---|---|
| 1 | `/joint_states` = `motor - offset` with offsets `[180,90,83,30,110,30]`, **all signs +1** | ROS signs were E and guessed `[+,-,-,+,-,+]` — **wrong for J2/J3/J5; corrected** |
| 2 | `AngleMode`/`MoveXyzRotation` feedback is **"模拟进度反馈"** (time-based); success reported after sleeping the estimate; `final_position = position  # 假设达到目标位置` | D (docs/09 Q2) |
| 3 | Cancel returns `canceled` without stopping the motors; linear move: "运动已开始，无法完全停止" | D (docs/09 Q3) |
| 4 | `FollowJointTrajectory` feedback sets `feedback.actual.positions = point.positions` (planned point reported as actual); no joint-name / limit validation; always reports success | D (docs/09 Q1, docs/15) |
| 5 | Comment: timer must not use the reentrant group "否则会导致CAN堵塞"; actions do use a `ReentrantCallbackGroup` and call the core in parallel with the joint-state timer | D (docs/10) |
| 6 | `read_motor_angles` service returns the cached angles **without age check**, `success=True` | D (cache freshness) |
| 7 | `joint_states` stamp = publish time; requested 120 Hz of blocking CAN reads | — |
| 8 | MoveIt trajectories executed by streaming interpolated points (factor 15) through `dynamic_move_with_velocity` | E (streaming capability) → B |
| 9 | Homing: per-joint `trigger_home` + `check_home_status`, timeout 120 s per joint, then move to `[180,90,83,30,110,30]`; parameter `init_mode` default 1 = recover (README says `init_mode:=0` to home) | — |
| 10 | No emergency-stop service/topic in the ROS API | — |
| 11 | `setup.py` declares `moveit_bridge = episode_controller.moveit_trajectory_bridge:main`, but that module is not in the package (broken entry point) | — |

### Official URDF `episode1_urdf_1113.urdf`

Joints `joint1..joint6`, root `base_link`. Limits (deg): J1 [-180,160], J2 [-90,90],
**J3 [-83,80]**, J4 [-30,305], J5 [-110,110], J6 [-30,305] — equal the motor
ranges shifted by the offsets **with sign +1** (independent confirmation of
finding 1). `velocity="0" effort="0"` on every joint (MoveIt needs a
joint_limits.yaml); collision geometry reuses the visual STLs.

## Consequences for this project

* Mapping, SDK and mock updated (see CHANGELOG 0.4.0); EVIDENCE_MATRIX upgraded.
* The official ROS2 node demonstrates exactly the gaps this project's safety layer
  addresses (simulated progress, cancel without stop, planned-as-actual
  feedback, unsynchronised CAN access, stale cache) — now evidenced from the
  official source rather than inferred.
* Route B streaming exists (`dynamic_move_with_velocity`), which makes a real
  streaming ros2_control backend plausible; its API is in a compiled module and
  was not executed.
