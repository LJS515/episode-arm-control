# Vendor material review (static analysis)

> Status: **static analysis of public material**, performed 2026-10-08.
> No hardware involved. Evidence levels per `EVIDENCE_MATRIX.md`.

## Sources

| ID | Source | Level | Accessed |
|---|---|---|---|
| S1 | Project owner's description of the current official system (two control routes, API list, ROS2 package contents, joint ranges, ROS offsets, 12 V/10 A, 500 g, 510 mm) | B | 2026-10-08 |
| S2 | Public repo `github.com/enpeizhao/episode_robot`, commit `9ede794` (2025-02-20): `README.md`, `episodeServer.py`, `0.server.py`, demos | C | 2026-10-08 |
| S3 | Official forum, board "Episode1 机械臂硬件常见问题交流" (fid=57), public threads tid=1932, 1953, 1996, 1997, 1943, 1260 and the topic list | B | 2026-10-08 |
| S4 | Official forum resource board (fid=56) — accessed later with the owner's login; see `official_material_review.md` | B | 2026-10-08 |

## Findings from S2 (historical public source, evidence C)

| # | Finding | Consequence for this project |
|---|---|---|
| C1 | Framing: 8-byte big-endian length prefix, both directions | same framing in our SDK (S1 confirms for the current version) |
| C2 | Payload was **`pickle`**, not JSON; server does `pickle.loads()` on network data | remote code execution risk if an old server is reachable → SECURITY.md; our SDK is JSON-only |
| C3 | Server reads the header with a single `recv(8)` (no loop) | short-read bug; motivates `recv_exact` |
| C4 | Server body loop `break`s on EOF and then unpickles a partial buffer | truncated frames are not detected server-side |
| C5 | One thread per client connection; no lock around motor commands | two clients can interleave commands on one CAN bus (D) → CommandArbiter + hardware_lock |
| C6 | `move_linear_xyz_rotation` replies with the *planned total time* and then blocks the connection thread while executing | reply ≠ completion → TrajectoryMonitor; cancel cannot be delivered on the same connection |
| C7 | Unknown action → no reply at all (`result is None`) | client needs a receive timeout; legacy mock reproduces it |
| C8 | `home` reply is a hard-coded `5` ("假设归位操作需要5秒") | do not trust it as a duration |
| C9 | Client opens a new TCP connection per command and `time.sleep(result)` after moves | open-loop completion; our default connection policy mirrors per-command for compatibility |
| C10 | `set_free_mode(1)` client waits 10 s with a warning before enabling | free mode is a safety-relevant transition |
| C11 | After homing, server commands motor angles `[180, 90, 83, 30, 110, 30]` ("回到垂直模式") | equals the ROS offsets (S1) → vertical posture = URDF zero (D) |
| C12 | Gear ratios `[25, 20, 25, 10, 4, 1]` | J5 ratio 4 ↔ 15T/60T belt in the BOM; J6 direct |
| C13 | DH table d/a/alpha: (166,55,90°) (0,200,0) (0,56,90°) (192,0,−90°) (0,0,90°) (55,0,0) | reference FK/IK in `kinematics.py` |
| C14 | Motor→DH mapping: q1=m1−180, q2=180−m2, q3=83−m3, q4=m4−30, q5=110−m5, q6=m6−30 | `VENDOR_DH` profile; tested against the literal formulas |
| C15 | DH qlim J1 [−180,140], J4 [−30,270], J6 [−30,290] vs motor ranges mapped [−180,160], [−30,305], [−30,305] | inconsistency flagged; review on hardware |
| C16 | BOM: drivers from 张大头 (ZDT) "CAN 套餐", USB2CAN converter, magnets on motor shafts for the driver's angle sensor | encoder is motor-side; output-side errors (belt slip) are invisible to it |

## Findings from S3 (official FAQ, evidence B)

| # | Finding (paraphrased) | Consequence |
|---|---|---|
| B1 | Angle `None` in the upper computer = no CAN communication with that driver; chain: PCAN driver → USB2CAN → cable → driver board (tid=1932) | SDK keeps per-joint `None`; cache marks the sample invalid; motion blocked |
| B2 | Checks: USB2CAN switch on 120 Ω termination, CAN_H/CAN_L wiring, `ID_Addr` unique, `P_Serial = CAN1_MAP`; base CAN wire break kills all downstream joints; bisect by unplugging the 4P connector (tid=1932) | CAN investigation checklist (docs/07) |
| B3 | Linux: `pcaninfo` must list the device; kernel changes require driver reinstall; Secure Boot may block the unsigned driver (tid=1932) | ROS2 route prerequisites (docs/03) |
| B4 | After a driver reset: `ID_Addr` = joint index, `P_Serial = CAN1_MAP`, `Clog_Ma = 3000`, `Clog_Ms = 6000`; "碰撞回零检测电流" 1500/1500/1500/400/400/1000 for joints 1–6 (tid=1953) | homing is **stall-current based** (collision homing) |
| B5 | J5 wrong angle: belt/pulley slip, or homing stopped early due to friction (threshold current reached) → reference zero shifts (tid=1996) | motivates final-error check and FINAL_ERROR_EXCEEDED fault; FK cannot detect it |
| B6 | Gripper chain: upper computer → 4P expansion port → CAN transceiver → STM32; C14 drives relay (pump/valve), A1 PWM drives servo, 7.4 V buck (tid=1997) | gripper is a separate CAN node with its own firmware |
| B7 | Driver firmware must be **V5.1.5D** (tid=1943) | record firmware version with every measurement |
| B8 | Calibration tool homing timeout 30 s (V0.9) → 60 s in later versions (tid=1260) | `homing_timeout_s = 60` |

## Explicitly NOT found (stay TBD / E)

CAN bit rate, CAN identifiers, driver frame layout, control-loop rates, current
official server source, and any project measurement of repeatability or latency.
(API parameter formats and the official ROS2 interface definitions were later
obtained — see `official_material_review.md`.)
