# ROS2 Jazzy runtime verification (WSL2)

> **Synthetic / Mock Data.** Every run below used the Mock backend or the Python
> TCP Mock upper computer. No robot, no PCAN, no CAN. Date: 2026-10-08.

## Environment

| Item | Value |
|---|---|
| Host | Windows 11 + WSL2, kernel 6.18.40.1-microsoft-standard-WSL2 |
| Distro | Ubuntu 24.04.4 LTS |
| ROS | ROS2 Jazzy (`ros-jazzy-desktop`), ros2_control / controller_manager 4.48.0, joint_trajectory_controller 4.42.1 |
| Compiler | g++ 13.3.0, flags `-Wall -Wextra -Wpedantic` |
| Python | 3.12.3, venv with `--system-site-packages`, project installed with `pip install -e .` |
| Build | `python -m colcon build --symlink-install` (inside the venv, see "Deployment fix") |
| MoveIt | 2.12.4 (`ros-jazzy-moveit`, installed during this verification) |

## 1. Python test suite on Linux

`python -m pytest -q` → **221 passed**, 30 consecutive runs without any failure or warning (after the two fixes in section 8; the first Linux runs showed an intermittent EBADF thread error ~1 in 7).

## 2. Build

| Package | Result |
|---|---|
| `episode_ros2` (ament_python) | built |
| `episode_ros2_control` (ament_cmake) | built, **0 compiler warnings** |

## 3. C++ unit tests (`colcon test`, gmock)

| Test | Result |
|---|---|
| rejects_wrong_joint_count | ✅ |
| rejects_bad_sign | ✅ |
| lifecycle_read_write_tracks_command (init → export → configure → activate → read/write, joint1 → 0.2 rad) | ✅ |
| write_outside_limits_is_error | ✅ |
| nan_command_is_ignored | ✅ |

## 4. ros2_control runtime — mock backend

`ros2 launch episode_ros2_control mock_ros2_control.launch.py`

| Check | Result |
|---|---|
| Hardware component `Episode1System` | active, 50 Hz read/write |
| Controllers | `joint_state_broadcaster` active, `episode_arm_controller` (JTC) active |
| `/joint_states` initial | all 0.0 (vertical posture = model zero) |
| JTC goal joint1 → 0.3 rad, joint5 → −0.2 rad in 3 s | **SUCCEEDED**, error_code 0; final 0.300 / −0.200 |
| `/joint_states` rate | 50.0 Hz |
| Warnings | only "Could not enable FIFO RT scheduling" (expected in WSL without RT privileges) |

## 5. ros2_control runtime — `vendor_tcp` backend → Python TCP Mock upper computer

Exercises the C++ length-prefixed JSON client (`VendorTcpBackend`) end to end.

| Check | Result |
|---|---|
| Probe + feedback via `get_motor_angles` over TCP | OK, `/joint_states` 50 Hz |
| JTC goal joint1 → 0.3 rad in 4 s (rate-limited `angle_mode` forwarding) | **SUCCEEDED**, final 0.30000 rad |
| Mock server killed during operation | `read()` → ERROR (stale feedback); controller_manager deactivated the hardware component and both controllers; component state → unconfigured |

Note: the slow 3–4 s trajectories succeed with the rate-limited forwarding. This
does **not** show that fast trajectories track well against the real upper
computer — see `docs/16_ros2_control_design.md`.

## 6. episode_ros2 runtime — `mock.launch.py` + `episode_ros2/test/ros2_runtime_smoke.py`

Nodes: `/episode_mock_hardware` (TCP Mock upper computer), `/episode_robot_interface`
(backend `vendor_tcp`), `/episode_diagnostics`. **17/17 checks passed**:

| # | Check | Observed |
|---|---|---|
| 1 | `/episode/robot_state` READY (latched) | READY |
| 2 | `/joint_states` rate ≥ 10 Hz | 20–23 Hz |
| 3 | initial positions | all 0 |
| 4 | FollowJointTrajectory goal accepted | ✅ |
| 5 | goal SUCCEEDED | status 4, error_code 0 |
| 6 | feedback contains desired/actual/error | 8 feedback messages |
| 7 | position reached | 0.1500 rad |
| 8 | out-of-limit goal (10 rad) rejected before acceptance | ✅ |
| 9 | cancel during motion → CANCELED | status 5 |
| 10 | cancel → ESTOP (vendor_tcp has no soft stop) | ESTOP |
| 11 | arm stopped short of target | stopped at ≈ 0.37 rad of 1.2 rad, no further motion |
| 12 | `~/release_estop(data=true)` → INACTIVE | ✅ |
| 13 | `~/declare_homed` → READY | ✅ |
| 14 | mock stops replying → FAULT | FAULT (READ_FAILURES) |
| 15 | `/joint_states` stops while stale | +0 messages in 1 s |
| 16 | `/diagnostics` watchdog STALE + state FAULT | "/joint_states silent for 2.23s", robot_state FAULT |
| 17 | replies restored → `~/clear_fault` → INACTIVE | ✅ |

Launch log: no tracebacks or errors.

## Deployment fix found during this run

The first launch failed: `ModuleNotFoundError: No module named 'episode_sdk'`.
colcon was started from `/usr/bin` and wrote entry-point shebangs pointing to the
system Python, which cannot see the venv's editable install. Fix: run colcon with
the venv interpreter (`python -m colcon build ...`), which produces
`#!/home/<user>/episode_ws/venv/bin/python` shebangs. README and
`episode_ros2/README.md` were corrected.

## API port found during this run

The installed ros2_control 4.48 marks `on_init(const HardwareInfo&)` and the
vector-returning `export_*_interfaces()` as deprecated. The SystemInterface was
ported to `on_init(const HardwareComponentInterfaceParams&)`, framework-created
interfaces and `set_state()/get_command()` before building, so it compiles
without deprecation warnings.

## 7. MoveIt 2.12.4 — plan + execute on both stacks

`ros2 launch episode_moveit_config moveit_mock.launch.py stack:=<stack>` +
`moveit_config/test/moveit_smoke.py`. Placeholder model (stick URDF, no collision
geometry), MoveIt planning limits 0.25 rad/s.

| Check | stack=ros2_control (JTC) | stack=python (safety-layer node) |
|---|---|---|
| move_group up, `/move_action` available | ✅ | ✅ |
| goal A (J1 0.4, J5 −0.3): SUCCESS + reached | ✅ 3.8 s, err 0.010 rad | ✅ 3.3 s, err 0.006 rad |
| goal B (6 joints): SUCCESS + reached | ✅ 8.7 s, err 0.007 rad | ✅ 6.7 s, err 0.010 rad |
| back to vertical: SUCCESS + reached | ✅ 8.7 s, err 0.011 rad | ✅ 6.2 s, err 0.010 rad |
| out-of-limit target (J2 = 2.5 rad): plan stays within limit | ✅ clamped to 1.5708 | ✅ clamped to 1.5708 |
| **Total** | **9/9** | **9/9** |

Remaining log lines are benign: "No 3D sensor plugin(s) defined for octomap updates"
(no perception configured) and "Found empty JointState message" (plan-only requests).

Findings during the first MoveIt runs:

1. Response adapter name: MoveIt 2.12 uses `DisplayMotionPath`; `DisplayMotionPlans`
   crashed move_group at start.
2. **Planned velocity above backend capability → execution aborted.** With
   max_velocity 0.8 rad/s for joint5, JTC aborted ("State tolerances failed for
   joint 5: Position Error −0.309, Tolerance 0.300"); the mock backend at
   speed_ratio 0.3 moves ≈0.31 rad/s. Limits lowered to 0.25 rad/s → pass.
3. **MoveIt clamps out-of-bounds joint goals** to the URDF bound
   ("Assuming maximum bounds instead") instead of rejecting them. The smoke test
   was changed to assert that the plan never exceeds the limit.

## 7b. MoveIt with the OFFICIAL URDF (student_ros V1.0, read-only vendor package)

`model:=official` → `episode_official.urdf.xacro` (official URDF + our ros2_control
block) and `episode1_official.srdf`. Collision matrix generated with
`tools/generate_collision_matrix.py` (2000 samples, seed 1): no pair in contact at
the zero pose; 8 pairs never in contact → `Never`; 7 pairs kept enabled.

| Check | stack=ros2_control | stack=python |
|---|---|---|
| goals A / B / vertical: SUCCESS + reached | ✅ err ≤ 0.011 rad | ✅ err ≤ 0.010 rad |
| out-of-limit target clamped, never beyond | ✅ | ✅ |
| self-colliding state found (link4 ↔ base_link) | ✅ | ✅ |
| plan to the self-colliding target refused (OMPL FAILURE) | ✅ | ✅ |
| **Total** | **11/11** | **11/11** |

The joint signs used by both stacks were corrected to the official convention
(all +1) before this run.

## 8. Intermittent defects found by repeated runs (fixed)

| Defect | Frequency | Root cause | Fix |
|---|---|---|---|
| Mock server thread `OSError: [Errno 9] Bad file descriptor` → test error | ~1/7 full runs (Linux) | `recv_exact/send_all` let non-reset `OSError` escape as raw exceptions; mock closed sockets under a blocked `recv()` | all socket `OSError` → `ConnectionClosedError`; mock does `shutdown()` before `close()`; 2 regression tests |
| `ReceiveTimeoutError` instead of `CommandTimeoutError` | ~1/20 runs (Windows) | socket timeout fires a few ms before the absolute deadline, then `deadline.expired()` is still false | classify by which deadline was binding (`recv_deadline is deadline`) |

After the fixes: 30 consecutive full runs on Linux and 30 on Windows, 221 passed each, no failures or warnings.

## Still not verified

Official upper computer and the official ROS2 node / compiled core (only read, not
run), real hardware, real-time behaviour (WSL has no RT scheduling).
