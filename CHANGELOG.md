# Changelog

All notable changes. Dates are development dates; "Mock" = verified only in the
Mock environment; nothing below has been run on the real robot yet.

## [0.4.0] — 2026-10-08 — aligned with the official resource centre (evidence B)

Sources: forum tid=1217 / tid=1191, official Python SDK V1.2.1, C++ SDK V1.1,
ROS2 package student_ros V1.0 (read only, not redistributed). Details:
`reports/official_material_review.md`.

### Fixed (were evidence E, now B)
- **ROS model signs are all +1** (`joint = motor − offset`, official node; official
  URDF limits agree). The earlier guess [+,−,−,+,−,+] was wrong for J2/J3/J5 —
  corrected in Python mapping, ROS configs, ROS node, C++ defaults, xacro,
  placeholder URDF (J3 limit [-83, 80]). Two regression tests added.
- `emergency_stop` sends `1`; new `release_emergency_stop()` sends `0`; the vendor
  adapter can release the E-stop in software.
- `move_xyz_rotation` sends 8 params incl. `speed_ratio`; Euler orders limited to
  `zyx`/`xyz`; `servo_gripper` integer 0..110; `get_pose(order)`.
- Whole-reply `None` from reads → new retryable `ReadFailedError` (retry actually
  applied — it was raised outside the retry loop at first).
- Legacy envelope omits `params` for parameterless commands, like the official SDK.
- `dynamic_move` uses the official format `[{"1":q…}, pulses, max_speed]`.
- Mock server models the current official behaviour (estop 1/0, set_free_mode 0.1,
  null read replies, servo clamp, 8-param pose move).

### Added
- `ros2_control/urdf/episode_official.urdf.xacro` (official URDF + our ros2_control block),
  `moveit_config/config/episode1_official.srdf` with a generated collision matrix,
  `moveit_config/tools/generate_collision_matrix.py`, launch argument `model:=official|placeholder`.
- `OFFICIAL_DH_THETA_RANGE_DEG`, `OFFICIAL_URDF_LIMITS_DEG` constants and checks.
- `reports/official_material_review.md`.

## [0.3.0] — 2026-10-08 — MoveIt + flakiness fixes

### Added
- `moveit_config/` (`episode_moveit_config`): SRDF, kinematics, planning limits,
  controller mapping, OMPL config, `moveit_mock.launch.py` (`stack:=ros2_control|python`),
  `test/moveit_smoke.py`, `test/moveit_debug_plan.py`.
- 2 regression tests for sockets closed by another thread.

### Fixed
- `recv_exact`/`send_all`: every socket `OSError` (EBADF, ENOTCONN, …) now maps to
  `ConnectionClosedError`; previously raw `OSError` could escape the SDK (found as a
  ~1/7 intermittent test error on Linux).
- Mock server closes client sockets with `shutdown()` before `close()`.
- Timeout classification: `CommandTimeoutError` decided by which deadline was binding,
  not by re-reading the clock (~1/20 intermittent mismatch on Windows).
- MoveIt planning limits lowered to the execution backend's capability (JTC aborted
  with 0.8 rad/s); response adapter name `DisplayMotionPath` for MoveIt 2.12.

### Verified (Mock only)
- MoveIt 2.12.4 plan + execute: 9/9 on the ros2_control stack, 9/9 on the python stack.
- Python suite 221/221, 30 consecutive runs each on Windows and Linux.

## [0.2.0] — 2026-10-08 — ROS2 Jazzy verification (WSL2, Mock)

### Changed
- `episode_ros2_control` ported to the ros2_control 4.48 API
  (`on_init(HardwareComponentInterfaceParams)`, framework-created interfaces,
  `set_state()/get_command()`); removed deprecated `export_*_interfaces()`.
- CMake: dropped the unavailable `ros2_control_test_assets` test dependency.
- Build instructions: colcon must be run with the venv interpreter
  (`python -m colcon build`); plain `colcon build` produced entry points using the
  system Python → `ModuleNotFoundError: episode_sdk`.

### Added
- `ros2_control/test/test_episode_system_hardware.cpp` (gmock, 5 cases).
- `ros2_control/urdf/episode_mock.urdf.xacro` (placeholder model),
  `ros2_control/launch/mock_ros2_control.launch.py`, `ros2_control/config/controllers.yaml`.
- `episode_ros2/test/ros2_runtime_smoke.py` (17 runtime checks).
- `reports/ros2_runtime_wsl.md`.

### Verified (Mock only)
- Python suite 219/219 on Linux; colcon build 0 warnings; gmock 5/5;
  controller_manager + JTC trajectories with mock and TCP-mock backends;
  episode_ros2 smoke test 17/17.

## [0.1.0] — 2026-10-08

### Added — SDK (`episode_sdk`)
- Length-prefixed JSON framing with `recv_exact`, deadlines, 1 MiB frame limit,
  NaN/Inf rejection, UTF-8 validation.
- `EpisodeClient` with legacy-compatible (`{"action","params"}`) and enhanced
  (request id, protocol version, structured errors) modes; per-command or
  persistent connections; dedicated-connection E-stop; idempotent-only retries.
- Full API: `emergency_stop`, `home`, `angle_mode`, `move_xyz_rotation`,
  `move_linear_xyz_rotation`, `dynamic_move` (format E), `get_motor_angles`,
  `get_transform`, `get_pose`, gripper / servo / free mode / RoboDK.
- `MockEpisodeServer` mirroring historical server behaviour, with fault injection.

### Added — control layer (`episode_hardware`)
- Joint mapping (vendor DH = C, ROS model offsets = B / signs = E), reference
  FK + multi-start DLS IK (pure Python).
- State cache with timestamps, staleness, failure counters.
- 8-state safety state machine, command arbiter (reject/queue/preempt, E-stop
  override), safety supervisor with latched faults, trajectory monitor.
- `SafeRobotController` composition with a single hardware lock.

### Added — ROS2 / ros2_control (static only in 0.1.0; run in 0.2.0)
- `episode_ros2` (interface, mock hardware, diagnostics nodes; launch; config).
- `episode_ros2_control` SystemInterface (built and tested in 0.2.0).

### Added — diagnostic tools, engineering documentation and validation reports.

### Fixed during development (found by tests)
- Reference IK accepted 180°-flipped tool orientations (cross-product error
  metric is blind at 180°) → switched to SO(3) log-map error + multi-start.
- Arbiter did not set `cancel_requested` on preempted tickets.
- Mock server raised in a background thread when a client disconnected first.
- Connection-refused test assumed Linux behaviour; Windows retries SYN ≈ 2 s.
