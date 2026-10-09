# episode_ros2 — ROS2 front end for the Episode 1 safety layer

> **Status:** built with colcon and run under **ROS2 Jazzy on WSL2 Ubuntu 24.04**
> against the TCP Mock upper computer: `test/ros2_runtime_smoke.py` passes **17/17**
> (READY, joint_states rate, action success + feedback, limit rejection, cancel →
> physical stop, ESTOP release, feedback loss → FAULT + diagnostics STALE, clear fault).
> **Not run on the real robot, not run against the official upper computer.**
> Details: `../reports/ros2_runtime_wsl.md`.

## What is in here

| Node | Purpose |
|---|---|
| `robot_interface_node` | `SafeRobotController` behind ROS2: `/joint_states`, `FollowJointTrajectory` action, Trigger/SetBool services, `/diagnostics` |
| `mock_hardware_node` | Runs the TCP Mock of the official upper computer inside ROS2, with fault-injection services |
| `diagnostics_node` | Independent watchdog for `/joint_states` freshness, rate and plausibility |

This is **not** the official `episode_controller` package. The official package
talks to CAN through the compiled `episode_controller_core`; this package talks
to the official *Windows upper computer* over TCP/JSON (route A) and adds the
safety layer in front of it. See `../docs/05_control_paths.md`.

## Interfaces

| Name | Type | Semantics |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState`, reliable, depth 10 | **measured** positions only, model radians; not published while stale/partial; stamp = sample time |
| `/episode/motor_angles` | `std_msgs/Float64MultiArray` | raw motor degrees (official coordinate) |
| `/episode/robot_state` | `std_msgs/String`, reliable + transient_local | `UNCONFIGURED/INACTIVE/HOMING/READY/MOVING/STOPPING/FAULT/ESTOP` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | feedback freshness, state, arbiter |
| `/episode/estop` | `std_msgs/Empty` (sub) | software E-stop (own callback group) |
| `episode_arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` action | executes waypoints as point-to-point moves; feedback has desired/actual/error; cancel = physical stop |
| `~/emergency_stop`, `~/home`, `~/stop`, `~/clear_fault`, `~/declare_homed` | `std_srvs/Trigger` | short transactions only |
| `~/release_estop` (`data`=hardware reset confirmed), `~/gripper`, `~/set_free_mode` | `std_srvs/SetBool` | short transactions only |

### Why Action for motion, Service only for short transactions
A move lasts seconds, needs progress feedback and must be cancellable; a service
call blocks the client and has no cancel. `~/home` therefore only *starts*
homing — progress is on `/episode/robot_state`.

### Executor and callback groups
`MultiThreadedExecutor(4)` with four groups: `poll_group` (ME, timers),
`action_group` (Reentrant), `srv_group` (ME), `estop_group` (ME, separate so an
E-stop never waits behind another service). Parallel callbacks do **not** make
hardware access safe: every bus transaction goes through
`SafeRobotController.hardware_lock`, and the `CommandArbiter` guarantees a single
motion owner. E-stop bypasses the lock.

### Known limitations
* Trajectory timing (`time_from_start`) is not honoured: the official API is
  point-to-point, so waypoints (≥ `waypoint_min_spacing_deg` apart) run as
  stop-and-go segments. Streaming via `dynamic_move` is evidence E (format unknown).
* The `ros_model` joint signs match the official ROS 2 node and URDF
  (evidence B) but still require a visual per-joint check before hardware use.
* Official custom actions (`AngleMode`, `MoveXyzRotation`, `LinearMove`) are not
  re-exposed; see `interfaces_proposal/`.

## Build & run (Ubuntu 24.04 + ROS2 Jazzy) — executed on WSL2

```bash
source /opt/ros/jazzy/setup.bash
# 1) make the pure-Python layer importable for ROS (venv that sees system ROS packages)
python3 -m venv --system-site-packages ~/episode_ws/venv && source ~/episode_ws/venv/bin/activate
pip install -e /path/to/episode-arm-control
# 2) build — run colcon WITH THE VENV PYTHON so entry-point shebangs point to the venv
#    (plain `colcon build` writes #!/usr/bin/python3 -> ModuleNotFoundError: episode_sdk)
mkdir -p ~/episode_ws/src && ln -s /path/to/episode-arm-control/episode_ros2 ~/episode_ws/src/
cd ~/episode_ws && python -m colcon build --symlink-install && source install/setup.bash
# 3) mock bring-up (Synthetic / Mock Data)
ros2 launch episode_ros2 mock.launch.py
python3 /path/to/episode-arm-control/episode_ros2/test/ros2_runtime_smoke.py   # second terminal
ros2 topic echo /episode/robot_state
ros2 action send_goal /episode_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1,joint2,joint3,joint4,joint5,joint6], points: [{positions: [0.1,0,0,0,0,0], time_from_start: {sec: 2}}]}}" --feedback
ros2 service call /episode_mock_hardware/drop_replies std_srvs/srv/SetBool "{data: true}"   # -> FAULT
```

Expected checks: state goes `INACTIVE → READY` (assume_vendor_homed in mock),
action succeeds with feedback, `drop_replies` leads to `FAULT` + `/diagnostics`
STALE/ERROR, `~/clear_fault` returns to `INACTIVE` once replies resume.
