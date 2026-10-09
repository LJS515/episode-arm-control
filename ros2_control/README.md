# episode_ros2_control — SystemInterface for the Episode 1 arm

> **Status: built and tested in ROS2 Jazzy on WSL2 Ubuntu 24.04 (ros2_control
> 4.48, g++ 13.3) with the mock backend and against the Python TCP Mock upper
> computer.** 0 compiler warnings with `-Wall -Wextra -Wpedantic`; gmock 5/5;
> controller_manager + JTC trajectories succeed. **Not run on hardware or against
> the official upper computer.** Details: `../reports/ros2_runtime_wsl.md`.

## Files

| File | Content |
|---|---|
| `episode_system_hardware.hpp/.cpp` | `EpisodeSystemHardware : hardware_interface::SystemInterface` + `MockBackend` + `VendorTcpBackend` |
| `episode_system_hardware.xml` | pluginlib description |
| `CMakeLists.txt`, `package.xml` | ament_cmake package `episode_ros2_control` |
| `episode_ros2_control.xacro` | `<ros2_control>` block with parameters (include from the official URDF) |
| `urdf/episode_mock.urdf.xacro` | **placeholder** stick-model URDF for mock runs (not the official URDF) |
| `config/controllers.yaml` | controller_manager + JTC + JSB configuration |
| `launch/mock_ros2_control.launch.py` | robot_state_publisher + ros2_control_node + spawners (`backend:=mock|vendor_tcp`) |
| `test/test_episode_system_hardware.cpp` | gmock: joint count, sign validation, lifecycle + tracking, limit ERROR, NaN ignore |

## API version

Written against the **current** hardware_interface API shipped with Jazzy
(ros2_control 4.48): `on_init(const HardwareComponentInterfaceParams&)`, state and
command interfaces created by the framework from the URDF, values accessed with
`set_state()` / `get_command()`. The older `on_init(const HardwareInfo&)` and
vector-returning `export_*_interfaces()` are deprecated in this release; the first
draft used them and was ported before building.

## Lifecycle mapping

| Callback | Behaviour |
|---|---|
| `on_init` | validates 6 joints, one `position` command + `position`/`velocity` state interfaces each; parses mapping/limits; signs must be ±1 |
| `on_configure` | creates backend (`mock` / `vendor_tcp`), probes it, starts feedback thread |
| `on_activate` | waits ≤ 1 s for a fresh sample; sets command = current position (no jump); else ERROR |
| `read()` | copies the latest sample → model radians; **ERROR if older than `stale_after_s` or ≥ 3 consecutive failures** |
| `write()` | skips NaN (no command yet); **ERROR if outside soft limits**; deadband + min interval before sending `angle_mode` |
| `on_deactivate` | if still moving → `emergency_stop` (no documented soft stop) |
| `on_error` | `emergency_stop` |
| `on_cleanup/shutdown` | stops thread, disconnects |

Observed (TCP Mock): killing the upper computer makes `read()` return ERROR; the
controller_manager deactivates the hardware component and both controllers.

## The impedance mismatch (most important design note)

`ros2_control` assumes a *streaming* position interface: JTC writes an
interpolated setpoint every cycle (e.g. 50 Hz) and expects the hardware to track
it. The official TCP API is *point-to-point*: each `angle_mode` plans its own
motion profile and replies with an estimated duration. Forwarding every setpoint
would (a) flood the upper computer, (b) restart the drive profile every cycle →
stutter. The implementation therefore rate-limits (deadband +
`min_command_interval_s`). Slow 3–4 s trajectories succeed against the TCP Mock;
how this behaves with the real upper computer and fast trajectories is unknown.
Proper solutions, in order of preference:

1. a streaming entry point in the official core (`dynamic_move`? — evidence E, needs vendor docs);
2. a backend on the official compiled control core (route B) if its API gets documented;
3. keep the Python `FollowJointTrajectory` server (`episode_ros2`) which honestly
   executes waypoints point-to-point.

## Build, test, run (executed on WSL2)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/episode_ws/src && ln -s /path/to/episode-arm-control/ros2_control episode_ros2_control
cd ~/episode_ws && colcon build --packages-select episode_ros2_control && source install/setup.bash
colcon test --packages-select episode_ros2_control && colcon test-result --verbose
ros2 launch episode_ros2_control mock_ros2_control.launch.py
ros2 control list_hardware_components
ros2 action send_goal /episode_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1,joint2,joint3,joint4,joint5,joint6], points: [{positions: [0.3,0,0,0,-0.2,0], time_from_start: {sec: 3}}]}}"
```
