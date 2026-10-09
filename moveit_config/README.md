# episode_moveit_config — MoveIt 2 configuration (official model + placeholder)

> **Status:** run with MoveIt 2.12.4 on ROS2 Jazzy (WSL2) with the **official
> Episode 1 URDF/STL** (`model:=official`): `test/moveit_smoke.py --collision-check`
> passes **11/11 on both execution stacks**, including refusal of a self-colliding
> target. **Synthetic / Mock Data** (mock backends, no hardware).
>
> The official model package `episode1_urdf_1113` (official student_ros V1.0) is
> **not** redistributed here — put it into your workspace `src/`. A placeholder
> stick model without collision geometry remains available (`model:=placeholder`).

## Use

```bash
# build together with episode_ros2 and episode_ros2_control (see ../README.md, run colcon with the venv python)
ros2 launch episode_moveit_config moveit_mock.launch.py stack:=ros2_control model:=official   # or stack:=python
python3 test/moveit_smoke.py --stack ros2_control --collision-check
# regenerate the collision matrix (move_group only):
ros2 launch episode_moveit_config moveit_mock.launch.py stack:=none model:=official
python3 tools/generate_collision_matrix.py --samples 2000 --srdf config/episode1_official.srdf --write
python3 test/moveit_debug_plan.py 0 2.5 0 0 0 0      # plan only, print planned end point
```

## Key settings and why

| Setting | Value | Reason |
|---|---|---|
| `joint_limits.yaml` max_velocity | 0.25 rad/s | must stay below the execution backend's real speed; 0.8 rad/s made JTC abort (`State tolerances failed for joint 5`) |
| `allowed_execution_duration_scaling` | 3.0 | the Python stack runs stop-and-go waypoints and ignores `time_from_start` |
| response adapter | `DisplayMotionPath` | MoveIt 2.12 name (`DisplayMotionPlans` does not exist → move_group crashes) |

## Behaviour to know

MoveIt **clamps** an out-of-bounds joint target to the URDF bound (WARN "Assuming
maximum bounds instead") and plans to the bound instead of rejecting it. The
Python interface node rejects such trajectories in its goal callback. Callers
must validate their own targets.

Real joint speed/acceleration limits: **TBD (待真机测量)**.
