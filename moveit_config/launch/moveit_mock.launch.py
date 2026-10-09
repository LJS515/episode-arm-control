"""MoveIt 2 move_group on top of either execution stack (Synthetic / Mock Data).

    ros2 launch episode_moveit_config moveit_mock.launch.py stack:=ros2_control model:=official
    ros2 launch episode_moveit_config moveit_mock.launch.py stack:=python model:=official

stack:=ros2_control  -> episode_ros2_control (controller_manager + JTC + mock backend)
stack:=python        -> episode_ros2 (TCP mock upper computer + safety-layer node)
                        + robot_state_publisher with the same URDF
model:=official      -> official URDF/STL (package episode1_urdf_1113, evidence B)
                        + config/episode1_official.srdf (collision matrix generated)
model:=placeholder   -> stick model without collision geometry

Both stacks serve /episode_arm_controller/follow_joint_trajectory, so one
moveit_controllers.yaml covers both.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODELS = {
    "placeholder": ("episode_mock.urdf.xacro", "config/episode1_mock.srdf", "episode1_mock"),
    "official": ("episode_official.urdf.xacro", "config/episode1_official.srdf", "episode1_urdf_1113"),
}


def _setup(context, *args, **kwargs):
    import xacro
    from moveit_configs_utils import MoveItConfigsBuilder

    stack = LaunchConfiguration("stack").perform(context)
    model = LaunchConfiguration("model").perform(context)
    xacro_name, srdf, robot_name = MODELS[model]
    r2c_share = get_package_share_directory("episode_ros2_control")
    urdf_xacro = os.path.join(r2c_share, "urdf", xacro_name)

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="episode_moveit_config")
        .robot_description(file_path=urdf_xacro, mappings={"backend": "mock"})
        .robot_description_semantic(file_path=srdf)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    actions = [
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[moveit_config.to_dict()]),
    ]

    if model == "official":
        # The official URDF's root is base_link; the SRDF virtual joint plans in "world".
        # Publish world -> base_link so RViz (fixed frame "world") can display the robot.
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher",
                            arguments=["--frame-id", "world", "--child-frame-id", "base_link"]))

    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        rviz_cfg = os.path.join(get_package_share_directory("episode_moveit_config"), "config", "moveit.rviz")
        actions.append(Node(
            package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_cfg],
            parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                        moveit_config.joint_limits]))

    if stack == "ros2_control":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(r2c_share, "launch", "mock_ros2_control.launch.py")),
            launch_arguments={"model": model}.items()))
    elif stack == "python":
        urdf = xacro.process_file(urdf_xacro, mappings={"backend": "mock"}).toxml()
        fd, path = tempfile.mkstemp(prefix="episode_", suffix=".urdf")
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(urdf)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("episode_ros2"),
                                                       "launch", "mock.launch.py")),
            launch_arguments={"urdf": path}.items()))
    elif stack == "none":
        pass  # move_group only (e.g. for tools/generate_collision_matrix.py)
    else:
        raise RuntimeError(f"unknown stack {stack!r} (use ros2_control, python or none)")
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stack", default_value="ros2_control"),
        DeclareLaunchArgument("model", default_value="official"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
