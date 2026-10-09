"""ros2_control bring-up with the MOCK backend (Synthetic / Mock Data).

    ros2 launch episode_ros2_control mock_ros2_control.launch.py
    ros2 launch episode_ros2_control mock_ros2_control.launch.py backend:=vendor_tcp port:=12345   # against a TCP mock / official upper computer

Starts robot_state_publisher, ros2_control_node, joint_state_broadcaster and
episode_arm_controller (JTC). model:=placeholder (stick model, default) or
model:=official (official URDF/STL from package episode1_urdf_1113).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    import xacro

    share = get_package_share_directory("episode_ros2_control")
    model = LaunchConfiguration("model").perform(context)
    xacro_file = {"placeholder": "episode_mock.urdf.xacro", "official": "episode_official.urdf.xacro"}[model]
    doc = xacro.process_file(
        os.path.join(share, "urdf", xacro_file),
        mappings={k: LaunchConfiguration(k).perform(context) for k in ("backend", "host", "port")},
    )
    robot_description = {"robot_description": doc.toxml()}
    controllers = os.path.join(share, "config", "controllers.yaml")
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers], remappings=[("~/robot_description", "/robot_description")],
             output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["episode_arm_controller", "--controller-manager", "/controller_manager"]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mock"),
        # official = official URDF/STL (needs package episode1_urdf_1113 in the workspace)
        DeclareLaunchArgument("model", default_value="placeholder"),
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="12345"),
        OpaqueFunction(function=_nodes),
    ])
