"""Mock bring-up: mock upper computer + interface node (vendor_tcp backend) + watchdog.

    ros2 launch episode_ros2 mock.launch.py
    ros2 launch episode_ros2 mock.launch.py urdf:=/path/to/episode.urdf   # also start robot_state_publisher

Everything is Synthetic / Mock Data. STATUS: run under ROS2 Jazzy (WSL2); smoke test 17/17.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rsp(context, *args, **kwargs):
    urdf = LaunchConfiguration("urdf").perform(context)
    if not urdf:
        return []
    with open(urdf, "r", encoding="utf-8") as fh:
        desc = fh.read()
    return [Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": desc}])]


def generate_launch_description():
    share = get_package_share_directory("episode_ros2")
    robot_yaml = os.path.join(share, "config", "robot.yaml")
    safety_yaml = os.path.join(share, "config", "safety.yaml")
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="12399"),
        DeclareLaunchArgument("urdf", default_value=""),
        Node(package="episode_ros2", executable="mock_hardware_node", name="episode_mock_hardware",
             parameters=[robot_yaml, {"port": port}], output="screen"),
        Node(package="episode_ros2", executable="robot_interface_node", name="episode_robot_interface",
             parameters=[robot_yaml, safety_yaml,
                         {"backend": "vendor_tcp", "port": port, "assume_vendor_homed": True}],
             output="screen"),
        Node(package="episode_ros2", executable="diagnostics_node", name="episode_diagnostics",
             parameters=[robot_yaml], output="screen"),
        OpaqueFunction(function=_rsp),
    ])
