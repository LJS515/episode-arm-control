"""Real-hardware bring-up against the OFFICIAL upper computer (route A).

    ros2 launch episode_ros2 hardware.launch.py host:=192.168.1.10 confirm_real_robot:=true

Preconditions (docs/12_safety_architecture.md, first power-on checklist):
  * official upper computer running, robot homed by it, workspace clear
  * hardware E-stop / power switch within reach
  * speed_ratio_cap <= 0.3 for the first runs
STATUS: not yet run on hardware.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _guard(context, *args, **kwargs):
    if LaunchConfiguration("confirm_real_robot").perform(context).lower() != "true":
        return [LogInfo(msg="refusing to start: pass confirm_real_robot:=true after the safety checklist"),
                Shutdown(reason="real robot not confirmed")]
    share = get_package_share_directory("episode_ros2")
    return [
        Node(package="episode_ros2", executable="robot_interface_node", name="episode_robot_interface",
             parameters=[os.path.join(share, "config", "robot.yaml"),
                         os.path.join(share, "config", "safety.yaml"),
                         {"backend": "vendor_tcp",
                          "host": LaunchConfiguration("host").perform(context),
                          "port": int(LaunchConfiguration("port").perform(context)),
                          "assume_vendor_homed": False}],
             output="screen"),
        Node(package="episode_ros2", executable="diagnostics_node", name="episode_diagnostics",
             parameters=[os.path.join(share, "config", "robot.yaml")], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="12345"),
        DeclareLaunchArgument("confirm_real_robot", default_value="false"),
        OpaqueFunction(function=_guard),
    ])
