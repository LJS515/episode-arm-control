#!/usr/bin/env bash
# One-command simulation: official Episode 1 model in RViz + MoveIt + mock execution.
# Synthetic / Mock Data — no robot, no CAN.
#
#   bash moveit_config/scripts/start_sim.sh                 # ros2_control + JTC (default)
#   STACK=python bash moveit_config/scripts/start_sim.sh    # this project's safety-layer node
#
# Prerequisites (see README.md §7): workspace ~/episode_ws built with
# `python -m colcon build` inside ~/episode_ws/venv, and the official package
# episode1_urdf_1113 in ~/episode_ws/src.
set -e
WS=${WS:-$HOME/episode_ws}
STACK=${STACK:-ros2_control}
source /opt/ros/jazzy/setup.bash
source "$WS/venv/bin/activate"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-21}
echo "starting simulation (stack=$STACK, model=official) — close the RViz window or press Ctrl-C to stop"
exec ros2 launch episode_moveit_config moveit_mock.launch.py stack:="$STACK" model:=official rviz:=true
