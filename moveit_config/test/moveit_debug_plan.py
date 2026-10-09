#!/usr/bin/env python3
"""Debug helper: plan-only to a joint target and print the planned end point and URDF limits."""

import sys
import threading
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def main():
    target = [float(v) for v in sys.argv[1:7]]
    rclpy.init()
    n = Node("episode_moveit_debug")
    ex = MultiThreadedExecutor(2)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    ac = ActionClient(n, MoveGroup, "/move_action")
    ac.wait_for_server(timeout_sec=20)
    g = MoveGroup.Goal()
    g.request.group_name = "arm"
    g.request.allowed_planning_time = 5.0
    c = Constraints()
    for j, v in zip(JOINTS, target):
        c.joint_constraints.append(JointConstraint(joint_name=j, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
    g.request.goal_constraints = [c]
    g.planning_options.plan_only = True
    f = ac.send_goal_async(g)
    while not f.done():
        time.sleep(0.05)
    r = f.result().get_result_async()
    while not r.done():
        time.sleep(0.05)
    res = r.result().result
    pts = res.planned_trajectory.joint_trajectory.points
    print("error_code", res.error_code.val, "points", len(pts))
    if pts:
        print("names", res.planned_trajectory.joint_trajectory.joint_names)
        print("end", [round(p, 4) for p in pts[-1].positions])
    ac.destroy()
    ex.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
