#!/usr/bin/env python3
"""MoveIt plan + execute smoke test (Synthetic / Mock Data).

Run while `ros2 launch episode_moveit_config moveit_mock.launch.py stack:=<...>` is up:

    python3 moveit_config/test/moveit_smoke.py --stack ros2_control
    python3 moveit_config/test/moveit_smoke.py --stack python

Uses the MoveGroup action directly (no moveit_py needed):
  1. joint goal A  -> plan + execute must succeed, /joint_states must reach A
  2. joint goal B  -> plan + execute must succeed (multi-joint)
  3. back to the "vertical" posture (all zeros)
  4. plan-only to a joint target outside the URDF limits -> MoveIt clamps it to the
     bound; the planned end point must not exceed the limit
Exit code 0 only if all checks pass.
"""

import argparse
import sys
import threading
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CODE_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if k.isupper() and isinstance(v, int)}


class Smoke(Node):
    def __init__(self):
        super().__init__("episode_moveit_smoke")
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        self.ac = ActionClient(self, MoveGroup, "/move_action")

    def _js(self, m: JointState):
        self.js = dict(zip(m.name, m.position))

    def goal(self, target, plan_only=False):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = "arm"
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = 0.5
        r.max_acceleration_scaling_factor = 0.5
        c = Constraints()
        for n, v in zip(JOINTS, target):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=0.01,
                                                       tolerance_below=0.01, weight=1.0))
        r.goal_constraints = [c]
        g.planning_options.plan_only = plan_only
        g.planning_options.replan = False
        return g

    def run(self, target, plan_only=False, timeout=120.0):
        """Returns (error_code or None, seconds, planned end positions or None)."""
        fut = self.ac.send_goal_async(self.goal(target, plan_only))
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10:
            time.sleep(0.02)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, 0.0, None
        rf = gh.get_result_async()
        while not rf.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        if not rf.done():
            return None, time.monotonic() - t0, None
        res = rf.result().result
        pts = res.planned_trajectory.joint_trajectory.points
        end = None
        if pts:
            names = list(res.planned_trajectory.joint_trajectory.joint_names)
            end = [pts[-1].positions[names.index(j)] for j in JOINTS]
        return res.error_code.val, time.monotonic() - t0, end


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stack", default="ros2_control")
    ap.add_argument("--collision-check", action="store_true",
                    help="also verify that a self-colliding target is refused (needs a model with collision geometry)")
    args = ap.parse_args()
    rclpy.init()
    n = Smoke()
    ex = MultiThreadedExecutor(4)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    results = []

    def check(name, ok, info=""):
        results.append(bool(ok))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {info}")

    check("move_group action server", n.ac.wait_for_server(timeout_sec=30.0))
    t0 = time.monotonic()
    while not n.js and time.monotonic() - t0 < 10:
        time.sleep(0.1)
    check("joint_states available", bool(n.js), f"({n.js})")

    for label, target in [("goal A (joint1 0.4, joint5 -0.3)", [0.4, 0.0, 0.0, 0.0, -0.3, 0.0]),
                          ("goal B (multi-joint)", [-0.3, 0.3, -0.4, 0.5, 0.6, 1.0]),
                          ("back to vertical", [0.0] * 6)]:
        code, dt, _ = n.run(target)
        time.sleep(0.5)
        actual = [n.js.get(j, float("nan")) for j in JOINTS]
        err = max(abs(a - t) for a, t in zip(actual, target))
        check(f"{label}: plan+execute SUCCESS", code == MoveItErrorCodes.SUCCESS,
              f"(code={code} {CODE_NAMES.get(code, '')}, {dt:.1f}s)")
        check(f"{label}: joint_states reached target", code == MoveItErrorCodes.SUCCESS and err < 0.03,
              f"(max err {err:.4f} rad)")

    # MoveIt does NOT reject an out-of-bounds joint target: it clamps the constraint to the
    # URDF bound ("Assuming maximum bounds instead") and plans to the bound. Verify the
    # plan never leaves the limits. (The python stack's goal callback rejects such targets.)
    code, _, end = n.run([0.0, 2.5, 0.0, 0.0, 0.0, 0.0], plan_only=True)  # joint2 limit +-pi/2
    check("out-of-limit target: plan clamped to URDF bound, never beyond",
          code == MoveItErrorCodes.SUCCESS and end is not None and end[1] <= 1.5708 + 1e-4,
          f"(code={code} {CODE_NAMES.get(code, '')}, planned joint2 end={None if end is None else round(end[1], 4)})")

    if args.collision_check:
        # Find a self-colliding joint state (deterministic sampling) and require
        # MoveIt to refuse planning to it -> proves collision geometry is active.
        import math
        import random

        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity

        cli = n.create_client(GetStateValidity, "/check_state_validity")
        cli.wait_for_service(timeout_sec=10)
        limits = [(-180, 160), (-90, 90), (-83, 80), (-30, 305), (-110, 110), (-30, 305)]
        rng = random.Random(7)
        colliding, pair = None, None
        for _ in range(3000):
            q = [math.radians(rng.uniform(lo, hi)) for lo, hi in limits]
            req = GetStateValidity.Request(group_name="arm",
                                           robot_state=RobotState(joint_state=JointState(name=JOINTS, position=q)))
            fut = cli.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 5:
                time.sleep(0.002)
            res = fut.result()
            if res is not None and not res.valid and res.contacts:
                colliding = q
                pair = (res.contacts[0].contact_body_1, res.contacts[0].contact_body_2)
                break
        check("self-colliding state found by sampling", colliding is not None, f"(contact {pair})")
        if colliding is not None:
            code, _, _ = n.run(colliding, plan_only=True)
            check("plan to self-colliding target refused", code is not None and code != MoveItErrorCodes.SUCCESS,
                  f"(code={code} {CODE_NAMES.get(code, '')})")

    ok = all(results)
    print(f"\n[{args.stack}] {sum(results)}/{len(results)} checks passed — Synthetic / Mock Data")
    n.ac.destroy()
    ex.shutdown()
    n.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
