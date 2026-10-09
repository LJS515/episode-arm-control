#!/usr/bin/env python3
"""Runtime smoke test for `ros2 launch episode_ros2 mock.launch.py` (Synthetic / Mock Data).

Run in a sourced ROS2 Jazzy shell while mock.launch.py is running:

    python3 episode_ros2/test/ros2_runtime_smoke.py

Checks, in order:
  1. /episode/robot_state is READY (latched topic)
  2. /joint_states arrives at roughly the configured rate and is all zeros (vertical posture)
  3. FollowJointTrajectory goal succeeds, feedback carries desired/actual/error
  4. a goal outside the joint limits is rejected
  5. cancel during motion -> goal CANCELED, arm stops (no soft stop on vendor_tcp -> ESTOP)
  6. ~/release_estop(data=true) -> INACTIVE; ~/declare_homed -> READY
  7. mock drops replies -> FAULT, /joint_states stops, /diagnostics reports a problem
  8. replies restored -> ~/clear_fault -> INACTIVE
Exit code 0 only if every check passes.
"""

import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
IFACE = "/episode_robot_interface"
MOCK = "/episode_mock_hardware"


class Smoke(Node):
    def __init__(self):
        super().__init__("episode_smoke_test")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = None
        self.js = []
        self.diag = []
        self.create_subscription(String, "/episode/robot_state", lambda m: setattr(self, "state", m.data), latched)
        self.create_subscription(JointState, "/joint_states", lambda m: self.js.append((time.monotonic(), list(m.position))), 50)
        self.create_subscription(DiagnosticArray, "/diagnostics", lambda m: self.diag.append(m), 20)
        self.ac = ActionClient(self, FollowJointTrajectory, "/episode_arm_controller/follow_joint_trajectory")
        self.results = []

    def call(self, srv_type, name, **kw):
        cli = self.create_client(srv_type, name)
        assert cli.wait_for_service(timeout_sec=5.0), f"service {name} not available"
        fut = cli.call_async(srv_type.Request(**kw))
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10:
            time.sleep(0.02)
        assert fut.done(), f"service {name} timed out"
        return fut.result()

    def wait_state(self, want, timeout=10.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.state == want:
                return True
            time.sleep(0.05)
        return False

    def goal(self, positions, sec=2):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        g.trajectory.points = [JointTrajectoryPoint(positions=positions, time_from_start=rclpy.duration.Duration(seconds=sec).to_msg())]
        return g

    def wait_future(self, fut, timeout):
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        return fut.done()


def main() -> int:
    rclpy.init()
    n = Smoke()
    ex = MultiThreadedExecutor(4)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    results = []

    def check(name, ok, info=""):
        results.append((name, bool(ok)))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {info}")

    # 1
    check("robot_state READY", n.wait_state("READY", 15), f"(got {n.state})")
    # 2
    time.sleep(2.0)
    recent = [p for t, p in n.js if time.monotonic() - t < 2.0]
    rate = len(recent) / 2.0
    check("joint_states rate >= 10 Hz", rate >= 10, f"({rate:.1f} Hz)")
    check("joint_states vertical = zeros", recent and all(abs(v) < 1e-6 for v in recent[-1]), f"({recent[-1] if recent else None})")
    # 3
    assert n.ac.wait_for_server(timeout_sec=5.0)
    fbs = []
    gh_fut = n.ac.send_goal_async(n.goal([0.15, 0.0, 0.0, 0.0, 0.0, 0.0]), feedback_callback=lambda f: fbs.append(f.feedback))
    n.wait_future(gh_fut, 5)
    gh = gh_fut.result()
    check("goal accepted", gh is not None and gh.accepted)
    res_fut = gh.get_result_async()
    n.wait_future(res_fut, 30)
    res = res_fut.result()
    check("goal SUCCEEDED", res is not None and res.status == GoalStatus.STATUS_SUCCEEDED,
          f"(status={getattr(res, 'status', None)}, error_code={getattr(getattr(res, 'result', None), 'error_code', None)})")
    has_fb = bool(fbs) and len(fbs[-1].actual.positions) == 6 and len(fbs[-1].error.positions) == 6
    check("feedback has desired/actual/error", has_fb, f"({len(fbs)} feedback msgs)")
    time.sleep(0.3)
    check("joint_states reached 0.15 rad", n.js and abs(n.js[-1][1][0] - 0.15) < 0.02, f"({n.js[-1][1][0]:.4f})")
    # 4
    gh_fut = n.ac.send_goal_async(n.goal([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    n.wait_future(gh_fut, 5)
    check("out-of-limit goal rejected", gh_fut.result() is not None and not gh_fut.result().accepted)
    # 5
    gh_fut = n.ac.send_goal_async(n.goal([1.2, 0.0, 0.0, 0.0, 0.0, 0.0], sec=10))
    n.wait_future(gh_fut, 5)
    gh = gh_fut.result()
    time.sleep(0.6)
    cancel_fut = gh.cancel_goal_async()
    n.wait_future(cancel_fut, 5)
    res_fut = gh.get_result_async()
    n.wait_future(res_fut, 10)
    check("cancel -> goal CANCELED", res_fut.result() is not None and res_fut.result().status == GoalStatus.STATUS_CANCELED,
          f"(status={getattr(res_fut.result(), 'status', None)})")
    check("cancel -> ESTOP (vendor_tcp has no soft stop)", n.wait_state("ESTOP", 5), f"(state={n.state})")
    p1 = n.js[-1][1][0] if n.js else None
    time.sleep(1.0)
    p2 = n.js[-1][1][0] if n.js else None
    check("arm stopped short of 1.2 rad", p1 is not None and p2 is not None and abs(p2 - p1) < 1e-3 and p2 < 1.1,
          f"({p1}, {p2})")
    # 6
    r = n.call(SetBool, f"{IFACE}/release_estop", data=True)
    check("release_estop -> INACTIVE", r.success and n.wait_state("INACTIVE", 5), f"({r.message})")
    r = n.call(Trigger, f"{IFACE}/declare_homed")
    check("declare_homed -> READY", r.success and n.wait_state("READY", 5), f"({r.message})")
    # 7
    n.call(SetBool, f"{MOCK}/drop_replies", data=True)
    check("feedback loss -> FAULT", n.wait_state("FAULT", 10), f"(state={n.state})")
    diag_mark = len(n.diag)
    count = len(n.js)
    time.sleep(1.0)
    check("joint_states stops while stale", len(n.js) == count, f"(+{len(n.js) - count} msgs)")
    time.sleep(2.0)
    after = [s for d in n.diag[diag_mark:] for s in d.status]
    watchdog = [s for s in after if s.name == "episode/joint_states_watchdog" and s.level == b"\x03"]
    state_err = [s for s in after if s.name in ("episode/state", "episode/robot_state") and "FAULT" in s.message]
    check("/diagnostics: watchdog STALE + state FAULT", bool(watchdog) and bool(state_err),
          f"({[(s.name, s.message) for s in (watchdog[:1] + state_err[:1])]})")
    # 8
    n.call(SetBool, f"{MOCK}/drop_replies", data=False)
    time.sleep(1.5)
    r = n.call(Trigger, f"{IFACE}/clear_fault")
    check("clear_fault -> INACTIVE", r.success and n.wait_state("INACTIVE", 5), f"({r.message})")

    ok = all(v for _, v in results)
    print(f"\n{sum(v for _, v in results)}/{len(results)} checks passed — Synthetic / Mock Data")
    n.ac.destroy()  # destroy the action client before the executor stops (avoids "never retrieved" noise)
    ex.shutdown()
    n.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
