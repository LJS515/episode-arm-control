"""Independent watchdog: is /joint_states alive, fresh and plausible?

STATUS: run under ROS2 Jazzy (WSL2) with the Mock; see reports/ros2_runtime_wsl.md.

Why a separate node: a monitor living only inside the process it monitors
goes silent together with it. This node observes the interface node from
the outside and publishes ``diagnostic_msgs/DiagnosticArray`` on
``/diagnostics`` (aggregator-compatible).

Checks
------
* message rate of /joint_states vs. expected rate
* age of the last message (receive time) and of its header stamp
* stamps never go backwards
* NaN / Inf in positions
* robot state reported on /episode/robot_state (latched)
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

OK, WARN, ERROR, STALE = (bytes([i]) for i in range(4))


class DiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("episode_diagnostics")
        self.declare_parameter("expected_rate_hz", 20.0)
        self.declare_parameter("stale_after_s", 0.5)
        self.declare_parameter("min_rate_ratio", 0.5)
        self.expected = float(self.get_parameter("expected_rate_hz").value)
        self.stale_after = float(self.get_parameter("stale_after_s").value)
        self.min_ratio = float(self.get_parameter("min_rate_ratio").value)
        self.rx_times: Deque[float] = deque(maxlen=200)
        self.last_stamp: Optional[float] = None
        self.backwards = 0
        self.non_finite = 0
        self.robot_state = "UNKNOWN"
        self.create_subscription(JointState, "joint_states", self._on_js, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "episode/robot_state", self._on_state, latched)
        self.pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.create_timer(1.0, self._publish)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_js(self, msg: JointState) -> None:
        self.rx_times.append(self._now())
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.backwards += 1
        self.last_stamp = stamp
        if any(not math.isfinite(p) for p in msg.position):
            self.non_finite += 1

    def _on_state(self, msg: String) -> None:
        self.robot_state = msg.data

    def _publish(self) -> None:
        now = self._now()
        st = DiagnosticStatus(name="episode/joint_states_watchdog", hardware_id="episode1")
        recent = [t for t in self.rx_times if now - t <= 2.0]
        rate = len(recent) / 2.0
        age = None if not self.rx_times else now - self.rx_times[-1]
        if age is None:
            st.level, st.message = STALE, "no /joint_states received"
        elif age > self.stale_after:
            st.level, st.message = STALE, f"/joint_states silent for {age:.2f}s"
        elif self.non_finite or self.backwards:
            st.level, st.message = ERROR, "non-finite values or stamps going backwards"
        elif rate < self.expected * self.min_ratio:
            st.level, st.message = WARN, f"rate {rate:.1f} Hz below {self.expected * self.min_ratio:.1f} Hz"
        else:
            st.level, st.message = OK, f"{rate:.1f} Hz"
        st.values = [KeyValue(key=k, value=str(v)) for k, v in (
            ("rate_hz", round(rate, 2)), ("age_s", None if age is None else round(age, 3)),
            ("stamps_backwards", self.backwards), ("non_finite", self.non_finite), ("robot_state", self.robot_state))]
        state = DiagnosticStatus(name="episode/robot_state", hardware_id="episode1", message=self.robot_state,
                                 level=ERROR if self.robot_state in ("FAULT", "ESTOP") else OK)
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st, state]
        self.pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
