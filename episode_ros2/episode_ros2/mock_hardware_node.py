"""Runs the TCP Mock Server inside a ROS2 node, with fault-injection services.

STATUS: run under ROS2 Jazzy (WSL2); see reports/ros2_runtime_wsl.md.
All data is Synthetic / Mock Data.

The interface node can then use ``backend:=vendor_tcp`` against this mock —
exercising the full path SDK -> TCP -> "upper computer" exactly as with the
real official software, minus CAN and motors.

Fault services (std_srvs/SetBool):
  ~/drop_replies        data=true -> stop answering (feedback loss)
  ~/none_joint3         data=true -> joint 3 reported as None (FAQ tid=1932 pattern)
  ~/ik_fail             data=true -> every Cartesian move returns -1
  ~/belt_slip_j5        data=true -> J5 ends 4 deg short (FAQ tid=1996 pattern)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from episode_sdk.mock_server import MockEpisodeServer
from episode_sdk.models import ProtocolMode


class MockHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("episode_mock_hardware")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 12345)
        self.declare_parameter("mode", "legacy")
        self.declare_parameter("homed", True)
        self.server = MockEpisodeServer(
            self.get_parameter("host").value, int(self.get_parameter("port").value),
            mode=ProtocolMode(self.get_parameter("mode").value), homed=bool(self.get_parameter("homed").value),
        ).start()
        self.get_logger().warn(f"MOCK upper computer on {self.server.host}:{self.server.port} — Synthetic / Mock Data")
        self.create_service(SetBool, "~/drop_replies", self._drop)
        self.create_service(SetBool, "~/none_joint3", self._none3)
        self.create_service(SetBool, "~/ik_fail", self._ik)
        self.create_service(SetBool, "~/belt_slip_j5", self._slip)

    def _ok(self, res, msg):
        res.success, res.message = True, msg
        return res

    def _drop(self, req, res):
        self.server.faults.drop_next_replies = 10 ** 9 if req.data else 0
        return self._ok(res, f"drop_replies={req.data}")

    def _none3(self, req, res):
        self.server.faults.none_joints = [3] if req.data else []
        return self._ok(res, f"none_joint3={req.data}")

    def _ik(self, req, res):
        self.server.arm.faults.ik_fail = bool(req.data)
        return self._ok(res, f"ik_fail={req.data}")

    def _slip(self, req, res):
        self.server.arm.faults.final_offset_deg = {5: 4.0} if req.data else {}
        return self._ok(res, f"belt_slip_j5={req.data}")

    def destroy_node(self) -> bool:
        self.server.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
