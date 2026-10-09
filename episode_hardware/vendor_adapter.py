"""Adapter from ``RobotHardware`` to the official TCP/JSON upper computer.

STATUS: implemented against the Mock Server; **not yet validated on the
real robot** (pending hardware).

Capability notes (why ``soft_stop`` is False)
-------------------------------------------
The official command list (evidence B) contains ``emergency_stop`` but no
documented "stop the current motion" command. A common workaround — sending
``angle_mode`` to the current position — is NOT used here, because:

* the historical server handles each client connection in its own thread
  and ``move_linear_xyz_rotation`` blocks that thread while executing
  (evidence C), so a second command arrives on a *different* thread and may
  interleave on the CAN bus (evidence D) rather than replace the motion;
* whether the drivers accept a new position target mid-move is undocumented.

Until verified on hardware, a requested stop escalates to
``emergency_stop`` (configurable: ``stop_strategy``).
"""

from __future__ import annotations

import logging
from typing import Any, List, Optional, Sequence

from episode_sdk.client import EpisodeClient
from episode_sdk.exceptions import EpisodeError, NoSolutionError, TransportError
from episode_sdk.models import Pose

from .interfaces import HardwareCapabilities, HardwareCommandError, HardwareReadError, RobotHardware

log = logging.getLogger("episode_hardware.vendor")


class VendorTcpAdapter(RobotHardware):
    def __init__(self, client: EpisodeClient, *, stop_strategy: str = "estop", default_speed_cap: float = 1.0):
        if stop_strategy not in ("estop", "hold_position"):
            raise ValueError("stop_strategy must be 'estop' or 'hold_position'")
        self.client = client
        self.stop_strategy = stop_strategy
        self.default_speed_cap = default_speed_cap
        self._connected = False

    @property
    def capabilities(self) -> HardwareCapabilities:
        return HardwareCapabilities(
            soft_stop=self.stop_strategy == "hold_position",
            streaming=False,
            reports_completion=False,
            software_estop_release=True,  # emergency_stop params 0 (evidence B)
            description=f"official TCP upper computer at {self.client.host}:{self.client.port} "
                        f"(mode={self.client.mode.value})",
        )

    def connect(self) -> None:
        # probe once; raises a TransportError subclass on failure
        try:
            self.client.get_motor_angles()
        except EpisodeError as exc:
            raise HardwareCommandError(f"cannot reach official server: {exc}") from exc
        self._connected = True

    def disconnect(self) -> None:
        self.client.close()
        self._connected = False

    def read_joint_positions(self) -> List[Optional[float]]:
        try:
            return self.client.get_motor_angles()
        except EpisodeError as exc:  # incl. ReadFailedError (server returned None)
            raise HardwareReadError(f"get_motor_angles failed: {exc.code}") from exc

    def command_joint_positions(self, motor_deg: Sequence[float], speed_ratio: float) -> Optional[float]:
        try:
            return self.client.angle_mode(list(motor_deg), min(speed_ratio, self.default_speed_cap)).estimated_duration_s
        except EpisodeError as exc:
            raise HardwareCommandError(f"angle_mode failed: {exc.code}") from exc

    def move_pose(self, pose: Pose, linear: bool) -> Optional[float]:
        fn = self.client.move_linear_xyz_rotation if linear else self.client.move_xyz_rotation
        try:
            return fn(pose.x, pose.y, pose.z, (pose.rx, pose.ry, pose.rz), pose.order).estimated_duration_s
        except NoSolutionError:
            raise
        except EpisodeError as exc:
            raise HardwareCommandError(f"pose move failed: {exc.code}") from exc

    def home(self) -> Optional[float]:
        return self.client.home().estimated_duration_s

    def soft_stop(self) -> bool:
        if self.stop_strategy == "hold_position":
            # UNVERIFIED (evidence E): may not preempt an executing motion.
            cur = self.client.get_motor_angles()
            if any(c is None for c in cur):
                return False
            self.client.angle_mode([float(c) for c in cur], 0.1)  # type: ignore[arg-type]
            return True
        return False

    def emergency_stop(self) -> None:
        try:
            self.client.emergency_stop()
        except TransportError:
            log.critical("software emergency_stop could not be delivered — use the HARDWARE E-stop")
            raise

    def release_emergency_stop(self) -> bool:
        try:
            self.client.release_emergency_stop()
            return True
        except EpisodeError as exc:
            raise HardwareCommandError(f"release emergency stop failed: {exc.code}") from exc

    def set_free_mode(self, enable: bool) -> None:
        self.client.set_free_mode(enable)

    def set_gripper(self, closed: bool) -> None:
        (self.client.gripper_on if closed else self.client.gripper_off)()

    def set_servo_gripper(self, angle_deg: float) -> None:
        self.client.servo_gripper(angle_deg)
