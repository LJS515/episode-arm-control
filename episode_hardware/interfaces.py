"""Hardware abstraction used by the safety / control layer.

Two implementations exist:

* ``VendorTcpAdapter`` — wraps the official TCP/JSON upper computer
  (route A). The CAN bus is owned by the official process.
* ``MockHardwareAdapter`` — in-process simulation with fault injection.

A future ``VendorCoreAdapter`` could wrap the compiled
``episode_controller_core`` library from the official ROS2 package
(route B). Its C API is not public, so it is only a placeholder here.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import List, Optional, Sequence

from episode_sdk.exceptions import EpisodeError
from episode_sdk.models import Pose


class HardwareError(EpisodeError):
    code = "HARDWARE_ERROR"


class HardwareReadError(HardwareError):
    code = "HARDWARE_READ_FAILED"
    retryable = True


class HardwareCommandError(HardwareError):
    code = "HARDWARE_COMMAND_FAILED"


@dataclass(frozen=True)
class HardwareCapabilities:
    #: can a running motion be stopped without an emergency stop?
    soft_stop: bool
    #: does the backend accept high-rate streaming targets?
    streaming: bool
    #: does the backend report motion completion (vs. only an estimate)?
    reports_completion: bool
    #: can ESTOP be released by software? (on real hardware: operator action)
    software_estop_release: bool
    description: str = ""


class RobotHardware(abc.ABC):
    """All angles are *motor degrees* (see joint_mapping)."""

    @property
    @abc.abstractmethod
    def capabilities(self) -> HardwareCapabilities: ...

    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    @abc.abstractmethod
    def read_joint_positions(self) -> List[Optional[float]]:
        """Return 6 motor angles; individual entries may be ``None``.

        Raises ``HardwareReadError`` when nothing could be read.
        """

    @abc.abstractmethod
    def command_joint_positions(self, motor_deg: Sequence[float], speed_ratio: float) -> Optional[float]:
        """Start a point-to-point joint move; returns estimated duration."""

    @abc.abstractmethod
    def move_pose(self, pose: Pose, linear: bool) -> Optional[float]: ...

    @abc.abstractmethod
    def home(self) -> Optional[float]: ...

    @abc.abstractmethod
    def soft_stop(self) -> bool:
        """Request a controlled stop. Returns False if unsupported."""

    @abc.abstractmethod
    def emergency_stop(self) -> None: ...

    def release_emergency_stop(self) -> bool:
        """Default: not possible from software."""
        return False

    @abc.abstractmethod
    def set_free_mode(self, enable: bool) -> None: ...

    @abc.abstractmethod
    def set_gripper(self, closed: bool) -> None: ...

    @abc.abstractmethod
    def set_servo_gripper(self, angle_deg: float) -> None: ...
