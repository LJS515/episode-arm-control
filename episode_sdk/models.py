"""Data models shared by the SDK, the hardware layer and the ROS2 adapters."""

from __future__ import annotations

import enum
import itertools
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

NUM_JOINTS = 6
JOINT_NAMES: Tuple[str, ...] = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


class ProtocolMode(str, enum.Enum):
    """Which JSON envelope is used on the wire.

    LEGACY   ``{"action": ..., "params": ...}`` -> raw result value.
             Compatible with the official server; nothing else is assumed.
    ENHANCED Adds ``protocol_version``, ``request_id`` and a structured
             ``{"ok", "result", "error"}`` reply. Only our Mock Server (or a
             future server we control) speaks this mode.
    """

    LEGACY = "legacy"
    ENHANCED = "enhanced"


class ConnectionPolicy(str, enum.Enum):
    #: open a fresh TCP connection for every command (what the historical
    #: official client did; safest default for compatibility)
    PER_COMMAND = "per_command"
    #: keep one connection open (the historical server loops per connection)
    PERSISTENT = "persistent"


@dataclass(frozen=True)
class ClientTimeouts:
    connect_s: float = 2.0
    send_s: float = 2.0
    receive_s: float = 5.0
    #: hard upper bound for one request/response round trip, including
    #: connect + send + receive.
    command_total_s: float = 10.0


@dataclass(frozen=True)
class JointLimits:
    """Per-joint [min, max] in *motor/physical* degrees."""

    lower: Tuple[float, ...]
    upper: Tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.lower) != NUM_JOINTS or len(self.upper) != NUM_JOINTS:
            raise ValueError("JointLimits needs 6 lower and 6 upper values")
        for lo, hi in zip(self.lower, self.upper):
            if lo >= hi:
                raise ValueError(f"invalid limit pair [{lo}, {hi}]")

    @classmethod
    def episode_default(cls) -> "JointLimits":
        # Physical motor-angle ranges, evidence B (user-supplied official data).
        return cls(lower=(0.0,) * 6, upper=(340.0, 180.0, 163.0, 335.0, 220.0, 335.0))

    def as_pairs(self) -> List[Tuple[float, float]]:
        return list(zip(self.lower, self.upper))


@dataclass(frozen=True)
class Pose:
    """Cartesian target as understood by the official API.

    Units: millimetres and degrees (evidence C: historical client docs say
    "毫米或其他，根据实际设定" — the unit is therefore still to be verified
    on the real robot, evidence E for anything beyond mm).
    """

    x: float
    y: float
    z: float
    rx: float = 90.0
    ry: float = 0.0
    rz: float = 180.0
    order: str = "zyx"

    def to_legacy_params(self, speed_ratio: Optional[float] = None) -> List[Any]:
        """``[x, y, z, r1, r2, r3, order]`` plus ``speed_ratio`` for move_xyz_rotation.

        Official doc (tid=1217, B): move_xyz_rotation takes 8 params (last one
        speed_ratio 0..1); move_linear_xyz_rotation takes 7 (no speed).
        """
        params: List[Any] = [self.x, self.y, self.z, self.rx, self.ry, self.rz, self.order]
        if speed_ratio is not None:
            params.append(speed_ratio)
        return params


@dataclass(frozen=True)
class MotionAck:
    """Reply to a motion command.

    IMPORTANT: the official server replies *before or while* the motion runs
    and returns an *estimated duration*. An ack is NOT evidence that the
    robot reached the target — use actual feedback (TrajectoryMonitor).
    """

    action: str
    estimated_duration_s: Optional[float]
    raw: Any = None
    request_id: Optional[str] = None


_counter = itertools.count(1)
_counter_lock = threading.Lock()


def new_request_id(prefix: str = "req") -> str:
    with _counter_lock:
        n = next(_counter)
    return f"{prefix}-{os.getpid()}-{int(time.time() * 1000)}-{n}"


@dataclass
class CommandEnvelope:
    action: str
    params: Any = None
    request_id: str = field(default_factory=new_request_id)
    options: Dict[str, Any] = field(default_factory=dict)

    def to_wire(self, mode: ProtocolMode, protocol_version: str) -> Dict[str, Any]:
        if mode is ProtocolMode.LEGACY:
            # Exactly the official shape. The official SDK V1.2.1 omits "params"
            # for parameterless commands (e.g. {"action": "get_motor_angles"}),
            # evidence B. Do not add fields here.
            if self.params is None:
                return {"action": self.action}
            return {"action": self.action, "params": self.params}
        msg: Dict[str, Any] = {
            "protocol_version": protocol_version,
            "request_id": self.request_id,
            "action": self.action,
            "params": self.params,
        }
        if self.options:
            msg["options"] = self.options
        return msg


def as_float_list(values: Sequence[Any]) -> List[float]:
    return [float(v) for v in values]
