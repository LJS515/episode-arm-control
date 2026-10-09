"""Client-side parameter validation.

Everything here runs *before* a command is serialised, so a malformed
request never reaches the official upper computer / CAN bus. The official
server (historical source, evidence C) performs no type or range checking
on ``angle_mode`` parameters.
"""

from __future__ import annotations

import math
import numbers
from typing import Any, List, Optional, Sequence

from .exceptions import (
    JointCountError,
    JointLimitError,
    NonFiniteValueError,
    ParameterTypeError,
    SpeedRatioError,
    ValidationError,
)
from .models import NUM_JOINTS, JointLimits, Pose

#: The official server supports only these two orders (API doc tid=1217, evidence B).
ALLOWED_EULER_ORDERS = frozenset({"zyx", "xyz"})


def _is_real_number(v: Any) -> bool:
    # bool is a subclass of int: True would silently become 1.0 degree.
    return isinstance(v, numbers.Real) and not isinstance(v, bool)


def validate_number(value: Any, name: str) -> float:
    if not _is_real_number(value):
        raise ParameterTypeError(f"{name} must be a real number", details={"got": type(value).__name__})
    f = float(value)
    if not math.isfinite(f):
        raise NonFiniteValueError(f"{name} must be finite", details={"value": repr(value)})
    return f


def validate_joint_vector(
    values: Any,
    limits: Optional[JointLimits] = None,
    *,
    margin_deg: float = 0.0,
    name: str = "angles",
) -> List[float]:
    """Validate a 6-element joint vector in motor degrees.

    ``margin_deg`` shrinks the allowed range on both sides (soft limit).
    """
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise ParameterTypeError(f"{name} must be a sequence of 6 numbers", details={"got": type(values).__name__})
    if len(values) != NUM_JOINTS:
        raise JointCountError(f"{name} must have exactly {NUM_JOINTS} elements", details={"got": len(values)})
    out = [validate_number(v, f"{name}[{i}]") for i, v in enumerate(values)]
    if limits is not None:
        violations = []
        for i, (v, lo, hi) in enumerate(zip(out, limits.lower, limits.upper)):
            if v < lo + margin_deg or v > hi - margin_deg:
                violations.append({"joint": i + 1, "value": v, "min": lo + margin_deg, "max": hi - margin_deg})
        if violations:
            raise JointLimitError("joint target outside allowed range", details={"violations": violations})
    return out


def validate_speed_ratio(value: Any, *, minimum: float = 0.01, maximum: float = 1.0) -> float:
    try:
        f = validate_number(value, "speed_ratio")
    except ValidationError as exc:
        raise SpeedRatioError(exc.message, details=exc.details) from exc
    if not (minimum <= f <= maximum):
        raise SpeedRatioError(
            "speed_ratio out of range", details={"value": f, "min": minimum, "max": maximum}
        )
    return f


def validate_euler_order(order: Any) -> str:
    if not isinstance(order, str) or order.lower() not in ALLOWED_EULER_ORDERS:
        raise ParameterTypeError("invalid euler order", details={"got": order, "allowed": sorted(ALLOWED_EULER_ORDERS)})
    return order.lower()


def validate_pose(pose: Pose, *, max_reach_mm: Optional[float] = None, shoulder_height_mm: float = 166.0) -> Pose:
    """Validate a Cartesian target.

    ``max_reach_mm`` is a coarse sanity bound (distance from the shoulder
    point), NOT a reachability guarantee — only the official IK decides
    reachability. The 166 mm shoulder height is the historical DH ``d1``
    (evidence C).
    """
    x = validate_number(pose.x, "x")
    y = validate_number(pose.y, "y")
    z = validate_number(pose.z, "z")
    for n in ("rx", "ry", "rz"):
        validate_number(getattr(pose, n), n)
    validate_euler_order(pose.order)
    if max_reach_mm is not None:
        r = math.sqrt(x * x + y * y + (z - shoulder_height_mm) ** 2)
        if r > max_reach_mm:
            raise JointLimitError(
                "Cartesian target beyond coarse reach bound",
                details={"distance_mm": round(r, 1), "max_reach_mm": max_reach_mm},
            )
    return pose


def validate_servo_angle(value: Any, *, minimum: int = 0, maximum: int = 110) -> int:
    """Official range: integer 0..110 (API doc tid=1217, evidence B).

    The server silently clamps out-of-range values; we reject them instead so a
    caller's mistake is visible. Larger angle = gripper closes; the doc advises
    95-105 deg max when empty and not to stall the servo for more than 1 min.
    """
    f = validate_number(value, "servo_angle")
    if not (minimum <= f <= maximum):
        raise ParameterTypeError("servo angle out of range", details={"value": f, "min": minimum, "max": maximum})
    return int(round(f))


def validate_bool_flag(value: Any, name: str) -> int:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int) and value in (0, 1):
        return value
    raise ParameterTypeError(f"{name} must be bool or 0/1", details={"got": value})


def validate_joint_reply(value: Any) -> List[Optional[float]]:
    """Validate a ``get_motor_angles`` reply.

    Individual entries may be ``None``: the official FAQ (evidence B, forum
    tid=1932) documents that joints with broken CAN communication show
    ``None``. We keep the per-joint None instead of raising, so the state
    cache can mark the reading invalid with a precise reason.
    """
    if not isinstance(value, (list, tuple)) or len(value) != NUM_JOINTS:
        raise ParameterTypeError("angle reply must be a list of 6 values", details={"got": value})
    out: List[Optional[float]] = []
    for i, v in enumerate(value):
        if v is None:
            out.append(None)
            continue
        out.append(validate_number(v, f"reply[{i}]"))
    return out
