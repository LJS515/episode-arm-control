"""Joint coordinate mapping: motor/physical degrees  <->  model angles.

The official stack uses at least three joint coordinate systems:

1. **Motor (physical) degrees** — what ``angle_mode`` takes and
   ``get_motor_angles`` returns. Range ``[0, max]`` per joint, measured from
   the collision-homing zero. Evidence B.
2. **DH model angles** used by the historical Python IK/FK
   (``calculate_T_based_on_degrees``), evidence C::

       q1 = m1 - 180     q2 = 180 - m2     q3 = 83 - m3
       q4 = m4 - 30      q5 = 110 - m5     q6 = m6 - 30

3. **ROS model angles** (URDF joint positions): ``model = motor - offset``
   with offsets ``[180, 90, 83, 30, 110, 30]`` and **all signs +1**
   (evidence B: official ``episode_controller/robot_interface_node.py``,
   ``publish_joint_states``; the official URDF limits, e.g. J3 [-83, 80] deg,
   equal the motor ranges shifted by the offsets only with sign +1). The
   offsets equal the post-homing "vertical" posture, so vertical = URDF zero.
   An earlier version of this project guessed the DH signs [+,-,-,+,-,+] for
   the ROS model (evidence E) — that guess was wrong for J2, J3, J5.

General form used here, per joint ``i``::

    model_deg = sign_i * (motor_deg - offset_i)
    motor_deg = offset_i + model_deg / sign_i
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from episode_sdk.models import NUM_JOINTS, JointLimits


@dataclass(frozen=True)
class JointMapping:
    name: str
    signs: Tuple[int, ...]
    offsets_deg: Tuple[float, ...]
    motor_limits: JointLimits = field(default_factory=JointLimits.episode_default)
    evidence: str = "E"
    notes: str = ""

    def __post_init__(self) -> None:
        if len(self.signs) != NUM_JOINTS or len(self.offsets_deg) != NUM_JOINTS:
            raise ValueError("mapping needs 6 signs and 6 offsets")
        if any(s not in (1, -1) for s in self.signs):
            raise ValueError("signs must be +1 or -1")

    # ------------------------------------------------------------------ scalar/vector conversions
    def motor_to_model_deg(self, motor: Sequence[Optional[float]]) -> List[Optional[float]]:
        self._check_len(motor)
        return [None if m is None else s * (float(m) - o) for m, s, o in zip(motor, self.signs, self.offsets_deg)]

    def model_to_motor_deg(self, model: Sequence[float]) -> List[float]:
        self._check_len(model)
        return [o + float(q) * s for q, s, o in zip(model, self.signs, self.offsets_deg)]

    def motor_to_model_rad(self, motor: Sequence[Optional[float]]) -> List[Optional[float]]:
        return [None if q is None else math.radians(q) for q in self.motor_to_model_deg(motor)]

    def model_rad_to_motor(self, model_rad: Sequence[float]) -> List[float]:
        return self.model_to_motor_deg([math.degrees(q) for q in model_rad])

    def model_velocity_from_motor(self, motor_vel_deg_s: Sequence[float]) -> List[float]:
        # offsets do not affect velocity, only the sign does
        self._check_len(motor_vel_deg_s)
        return [s * float(v) for v, s in zip(motor_vel_deg_s, self.signs)]

    # ------------------------------------------------------------------ limits
    def model_limits_deg(self) -> List[Tuple[float, float]]:
        """Motor limits expressed in model degrees (min/max re-ordered for sign)."""
        out = []
        for (lo, hi), s, o in zip(self.motor_limits.as_pairs(), self.signs, self.offsets_deg):
            a, b = s * (lo - o), s * (hi - o)
            out.append((min(a, b), max(a, b)))
        return out

    def model_limits_rad(self) -> List[Tuple[float, float]]:
        return [(math.radians(a), math.radians(b)) for a, b in self.model_limits_deg()]

    def within_motor_limits(self, motor: Sequence[float], margin_deg: float = 0.0) -> bool:
        return all(lo + margin_deg <= m <= hi - margin_deg for m, (lo, hi) in zip(motor, self.motor_limits.as_pairs()))

    @staticmethod
    def _check_len(v: Sequence) -> None:
        if len(v) != NUM_JOINTS:
            raise ValueError(f"expected {NUM_JOINTS} joint values, got {len(v)}")

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_dict(cls, d: Mapping) -> "JointMapping":
        limits = d.get("motor_limits_deg")
        ml = JointLimits(tuple(float(p[0]) for p in limits), tuple(float(p[1]) for p in limits)) if limits else JointLimits.episode_default()
        return cls(
            name=str(d.get("name", "custom")),
            signs=tuple(int(s) for s in d["signs"]),
            offsets_deg=tuple(float(o) for o in d["offsets_deg"]),
            motor_limits=ml,
            evidence=str(d.get("evidence", "E")),
            notes=str(d.get("notes", "")),
        )


#: Historical DH convention of the official Python client (evidence C).
VENDOR_DH = JointMapping(
    name="vendor_dh",
    signs=(1, -1, -1, 1, -1, 1),
    offsets_deg=(180.0, 180.0, 83.0, 30.0, 110.0, 30.0),
    evidence="C+B",
    notes="Formulas from EpisodeAPP (repo @9ede794, C); mapped ranges equal the official DH theta table (tid=1217, B).",
)

#: ROS URDF convention (evidence B, official ROS2 package student_ros V1.0).
ROS_MODEL = JointMapping(
    name="ros_model",
    signs=(1, 1, 1, 1, 1, 1),
    offsets_deg=(180.0, 90.0, 83.0, 30.0, 110.0, 30.0),
    evidence="B",
    notes="joint = motor - offset (official robot_interface_node.py); official URDF limits agree.",
)

#: Joint limits of the official URDF episode1_urdf_1113.urdf in degrees (evidence B).
OFFICIAL_URDF_LIMITS_DEG: Tuple[Tuple[float, float], ...] = (
    (-180.0, 160.0), (-90.0, 90.0), (-83.0, 80.0), (-30.0, 305.0), (-110.0, 110.0), (-30.0, 305.0),
)

PROFILES: Dict[str, JointMapping] = {m.name: m for m in (VENDOR_DH, ROS_MODEL)}

#: The posture the historical server commands right after homing (evidence C).
VERTICAL_MOTOR_DEG: Tuple[float, ...] = (180.0, 90.0, 83.0, 30.0, 110.0, 30.0)


def get_profile(name: str) -> JointMapping:
    try:
        return PROFILES[name]
    except KeyError as exc:
        raise KeyError(f"unknown mapping profile {name!r}; known: {sorted(PROFILES)}") from exc


def check_dh_qlim_consistency(mapping: JointMapping, dh_qlim_deg: Sequence[Tuple[float, float]]) -> List[Dict]:
    """Compare motor limits (mapped to model space) with a model qlim table.

    The historical client defines DH qlim (evidence C)::

        [-180,140] [0,180] [-80,83] [-30,270] [-110,110] [-30,290]

    while the motor ranges mapped into DH space give
    ``[-180,160] [0,180] [-80,83] [-30,305] [-110,110] [-30,305]``.
    J1/J4/J6 differ. The CURRENT official DH table (OFFICIAL_DH_THETA_RANGE_DEG,
    evidence B) matches the mapped motor ranges exactly, so the narrower qlim is
    a property of the historical client only.
    """
    report = []
    for i, ((mlo, mhi), (qlo, qhi)) in enumerate(zip(mapping.model_limits_deg(), dh_qlim_deg)):
        if abs(mlo - qlo) > 1e-6 or abs(mhi - qhi) > 1e-6:
            report.append({"joint": i + 1, "motor_mapped": (mlo, mhi), "qlim": (qlo, qhi)})
    return report


VENDOR_DH_QLIM_DEG: Tuple[Tuple[float, float], ...] = (
    (-180.0, 140.0), (0.0, 180.0), (-80.0, 83.0), (-30.0, 270.0), (-110.0, 110.0), (-30.0, 290.0),
)

#: DH theta ranges from the CURRENT official documentation (forum tid=1217,
#: "2.1.3 DH 参数", evidence B). They equal the motor ranges mapped through
#: VENDOR_DH exactly, which confirms the mapping offsets and signs of VENDOR_DH
#: and shows that the narrower qlim above only exists in the historical client.
OFFICIAL_DH_THETA_RANGE_DEG: Tuple[Tuple[float, float], ...] = (
    (-180.0, 160.0), (0.0, 180.0), (-80.0, 83.0), (-30.0, 305.0), (-110.0, 110.0), (-30.0, 305.0),
)
