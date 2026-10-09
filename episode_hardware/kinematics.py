"""Reference forward / inverse kinematics (pure Python, no numpy).

STATUS: reference implementation for simulation and validation.
It is NOT the official IK (the official IK is a compiled library, evidence B)
and must not be used to drive the real robot without cross-checking against
``get_T`` / ``get_pose`` on hardware.

DH parameters are taken from the historical public client
(``EpisodeAPP.EPDobot``, repo ``enpeizhao/episode_robot`` @9ede794,
evidence C). Standard DH convention, millimetres::

    i   d     a     alpha
    1   166   55    +pi/2
    2   0     200   0
    3   0     56    +pi/2
    4   192   0     -pi/2
    5   0     0     +pi/2
    6   55    0     0
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .joint_mapping import VENDOR_DH, JointMapping

Matrix = List[List[float]]


@dataclass(frozen=True)
class DHLink:
    d: float
    a: float
    alpha: float


EPISODE_DH: Tuple[DHLink, ...] = (
    DHLink(166.0, 55.0, math.pi / 2),
    DHLink(0.0, 200.0, 0.0),
    DHLink(0.0, 56.0, math.pi / 2),
    DHLink(192.0, 0.0, -math.pi / 2),
    DHLink(0.0, 0.0, math.pi / 2),
    DHLink(55.0, 0.0, 0.0),
)


# --------------------------------------------------------------------------- small linear algebra
def identity() -> Matrix:
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def matmul(a: Matrix, b: Matrix) -> Matrix:
    n, m, p = len(a), len(b), len(b[0])
    return [[sum(a[i][k] * b[k][j] for k in range(m)) for j in range(p)] for i in range(n)]


def dh_transform(theta: float, link: DHLink) -> Matrix:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(link.alpha), math.sin(link.alpha)
    return [
        [ct, -st * ca, st * sa, link.a * ct],
        [st, ct * ca, -ct * sa, link.a * st],
        [0.0, sa, ca, link.d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_x(deg: float) -> Matrix:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(deg: float) -> Matrix:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(deg: float) -> Matrix:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


_ROT = {"x": rot_x, "y": rot_y, "z": rot_z}


def euler_to_matrix(angles_deg: Sequence[float], order: str = "zyx") -> Matrix:
    """``order='zyx'`` with ``[a, b, c]`` -> ``Rz(a) @ Ry(b) @ Rx(c)``.

    This mirrors ``calculate_ee2base_T`` in the historical client
    (evidence C). How the compiled server interprets the order string is
    evidence D/E — verify with ``get_T`` on hardware.
    """
    r = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    for axis, ang in zip(order.lower(), angles_deg):
        r = matmul(r, _ROT[axis](ang))
    return r


def solve_linear(a: List[List[float]], b: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError("singular matrix")
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


# --------------------------------------------------------------------------- FK / IK
def fk_model(q_model_rad: Sequence[float], links: Sequence[DHLink] = EPISODE_DH) -> Matrix:
    t = identity()
    for q, link in zip(q_model_rad, links):
        t = matmul(t, dh_transform(q, link))
    return t


def fk_motor_deg(motor_deg: Sequence[float], mapping: JointMapping = VENDOR_DH) -> Matrix:
    q = mapping.motor_to_model_rad(motor_deg)
    return fk_model([float(v) for v in q])  # type: ignore[arg-type]


def rotation_error(current: Matrix, target: Matrix) -> List[float]:
    """Axis-angle vector ``w`` with ``exp([w]x) @ current == target`` (world frame).

    Uses the SO(3) log map. The popular shortcut
    ``0.5 * sum(cross(current_axis_k, target_axis_k))`` is *zero* for a 180 deg
    error, so an IK built on it can "converge" with the tool flipped. The SO(3)
    log map below handles the near-pi case explicitly.
    """
    r = [[sum(target[i][k] * current[j][k] for k in range(3)) for j in range(3)] for i in range(3)]  # Rd @ Rc^T
    cos_t = max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0))
    theta = math.acos(cos_t)
    if theta < 1e-9:
        return [0.0, 0.0, 0.0]
    if theta < math.pi - 1e-4:
        k = theta / (2.0 * math.sin(theta))
        return [k * (r[2][1] - r[1][2]), k * (r[0][2] - r[2][0]), k * (r[1][0] - r[0][1])]
    # near pi: axis from the diagonal, sign from the off-diagonal terms
    ax = [math.sqrt(max(0.0, (r[i][i] + 1.0) / 2.0)) for i in range(3)]
    i = max(range(3), key=lambda n: ax[n])
    for j in range(3):
        if j != i:
            ax[j] = math.copysign(ax[j], r[i][j] + r[j][i])
    return [theta * a for a in ax]


def _pose_error(t: Matrix, target_pos: Sequence[float], target_rot: Matrix, rot_weight: float) -> List[float]:
    ep = [target_pos[i] - t[i][3] for i in range(3)]
    er = rotation_error([row[:3] for row in t[:3]], target_rot)
    return ep + [rot_weight * e for e in er]


def ik_dls(
    target_pos_mm: Sequence[float],
    target_rot: Matrix,
    q0_model_rad: Sequence[float],
    *,
    limits_rad: Optional[Sequence[Tuple[float, float]]] = None,
    max_iter: int = 300,
    damping: float = 5.0,
    rot_weight: float = 100.0,
    tol_mm: float = 0.5,
) -> Optional[List[float]]:
    """Damped-least-squares numerical IK. Returns model radians or None."""
    q = [float(v) for v in q0_model_rad]
    eps = 1e-6
    for _ in range(max_iter):
        t = fk_model(q)
        e = _pose_error(t, target_pos_mm, target_rot, rot_weight)
        if math.sqrt(sum(v * v for v in e)) < tol_mm:
            if limits_rad and any(not (lo - 1e-6 <= v <= hi + 1e-6) for v, (lo, hi) in zip(q, limits_rad)):
                return None
            return q
        # numerical Jacobian of the error w.r.t. q (6x6)
        jac = [[0.0] * 6 for _ in range(6)]
        for j in range(6):
            dq = q[:]
            dq[j] += eps
            e2 = _pose_error(fk_model(dq), target_pos_mm, target_rot, rot_weight)
            for i in range(6):
                jac[i][j] = -(e2[i] - e[i]) / eps
        # dq = J^T (J J^T + lambda^2 I)^-1 e
        jjt = [[sum(jac[i][k] * jac[j][k] for k in range(6)) + (damping ** 2 if i == j else 0.0) for j in range(6)] for i in range(6)]
        try:
            y = solve_linear(jjt, e)
        except ValueError:
            return None
        step = [sum(jac[i][j] * y[i] for i in range(6)) for j in range(6)]
        q = [qi + si for qi, si in zip(q, step)]
        if limits_rad:
            q = [min(max(v, lo), hi) for v, (lo, hi) in zip(q, limits_rad)]
    return None


def ik_motor_deg(
    pos_mm: Sequence[float],
    euler_deg: Sequence[float],
    order: str,
    seed_motor_deg: Sequence[float],
    mapping: JointMapping = VENDOR_DH,
) -> Optional[List[float]]:
    """Multi-start DLS: the caller's seed first, then fixed wrist-flipped seeds.

    A local method started from the "vertical" posture can stall against a
    joint limit (J4/J6 near their 0 deg end). Trying the seed with the wrist
    rotated by ~180 deg finds the solution in that case. Deterministic.
    """
    target_rot = euler_to_matrix(euler_deg, order)
    limits = mapping.model_limits_rad()
    seeds = [list(seed_motor_deg)] + [list(s) for s in _EXTRA_SEEDS_MOTOR_DEG]
    for seed in seeds:
        q0 = mapping.motor_to_model_rad(seed)
        sol = ik_dls(pos_mm, target_rot, [float(v) for v in q0], limits_rad=limits)  # type: ignore[arg-type]
        if sol is None:
            continue
        motor = mapping.model_rad_to_motor(sol)
        if mapping.within_motor_limits(motor):
            return motor
    return None


#: wrist-flipped variants of the vertical posture (motor degrees)
_EXTRA_SEEDS_MOTOR_DEG = (
    (180.0, 90.0, 83.0, 200.0, 110.0, 200.0),
    (180.0, 60.0, 120.0, 170.0, 60.0, 170.0),
    (180.0, 120.0, 40.0, 170.0, 170.0, 170.0),
)
