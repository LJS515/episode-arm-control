"""Simulated Episode 1 arm with fault injection.

Everything produced by this module is **Synthetic / Mock Data**. The motion
model (trapezoid-free linear interpolation at a configurable per-joint
speed) is deliberately simple; it exists to exercise the control/safety
software, not to predict the real robot's dynamics.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set

from episode_sdk.models import NUM_JOINTS, JointLimits, Pose

from . import kinematics
from .clock import Clock, monotonic
from .interfaces import HardwareCapabilities, HardwareCommandError, HardwareReadError, RobotHardware
from .joint_mapping import VENDOR_DH, VERTICAL_MOTOR_DEG

#: Synthetic default joint speeds in motor deg/s at speed_ratio=1.0.
#: NOT measured. Placeholder until real data exists.
DEFAULT_MAX_SPEED_DEG_S = (60.0, 45.0, 60.0, 90.0, 90.0, 120.0)


@dataclass
class FaultInjection:
    """Knobs for fault-injection tests. All default to "healthy"."""

    #: next N reads raise HardwareReadError
    fail_next_reads: int = 0
    #: every read raises (persistent communication loss)
    fail_all_reads: bool = False
    #: joints (1-based) that report None, like the official FAQ case
    none_joints: Set[int] = field(default_factory=set)
    #: joints (1-based) that never move (stalled / slipped belt)
    stuck_joints: Set[int] = field(default_factory=set)
    #: per-joint constant error added to the *final* position (e.g. belt slip)
    final_offset_deg: Dict[int, float] = field(default_factory=dict)
    #: return the last value forever (frozen feedback / stale data)
    freeze_feedback: bool = False
    #: commands raise HardwareCommandError
    reject_commands: bool = False
    #: IK always fails
    ik_fail: bool = False
    #: multiply real motion duration (robot slower than estimate)
    slowdown_factor: float = 1.0


@dataclass
class _Segment:
    start: List[float]
    goal: List[float]
    t0: float
    duration: float


class SimulatedArm:
    def __init__(
        self,
        *,
        clock: Clock = monotonic,
        initial_motor_deg: Sequence[float] = (0.0,) * NUM_JOINTS,
        max_speed_deg_s: Sequence[float] = DEFAULT_MAX_SPEED_DEG_S,
        limits: JointLimits = JointLimits.episode_default(),
        homing_duration_s: float = 3.0,
        homed: bool = False,
    ) -> None:
        self.clock = clock
        self.limits = limits
        self.max_speed = list(max_speed_deg_s)
        self.homing_duration_s = homing_duration_s
        self.faults = FaultInjection()
        self._lock = threading.RLock()
        self._pos = [float(v) for v in initial_motor_deg]
        self._seg: Optional[_Segment] = None
        self._last_read: Optional[List[Optional[float]]] = None
        self.homed = homed
        self.estopped = False
        self.free_mode = False
        self.gripper_closed = False
        self.servo_angle: Optional[float] = None
        self.command_log: List[Dict] = []

    # ------------------------------------------------------------------ motion model
    def _advance(self) -> None:
        if self._seg is None:
            return
        s = self._seg
        now = self.clock()
        frac = 1.0 if s.duration <= 0 else min(1.0, (now - s.t0) / s.duration)
        pos = []
        for j in range(NUM_JOINTS):
            if (j + 1) in self.faults.stuck_joints:
                pos.append(s.start[j])
                continue
            v = s.start[j] + (s.goal[j] - s.start[j]) * frac
            if frac >= 1.0:
                v += self.faults.final_offset_deg.get(j + 1, 0.0)
            pos.append(v)
        self._pos = pos
        if frac >= 1.0:
            self._seg = None

    def is_moving(self) -> bool:
        with self._lock:
            self._advance()
            return self._seg is not None

    def true_positions(self) -> List[float]:
        with self._lock:
            self._advance()
            return list(self._pos)

    def estimate_duration(self, goal: Sequence[float], speed_ratio: float) -> float:
        cur = self.true_positions()
        t = max(abs(g - c) / v for g, c, v in zip(goal, cur, self.max_speed))
        return t / max(speed_ratio, 1e-6)

    def start_move(self, goal: Sequence[float], speed_ratio: float) -> float:
        with self._lock:
            if self.estopped:
                raise HardwareCommandError("simulated drive is in ESTOP")
            if self.faults.reject_commands:
                raise HardwareCommandError("injected command rejection")
            self._advance()
            est = self.estimate_duration(goal, speed_ratio)
            real = est * self.faults.slowdown_factor
            self._seg = _Segment(list(self._pos), [float(g) for g in goal], self.clock(), real)
            self.command_log.append({"t": self.clock(), "cmd": "move", "goal": list(goal), "speed_ratio": speed_ratio})
            return est

    def stop(self) -> None:
        with self._lock:
            self._advance()
            self._seg = None  # mock: instantaneous stop at current position
            self.command_log.append({"t": self.clock(), "cmd": "stop"})

    def emergency_stop(self) -> None:
        with self._lock:
            self.stop()
            self.estopped = True
            self.command_log.append({"t": self.clock(), "cmd": "estop"})

    def release_estop(self) -> None:
        with self._lock:
            self.estopped = False
            # conservative: the mock forgets homing after ESTOP
            self.homed = False

    def home(self) -> float:
        with self._lock:
            if self.estopped:
                raise HardwareCommandError("cannot home during ESTOP")
            self._advance()
            self._seg = _Segment(list(self._pos), list(VERTICAL_MOTOR_DEG), self.clock(), self.homing_duration_s)
            self.homed = True  # becomes meaningful once the segment completes
            self.command_log.append({"t": self.clock(), "cmd": "home"})
            return self.homing_duration_s

    # ------------------------------------------------------------------ feedback
    def read(self) -> List[Optional[float]]:
        with self._lock:
            if self.faults.fail_all_reads:
                raise HardwareReadError("injected persistent read failure")
            if self.faults.fail_next_reads > 0:
                self.faults.fail_next_reads -= 1
                raise HardwareReadError("injected transient read failure")
            if self.faults.freeze_feedback and self._last_read is not None:
                return list(self._last_read)
            pos: List[Optional[float]] = list(self.true_positions())
            for j in self.faults.none_joints:
                pos[j - 1] = None
            self._last_read = list(pos)
            return pos

    def solve_ik(self, pose: Pose) -> Optional[List[float]]:
        if self.faults.ik_fail:
            return None
        return kinematics.ik_motor_deg([pose.x, pose.y, pose.z], [pose.rx, pose.ry, pose.rz], pose.order,
                                       self.true_positions(), VENDOR_DH)


class MockHardwareAdapter(RobotHardware):
    def __init__(self, arm: Optional[SimulatedArm] = None, *, clock: Clock = monotonic):
        self.arm = arm or SimulatedArm(clock=clock)
        self._connected = False

    @property
    def capabilities(self) -> HardwareCapabilities:
        return HardwareCapabilities(soft_stop=True, streaming=False, reports_completion=False,
                                    software_estop_release=True, description="in-process simulation (Mock)")

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def _require(self) -> None:
        if not self._connected:
            raise HardwareCommandError("mock adapter not connected")

    def read_joint_positions(self) -> List[Optional[float]]:
        self._require()
        return self.arm.read()

    def command_joint_positions(self, motor_deg: Sequence[float], speed_ratio: float) -> Optional[float]:
        self._require()
        if self.arm.free_mode:
            raise HardwareCommandError("free mode active")
        return self.arm.start_move(motor_deg, speed_ratio)

    def move_pose(self, pose: Pose, linear: bool) -> Optional[float]:
        self._require()
        sol = self.arm.solve_ik(pose)
        if sol is None:
            from episode_sdk.exceptions import NoSolutionError

            raise NoSolutionError("mock IK found no solution")
        return self.arm.start_move(sol, 1.0)

    def home(self) -> Optional[float]:
        self._require()
        return self.arm.home()

    def soft_stop(self) -> bool:
        self._require()
        self.arm.stop()
        return True

    def emergency_stop(self) -> None:
        # deliberately does not require "connected": estop must always try
        self.arm.emergency_stop()

    def release_emergency_stop(self) -> bool:
        self.arm.release_estop()
        return True

    def set_free_mode(self, enable: bool) -> None:
        self._require()
        self.arm.stop()
        self.arm.free_mode = bool(enable)

    def set_gripper(self, closed: bool) -> None:
        self._require()
        self.arm.gripper_closed = bool(closed)

    def set_servo_gripper(self, angle_deg: float) -> None:
        self._require()
        if not math.isfinite(angle_deg):
            raise HardwareCommandError("non-finite servo angle")
        self.arm.servo_angle = float(angle_deg)
