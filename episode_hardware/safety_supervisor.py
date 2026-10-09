"""Safety supervisor: the single place that says "yes, this motion may start".

Responsibilities
----------------
Pre-motion checks  (``check_motion_request``)
    state is READY, robot homed, no latched fault, no ESTOP, feedback fresh
    and complete, target within (soft) joint limits, speed ratio in range,
    optional maximum per-command joint step.
Runtime checks     (``evaluate``)
    consecutive read failures, stale data during motion, measured position
    outside the physical range.
Post-motion checks (``on_motion_result``)
    motion timeout / final-error verdicts from the TrajectoryMonitor.
Fault handling
    faults are *latched* (sticky); ``clear_fault`` requires an operator
    acknowledgement and that the cause is gone (guard on CLEAR_FAULT).

This is a software layer on top of the official stack. It cannot replace
mechanical limits, the drivers' own protection or a hardware E-stop.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from episode_sdk.exceptions import (
    EStopActiveError,
    FaultLatchedError,
    JointLimitError,
    NotHomedError,
    StaleStateError,
    StateRejectedError,
)
from episode_sdk.models import JointLimits
from episode_sdk.validation import validate_joint_vector, validate_speed_ratio

from .clock import Clock, monotonic
from .state_cache import JointStateCache, JointStateSnapshot
from .state_machine import Event, RobotState, RobotStateMachine
from .trajectory_monitor import MonitorResult, MonitorStatus

log = logging.getLogger("episode_hardware.safety")


@dataclass(frozen=True)
class SafetyConfig:
    limits: JointLimits = field(default_factory=JointLimits.episode_default)
    soft_margin_deg: float = 2.0
    speed_ratio_min: float = 0.01
    speed_ratio_max: float = 1.0
    #: commissioning default; raise only after hardware validation
    speed_ratio_default_cap: float = 0.3
    max_consecutive_read_failures: int = 3
    max_joint_step_deg: Optional[float] = None
    position_out_of_range_tolerance_deg: float = 3.0
    final_tolerance_deg: float = 1.0
    settle_time_s: float = 0.3
    motion_timeout_factor: float = 1.5
    motion_timeout_margin_s: float = 2.0
    stop_timeout_s: float = 2.0
    homing_timeout_s: float = 60.0  # calibration tool timeout per official FAQ (B, tid=1260)
    require_homed: bool = True

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "SafetyConfig":
        kw: Dict[str, Any] = {}
        lim = d.get("joint_limits_deg")
        if lim:
            kw["limits"] = JointLimits(tuple(float(p[0]) for p in lim), tuple(float(p[1]) for p in lim))
        for name in cls.__dataclass_fields__:
            if name != "limits" and name in d:
                kw[name] = d[name]
        return cls(**kw)


@dataclass
class FaultRecord:
    code: str
    detail: str
    t: float
    cleared_at: Optional[float] = None
    cleared_by: Optional[str] = None


class SafetySupervisor:
    def __init__(self, config: SafetyConfig, state_machine: RobotStateMachine, cache: JointStateCache,
                 *, clock: Clock = monotonic) -> None:
        self.config = config
        self.sm = state_machine
        self.cache = cache
        self.clock = clock
        self._lock = threading.RLock()
        self.faults: List[FaultRecord] = []
        self._operator_ack = False
        self.sm.add_guard(Event.CLEAR_FAULT, self._guard_clear_fault)
        self.sm.add_guard(Event.MOTION_START, self._guard_motion_start)

    # ------------------------------------------------------------------ queries
    @property
    def active_faults(self) -> List[FaultRecord]:
        with self._lock:
            return [f for f in self.faults if f.cleared_at is None]

    @property
    def fault_latched(self) -> bool:
        return bool(self.active_faults) or self.sm.state is RobotState.FAULT

    # ------------------------------------------------------------------ pre-motion
    def check_motion_request(self, target_motor_deg: Sequence[Any], speed_ratio: Any) -> Tuple[List[float], float]:
        state = self.sm.state
        if state is RobotState.ESTOP:
            raise EStopActiveError("motion rejected: ESTOP active")
        if state is RobotState.FAULT or self.active_faults:
            raise FaultLatchedError("motion rejected: fault latched", details={"faults": [f.code for f in self.active_faults]})
        if self.config.require_homed and not self.sm.homed:
            raise NotHomedError("motion rejected: robot not homed")
        if state is not RobotState.READY:
            raise StateRejectedError(f"motion rejected in state {state.value}", details={"state": state.value})
        snap = self.cache.snapshot()
        self._require_fresh(snap)
        q = validate_joint_vector(target_motor_deg, self.config.limits, margin_deg=self.config.soft_margin_deg)
        s = validate_speed_ratio(speed_ratio, minimum=self.config.speed_ratio_min, maximum=self.config.speed_ratio_max)
        if self.config.max_joint_step_deg is not None:
            cur = snap.require_positions()
            steps = [abs(a - b) for a, b in zip(q, cur)]
            if max(steps) > self.config.max_joint_step_deg:
                raise JointLimitError("joint step exceeds configured maximum",
                                      details={"max_step": self.config.max_joint_step_deg, "steps": steps})
        return q, s

    def _require_fresh(self, snap: JointStateSnapshot) -> None:
        if not snap.valid:
            raise StaleStateError("feedback incomplete", details={"joint_valid": snap.joint_valid, "error": snap.last_error})
        if snap.stale:
            raise StaleStateError("feedback stale", details={"age_s": snap.age_s, "limit_s": self.cache.stale_after_s})

    def _guard_motion_start(self, _s: RobotState, _e: Event) -> Tuple[bool, str]:
        if self.active_faults:
            return False, "fault latched"
        if self.config.require_homed and not self.sm.homed:
            return False, "not homed"
        return True, ""

    # ------------------------------------------------------------------ runtime
    def evaluate(self) -> Optional[FaultRecord]:
        """Run periodic checks; returns a newly latched fault, if any."""
        snap = self.cache.snapshot()
        state = self.sm.state
        if state in (RobotState.UNCONFIGURED, RobotState.ESTOP):
            return None
        if snap.consecutive_failures >= self.config.max_consecutive_read_failures:
            return self.latch_fault("READ_FAILURES",
                                    f"{snap.consecutive_failures} consecutive read failures ({snap.last_error})")
        if state in (RobotState.MOVING, RobotState.STOPPING, RobotState.HOMING) and snap.stale and snap.stamp is not None:
            return self.latch_fault("STALE_DURING_MOTION", f"feedback age {snap.age_s}s during {state.value}")
        if snap.valid and not snap.stale:
            tol = self.config.position_out_of_range_tolerance_deg
            for i, (p, lo, hi) in enumerate(zip(snap.positions_deg, self.config.limits.lower, self.config.limits.upper)):
                if p is not None and (p < lo - tol or p > hi + tol):
                    return self.latch_fault("POSITION_OUT_OF_RANGE", f"joint {i+1} at {p:.2f} outside [{lo}, {hi}]")
        return None

    def on_motion_result(self, result: MonitorResult) -> Optional[FaultRecord]:
        if result.status is MonitorStatus.TIMED_OUT:
            return self.latch_fault("MOTION_TIMEOUT", result.reason)
        if result.status is MonitorStatus.FINAL_ERROR_EXCEEDED:
            return self.latch_fault("FINAL_ERROR_EXCEEDED", f"{result.reason}; error={result.final_error_deg}")
        if result.status is MonitorStatus.TRACKING_ERROR:
            return self.latch_fault("TRACKING_ERROR", result.reason)
        return None

    # ------------------------------------------------------------------ faults
    def latch_fault(self, code: str, detail: str = "") -> FaultRecord:
        with self._lock:
            for f in self.active_faults:
                if f.code == code:
                    return f  # already latched; keep first occurrence
            rec = FaultRecord(code=code, detail=detail, t=self.clock())
            self.faults.append(rec)
            self._operator_ack = False
        log.error("FAULT latched: %s %s", code, detail)
        if self.sm.state is not RobotState.ESTOP:
            self.sm.try_dispatch(Event.FAULT, f"{code}: {detail}")
        return rec

    def acknowledge(self) -> None:
        with self._lock:
            self._operator_ack = True

    def clear_fault(self, operator: str = "operator") -> RobotState:
        """Operator-initiated fault clearing. Raises if the guard rejects."""
        self.acknowledge()
        state = self.sm.dispatch(Event.CLEAR_FAULT, f"cleared by {operator}")
        with self._lock:
            now = self.clock()
            for f in self.faults:
                if f.cleared_at is None:
                    f.cleared_at, f.cleared_by = now, operator
            self._operator_ack = False
        return state

    def _guard_clear_fault(self, _s: RobotState, _e: Event) -> Tuple[bool, str]:
        if not self._operator_ack:
            return False, "operator acknowledgement required"
        snap = self.cache.snapshot()
        if snap.consecutive_failures > 0:
            return False, f"read failures still present ({snap.consecutive_failures})"
        if not snap.usable:
            return False, "feedback not fresh/complete"
        return True, ""

    def snapshot(self) -> Dict[str, Any]:
        return {
            "state": self.sm.state.value,
            "homed": self.sm.homed,
            "fault_reason": self.sm.fault_reason,
            "active_faults": [{"code": f.code, "detail": f.detail, "t": f.t} for f in self.active_faults],
        }
