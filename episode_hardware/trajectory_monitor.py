"""Trajectory execution monitor: desired vs. actual vs. error.

The official API returns an *estimated duration* and the historical client
just ``sleep()``s for it. That conflates "time has passed" with "the robot
arrived". This monitor decides completion from *measured* feedback:

* ``desired(t)`` — reference. The official internal velocity profile is not
  documented (evidence E), so the reference is a linear interpolation from
  start to goal over the estimated duration. It is used for logging and an
  optional tracking-error bound, never for completion.
* ``actual``     — encoder feedback from the state cache.
* ``error``      — ``goal - actual`` (final error) and ``desired - actual``
  (tracking error).

Completion requires: final error within tolerance on every joint,
continuously for ``settle_time_s``.

Failure verdicts:

* ``TIMED_OUT``            — deadline passed while the robot is still moving
  or feedback is missing.
* ``FINAL_ERROR_EXCEEDED`` — the robot has stopped (stationary for the settle
  time) after the expected duration, but is outside tolerance.
* ``TRACKING_ERROR``       — optional bound on ``|desired - actual|``.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Union

from episode_sdk.models import NUM_JOINTS

from .clock import Clock, monotonic


class MonitorStatus(str, enum.Enum):
    RUNNING = "RUNNING"
    SETTLING = "SETTLING"
    SUCCEEDED = "SUCCEEDED"
    TIMED_OUT = "TIMED_OUT"
    FINAL_ERROR_EXCEEDED = "FINAL_ERROR_EXCEEDED"
    TRACKING_ERROR = "TRACKING_ERROR"
    CANCELLED = "CANCELLED"

    @property
    def terminal(self) -> bool:
        return self in (MonitorStatus.SUCCEEDED, MonitorStatus.TIMED_OUT, MonitorStatus.FINAL_ERROR_EXCEEDED,
                        MonitorStatus.TRACKING_ERROR, MonitorStatus.CANCELLED)

    @property
    def failed(self) -> bool:
        return self.terminal and self not in (MonitorStatus.SUCCEEDED, MonitorStatus.CANCELLED)


@dataclass
class Sample:
    t: float
    desired: List[float]
    actual: List[Optional[float]]
    final_error: List[Optional[float]]
    tracking_error: List[Optional[float]]


@dataclass
class MonitorResult:
    status: MonitorStatus
    elapsed_s: float
    max_final_error_deg: Optional[float]
    final_error_deg: List[Optional[float]]
    samples: int
    invalid_samples: int
    reason: str = ""


class TrajectoryMonitor:
    def __init__(
        self,
        start_deg: Sequence[float],
        goal_deg: Sequence[float],
        *,
        expected_duration_s: Optional[float],
        tolerance_deg: Union[float, Sequence[float]] = 1.0,
        settle_time_s: float = 0.3,
        timeout_s: Optional[float] = None,
        timeout_margin_s: float = 2.0,
        timeout_factor: float = 1.5,
        stationary_eps_deg: float = 0.05,
        max_tracking_error_deg: Optional[float] = None,
        keep_samples: int = 5000,
        clock: Clock = monotonic,
    ) -> None:
        if len(start_deg) != NUM_JOINTS or len(goal_deg) != NUM_JOINTS:
            raise ValueError("start/goal need 6 values")
        self.start = [float(v) for v in start_deg]
        self.goal = [float(v) for v in goal_deg]
        self.expected = max(0.0, float(expected_duration_s or 0.0))
        tol = [float(tolerance_deg)] * NUM_JOINTS if isinstance(tolerance_deg, (int, float)) else [float(t) for t in tolerance_deg]
        if len(tol) != NUM_JOINTS:
            raise ValueError("tolerance needs 1 or 6 values")
        self.tolerance = tol
        self.settle_time_s = float(settle_time_s)
        self.timeout_s = float(timeout_s) if timeout_s is not None else self.expected * timeout_factor + timeout_margin_s
        self.stationary_eps = float(stationary_eps_deg)
        self.max_tracking_error = max_tracking_error_deg
        self.clock = clock
        self.t0 = clock()
        self.status = MonitorStatus.RUNNING
        self.reason = ""
        self.samples: List[Sample] = []
        self.keep_samples = keep_samples
        self.invalid_samples = 0
        self._settle_start: Optional[float] = None
        self._stationary_since: Optional[float] = None
        self._last_actual: Optional[List[float]] = None
        self._last_final_error: List[Optional[float]] = [None] * NUM_JOINTS

    # ------------------------------------------------------------------ reference
    def desired(self, t: float) -> List[float]:
        if self.expected <= 0:
            return list(self.goal)
        frac = min(1.0, max(0.0, (t - self.t0) / self.expected))
        return [s + (g - s) * frac for s, g in zip(self.start, self.goal)]

    # ------------------------------------------------------------------ update
    def update(self, actual: Sequence[Optional[float]], stamp: Optional[float] = None) -> MonitorStatus:
        if self.status.terminal:
            return self.status
        now = self.clock() if stamp is None else float(stamp)
        elapsed = now - self.t0
        des = self.desired(now)
        valid = len(actual) == NUM_JOINTS and all(a is not None for a in actual)
        if not valid:
            self.invalid_samples += 1
            self._settle_start = None
            self._push(now, des, list(actual), [None] * NUM_JOINTS, [None] * NUM_JOINTS)
            if elapsed > self.timeout_s:
                return self._finish(MonitorStatus.TIMED_OUT, "deadline passed without valid feedback")
            return self.status

        act = [float(a) for a in actual]  # type: ignore[arg-type]
        ferr = [g - a for g, a in zip(self.goal, act)]
        terr = [d - a for d, a in zip(des, act)]
        self._last_final_error = list(ferr)
        self._push(now, des, list(act), ferr, terr)

        # stationarity
        if self._last_actual is not None and max(abs(a - b) for a, b in zip(act, self._last_actual)) <= self.stationary_eps:
            if self._stationary_since is None:
                self._stationary_since = now
        else:
            self._stationary_since = None
        self._last_actual = act

        if self.max_tracking_error is not None and elapsed <= self.expected:
            if max(abs(e) for e in terr) > self.max_tracking_error:
                return self._finish(MonitorStatus.TRACKING_ERROR, f"tracking error > {self.max_tracking_error} deg")

        within = all(abs(e) <= tol for e, tol in zip(ferr, self.tolerance))
        if within:
            if self._settle_start is None:
                self._settle_start = now
            if now - self._settle_start >= self.settle_time_s:
                return self._finish(MonitorStatus.SUCCEEDED, "within tolerance and settled")
            self.status = MonitorStatus.SETTLING
        else:
            self._settle_start = None
            self.status = MonitorStatus.RUNNING
            stopped = self._stationary_since is not None and now - self._stationary_since >= self.settle_time_s
            if stopped and elapsed >= self.expected:
                return self._finish(MonitorStatus.FINAL_ERROR_EXCEEDED,
                                    f"stopped outside tolerance, max error {max(abs(e) for e in ferr):.3f} deg")
        if elapsed > self.timeout_s:
            return self._finish(MonitorStatus.TIMED_OUT, f"not settled within {self.timeout_s:.2f}s")
        return self.status

    def cancel(self, reason: str = "cancelled") -> MonitorStatus:
        if not self.status.terminal:
            self._finish(MonitorStatus.CANCELLED, reason)
        return self.status

    # ------------------------------------------------------------------ results
    def _finish(self, status: MonitorStatus, reason: str) -> MonitorStatus:
        self.status = status
        self.reason = reason
        self.finished_at = self.clock()
        return status

    def _push(self, t: float, des: List[float], act: List[Optional[float]], ferr: List[Optional[float]], terr: List[Optional[float]]) -> None:
        self.samples.append(Sample(t, des, act, ferr, terr))
        if len(self.samples) > self.keep_samples:
            del self.samples[: len(self.samples) - self.keep_samples]

    def result(self) -> MonitorResult:
        fe = self._last_final_error
        valid = [abs(e) for e in fe if e is not None]
        return MonitorResult(
            status=self.status,
            elapsed_s=(getattr(self, "finished_at", None) or self.clock()) - self.t0,
            max_final_error_deg=max(valid) if valid else None,
            final_error_deg=list(fe),
            samples=len(self.samples),
            invalid_samples=self.invalid_samples,
            reason=self.reason,
        )

    def to_rows(self) -> List[dict]:
        rows = []
        for s in self.samples:
            row = {"t": round(s.t - self.t0, 6)}
            for j in range(NUM_JOINTS):
                row[f"desired_{j+1}"] = s.desired[j]
                row[f"actual_{j+1}"] = s.actual[j] if j < len(s.actual) else None
                row[f"error_{j+1}"] = s.final_error[j]
            rows.append(row)
        return rows
