"""Thread-safe joint state cache with freshness tracking.

Why a timestamp is mandatory: a cached position without a sample time is
indistinguishable from a live one. If the reader thread dies or CAN goes
silent, consumers would keep acting on an old pose. Every snapshot here
carries ``stamp``, ``age_s`` and an explicit ``stale`` verdict.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from episode_sdk.models import NUM_JOINTS

from .clock import Clock, monotonic


@dataclass(frozen=True)
class JointStateSnapshot:
    positions_deg: Tuple[Optional[float], ...]
    #: time the sample was taken (clock domain of the cache)
    stamp: Optional[float]
    #: time the snapshot was created
    now: float
    age_s: Optional[float]
    #: True only if all 6 joints were read successfully
    valid: bool
    joint_valid: Tuple[bool, ...]
    stale: bool
    consecutive_failures: int
    total_failures: int
    total_samples: int
    last_success_time: Optional[float]
    last_error: Optional[str]
    velocity_deg_s: Tuple[Optional[float], ...]

    @property
    def usable(self) -> bool:
        """Safe to base a motion decision on this snapshot."""
        return self.valid and not self.stale

    def require_positions(self) -> List[float]:
        if not self.usable:
            raise ValueError("snapshot not usable (invalid or stale)")
        return [float(p) for p in self.positions_deg]  # type: ignore[arg-type]


class JointStateCache:
    def __init__(self, *, stale_after_s: float = 0.5, clock: Clock = monotonic):
        if stale_after_s <= 0:
            raise ValueError("stale_after_s must be > 0")
        self.stale_after_s = float(stale_after_s)
        self.clock = clock
        self._lock = threading.Lock()
        self._pos: Tuple[Optional[float], ...] = (None,) * NUM_JOINTS
        self._prev: Optional[Tuple[Tuple[Optional[float], ...], float]] = None
        self._vel: Tuple[Optional[float], ...] = (None,) * NUM_JOINTS
        self._stamp: Optional[float] = None
        self._joint_valid: Tuple[bool, ...] = (False,) * NUM_JOINTS
        self._consecutive_failures = 0
        self._total_failures = 0
        self._total_samples = 0
        self._last_success: Optional[float] = None
        self._last_error: Optional[str] = None

    def update_success(self, positions: Sequence[Optional[float]], stamp: Optional[float] = None) -> None:
        if len(positions) != NUM_JOINTS:
            raise ValueError("need 6 positions")
        stamp = self.clock() if stamp is None else float(stamp)
        pos = tuple(None if p is None else float(p) for p in positions)
        jv = tuple(p is not None for p in pos)
        with self._lock:
            if self._stamp is not None and stamp < self._stamp:
                # out-of-order sample: never let time go backwards
                return
            if self._stamp is not None and stamp > self._stamp:
                dt = stamp - self._stamp
                self._vel = tuple(
                    None if (a is None or b is None) else (a - b) / dt for a, b in zip(pos, self._pos)
                )
            self._pos = pos
            self._stamp = stamp
            self._joint_valid = jv
            self._total_samples += 1
            if all(jv):
                self._consecutive_failures = 0
                self._last_success = stamp
                self._last_error = None
            else:
                # partial read counts as a failure for the purpose of safety
                self._consecutive_failures += 1
                self._total_failures += 1
                missing = [i + 1 for i, ok in enumerate(jv) if not ok]
                self._last_error = f"joints without data: {missing}"

    def update_failure(self, error: str) -> None:
        with self._lock:
            self._consecutive_failures += 1
            self._total_failures += 1
            self._last_error = str(error)

    def snapshot(self) -> JointStateSnapshot:
        now = self.clock()
        with self._lock:
            stamp = self._stamp
            age = None if stamp is None else max(0.0, now - stamp)
            valid = stamp is not None and all(self._joint_valid)
            stale = (
                stamp is None
                or age is None
                or age > self.stale_after_s
                or (self._last_success is None)
                or (now - self._last_success) > self.stale_after_s
            )
            return JointStateSnapshot(
                positions_deg=self._pos,
                stamp=stamp,
                now=now,
                age_s=age,
                valid=valid,
                joint_valid=self._joint_valid,
                stale=stale,
                consecutive_failures=self._consecutive_failures,
                total_failures=self._total_failures,
                total_samples=self._total_samples,
                last_success_time=self._last_success,
                last_error=self._last_error,
                velocity_deg_s=self._vel,
            )

    def invalidate(self, reason: str = "invalidated") -> None:
        with self._lock:
            self._joint_valid = (False,) * NUM_JOINTS
            self._last_success = None
            self._last_error = reason
