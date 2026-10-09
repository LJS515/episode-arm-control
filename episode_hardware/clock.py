"""Injectable clocks so that timing logic is deterministic under test."""

from __future__ import annotations

import threading
import time
from typing import Callable

Clock = Callable[[], float]


def monotonic() -> float:
    return time.monotonic()


class FakeClock:
    """Manually advanced clock (thread-safe)."""

    def __init__(self, start: float = 1000.0):
        self._t = float(start)
        self._lock = threading.Lock()

    def __call__(self) -> float:
        with self._lock:
            return self._t

    def advance(self, dt: float) -> float:
        with self._lock:
            self._t += float(dt)
            return self._t

    def set(self, t: float) -> None:
        with self._lock:
            self._t = float(t)
