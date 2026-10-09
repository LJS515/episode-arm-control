"""Retry policy for *idempotent* requests only.

Rule: motion commands (``angle_mode``, ``move_*``, ``dynamic_move``,
``gripper_*``, ``servo_gripper``, ``set_free_mode``) are never retried
automatically. If a motion request timed out we do not know whether the
robot received it; resending could execute the move twice. Only reads such
as ``get_motor_angles`` / ``get_pose`` / ``get_T`` may be retried.
``emergency_stop`` is the exception: it is idempotent by nature and may be
re-sent until acknowledged.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, Tuple, Type, TypeVar

from .exceptions import (
    ConnectionClosedError,
    ConnectionFailedError,
    ConnectTimeoutError,
    EpisodeError,
    ReadFailedError,
    ReceiveTimeoutError,
)

T = TypeVar("T")
log = logging.getLogger(__name__)

IDEMPOTENT_ACTIONS = frozenset({"get_motor_angles", "get_T", "get_pose", "emergency_stop"})


@dataclass(frozen=True)
class RetryPolicy:
    max_attempts: int = 3
    initial_backoff_s: float = 0.05
    backoff_multiplier: float = 2.0
    max_backoff_s: float = 0.5
    retry_on: Tuple[Type[EpisodeError], ...] = (
        ConnectionFailedError,
        ConnectTimeoutError,
        ConnectionClosedError,
        ReceiveTimeoutError,
        ReadFailedError,
    )

    @classmethod
    def none(cls) -> "RetryPolicy":
        return cls(max_attempts=1)


def call_with_retry(
    fn: Callable[[], T],
    policy: RetryPolicy,
    *,
    action: str,
    sleep: Callable[[float], None] = time.sleep,
) -> T:
    attempts = policy.max_attempts if action in IDEMPOTENT_ACTIONS else 1
    backoff = policy.initial_backoff_s
    last: EpisodeError | None = None
    for attempt in range(1, attempts + 1):
        try:
            return fn()
        except policy.retry_on as exc:  # type: ignore[misc]
            last = exc
            if attempt >= attempts:
                break
            log.warning("retrying %s after %s (attempt %d/%d)", action, exc.code, attempt, attempts)
            sleep(backoff)
            backoff = min(backoff * policy.backoff_multiplier, policy.max_backoff_s)
    assert last is not None
    raise last
