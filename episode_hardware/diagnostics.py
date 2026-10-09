"""Diagnostics aggregation (maps onto diagnostic_msgs/DiagnosticStatus levels)."""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .state_cache import JointStateSnapshot
from .state_machine import RobotState


class Level(enum.IntEnum):
    # same numeric values as diagnostic_msgs/DiagnosticStatus
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


@dataclass
class DiagnosticItem:
    name: str
    level: Level
    message: str
    values: Dict[str, str] = field(default_factory=dict)


def build_diagnostics(
    snap: JointStateSnapshot,
    state: RobotState,
    *,
    homed: bool,
    active_faults: List[Dict[str, Any]],
    arbiter: Optional[Dict[str, Any]] = None,
    hardware_description: str = "",
    warn_age_s: Optional[float] = None,
) -> List[DiagnosticItem]:
    items: List[DiagnosticItem] = []

    # feedback freshness
    if snap.stamp is None:
        items.append(DiagnosticItem("episode/feedback", Level.STALE, "no feedback received yet"))
    elif snap.stale:
        items.append(DiagnosticItem("episode/feedback", Level.STALE, f"feedback stale (age {snap.age_s:.3f}s)",
                                    {"age_s": f"{snap.age_s:.3f}"}))
    elif not snap.valid:
        missing = [str(i + 1) for i, ok in enumerate(snap.joint_valid) if not ok]
        items.append(DiagnosticItem("episode/feedback", Level.ERROR, f"joints without data: {','.join(missing)}"))
    elif warn_age_s is not None and snap.age_s is not None and snap.age_s > warn_age_s:
        items.append(DiagnosticItem("episode/feedback", Level.WARN, f"feedback slow (age {snap.age_s:.3f}s)"))
    else:
        items.append(DiagnosticItem("episode/feedback", Level.OK, "fresh", {"age_s": f"{snap.age_s:.3f}"}))
    items[-1].values.update({
        "consecutive_failures": str(snap.consecutive_failures),
        "total_failures": str(snap.total_failures),
        "total_samples": str(snap.total_samples),
        "last_error": snap.last_error or "",
    })

    # state machine
    lvl = {RobotState.FAULT: Level.ERROR, RobotState.ESTOP: Level.ERROR,
           RobotState.UNCONFIGURED: Level.WARN, RobotState.INACTIVE: Level.WARN}.get(state, Level.OK)
    items.append(DiagnosticItem("episode/state", lvl, state.value, {
        "homed": str(homed),
        "faults": ";".join(f"{f['code']}:{f['detail']}" for f in active_faults),
    }))

    if arbiter is not None:
        items.append(DiagnosticItem("episode/arbiter", Level.ERROR if arbiter.get("estop") else Level.OK,
                                    "estop" if arbiter.get("estop") else "ok",
                                    {k: str(v) for k, v in arbiter.items()}))
    if hardware_description:
        items.append(DiagnosticItem("episode/hardware", Level.OK, hardware_description))
    return items


def worst_level(items: List[DiagnosticItem]) -> Level:
    return max((i.level for i in items), default=Level.OK)


def to_dict(items: List[DiagnosticItem]) -> List[Dict[str, Any]]:
    return [{"name": i.name, "level": i.level.name, "message": i.message, "values": i.values} for i in items]
