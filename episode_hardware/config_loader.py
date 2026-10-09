"""Load the YAML files in ``config/`` into typed objects."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional, Union

from episode_sdk.client import EpisodeClient
from episode_sdk.models import ClientTimeouts, ConnectionPolicy, JointLimits
from episode_sdk.retry import RetryPolicy

from .command_arbiter import BusyPolicy
from .joint_mapping import JointMapping
from .robot_controller import ControllerConfig
from .safety_supervisor import SafetyConfig

DEFAULT_CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
PathLike = Union[str, Path]


def load_yaml(path: PathLike) -> Dict[str, Any]:
    import yaml  # PyYAML is a declared dependency

    with Path(path).open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: top level must be a mapping")
    return data


def load_joint_limits(config_dir: PathLike = DEFAULT_CONFIG_DIR) -> JointLimits:
    d = load_yaml(Path(config_dir) / "joint_limits.yaml")
    lim = d["joint_limits_deg"]
    return JointLimits(tuple(float(p[0]) for p in lim), tuple(float(p[1]) for p in lim))


def load_mapping(config_dir: PathLike = DEFAULT_CONFIG_DIR, profile: Optional[str] = None) -> JointMapping:
    robot = load_yaml(Path(config_dir) / "robot.yaml")
    m = robot["mapping"]
    name = profile or m["active_profile"]
    p = dict(m["profiles"][name])
    p["name"] = name
    lim = load_joint_limits(config_dir)
    p["motor_limits_deg"] = list(zip(lim.lower, lim.upper))
    return JointMapping.from_dict(p)


def load_controller_config(config_dir: PathLike = DEFAULT_CONFIG_DIR) -> ControllerConfig:
    cdir = Path(config_dir)
    robot = load_yaml(cdir / "robot.yaml").get("controller", {})
    safety = dict(load_yaml(cdir / "safety.yaml").get("safety", {}))
    limits = load_yaml(cdir / "joint_limits.yaml")
    safety["joint_limits_deg"] = limits["joint_limits_deg"]
    return ControllerConfig(
        poll_rate_hz=float(robot.get("poll_rate_hz", 20.0)),
        stale_after_s=float(robot.get("stale_after_s", 0.5)),
        busy_policy=BusyPolicy(robot.get("busy_policy", "reject")),
        max_queue=int(robot.get("max_queue", 4)),
        homing_tolerance_deg=float(robot.get("homing_tolerance_deg", 2.0)),
        stationary_eps_deg=float(robot.get("stationary_eps_deg", 0.05)),
        safety=SafetyConfig.from_dict(safety),
    )


def load_client(config_dir: PathLike = DEFAULT_CONFIG_DIR, *, host: Optional[str] = None,
                port: Optional[int] = None) -> EpisodeClient:
    cdir = Path(config_dir)
    p = load_yaml(cdir / "protocol.yaml")["protocol"]
    t = p.get("timeouts", {})
    r = p.get("retry", {})
    return EpisodeClient(
        host or p.get("host", "127.0.0.1"),
        int(port or p.get("port", 12345)),
        legacy_compatible=bool(p.get("legacy_compatible", True)),
        connection_policy=ConnectionPolicy(p.get("connection_policy", "per_command")),
        timeouts=ClientTimeouts(float(t.get("connect_s", 2)), float(t.get("send_s", 2)),
                                float(t.get("receive_s", 5)), float(t.get("command_total_s", 10))),
        max_message_size=int(p.get("max_message_size", 1 << 20)),
        joint_limits=load_joint_limits(cdir),
        retry_policy=RetryPolicy(max_attempts=int(r.get("max_attempts", 3)),
                                 initial_backoff_s=float(r.get("initial_backoff_s", 0.05))),
        protocol_version=str(p.get("protocol_version", "1.0")),
    )
