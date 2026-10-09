#!/usr/bin/env python3
"""Validate the YAML configuration in ``config/`` (and the ROS2 copies).

Checks
------
* every file parses and has the expected top-level keys
* 6 joints everywhere; lower < upper; signs are +-1
* safety numbers are in sane ranges
* mapping offsets lie inside the motor ranges
* the ROS2 package's robot.yaml joint limits equal config/joint_limits.yaml
* reports (does not fail on) the known DH-qlim vs. motor-range mismatch

Exit code 0 = OK, 1 = errors found.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from episode_hardware.config_loader import (  # noqa: E402
    load_client,
    load_controller_config,
    load_joint_limits,
    load_mapping,
    load_yaml,
)
from episode_hardware.joint_mapping import (  # noqa: E402
    OFFICIAL_DH_THETA_RANGE_DEG,
    VENDOR_DH_QLIM_DEG,
    check_dh_qlim_consistency,
    get_profile,
)


def validate(config_dir: Path, ros_config_dir: Path) -> (List[str], List[str]):
    errors: List[str] = []
    notes: List[str] = []
    for name, key in [("joint_limits.yaml", "joint_limits_deg"), ("robot.yaml", "robot"),
                      ("safety.yaml", "safety"), ("protocol.yaml", "protocol"), ("logging.yaml", "version")]:
        try:
            d = load_yaml(config_dir / name)
            if key not in d:
                errors.append(f"{name}: missing key '{key}'")
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    if errors:
        return errors, notes
    lim = load_joint_limits(config_dir)
    for prof in ("vendor_dh", "ros_model"):
        m = load_mapping(config_dir, prof)
        for i, (o, lo, hi) in enumerate(zip(m.offsets_deg, lim.lower, lim.upper)):
            if not lo <= o <= hi:
                errors.append(f"mapping {prof}: J{i+1} offset {o} outside motor range [{lo},{hi}]")
        if prof == "vendor_dh" and (m.signs, m.offsets_deg) != (get_profile("vendor_dh").signs, get_profile("vendor_dh").offsets_deg):
            errors.append("robot.yaml vendor_dh differs from the code constant VENDOR_DH")
    cfg = load_controller_config(config_dir)
    s = cfg.safety
    if not 0 < s.speed_ratio_min < s.speed_ratio_default_cap <= s.speed_ratio_max <= 1.0:
        errors.append("safety: need 0 < speed_ratio_min < default_cap <= max <= 1")
    if s.max_consecutive_read_failures < 1:
        errors.append("safety: max_consecutive_read_failures must be >= 1")
    if cfg.stale_after_s * cfg.poll_rate_hz < 2:
        errors.append("controller: stale_after_s must cover at least 2 poll periods")
    if s.final_tolerance_deg <= 0 or s.settle_time_s < 0:
        errors.append("safety: tolerances must be positive")
    load_client(config_dir)  # raises on bad protocol section

    ros_robot = ros_config_dir / "robot.yaml"
    if ros_robot.is_file():
        d = load_yaml(ros_robot)
        params = d.get("episode_robot_interface", {}).get("ros__parameters", {})
        lower, upper = params.get("joint_lower_deg"), params.get("joint_upper_deg")
        if lower is not None and (tuple(map(float, lower)), tuple(map(float, upper))) != (lim.lower, lim.upper):
            errors.append("episode_ros2/config/robot.yaml joint limits differ from config/joint_limits.yaml")
    else:
        notes.append(f"{ros_robot} not found (skipped)")

    for diff in check_dh_qlim_consistency(get_profile("vendor_dh"), VENDOR_DH_QLIM_DEG):
        notes.append(f"KNOWN (C): J{diff['joint']} motor range in DH space {diff['motor_mapped']} "
                     f"!= historical client qlim {diff['qlim']} (current official DH table agrees with the motor range)")
    if check_dh_qlim_consistency(get_profile("vendor_dh"), OFFICIAL_DH_THETA_RANGE_DEG):
        errors.append("vendor_dh mapping no longer matches the official DH theta table (tid=1217)")
    notes.append("ros_model signs match official ROS 2 sources (evidence B); verify visually before hardware use")
    return errors, notes


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config-dir", default=str(ROOT / "config"))
    ap.add_argument("--ros-config-dir", default=str(ROOT / "episode_ros2" / "config"))
    args = ap.parse_args()
    errors, notes = validate(Path(args.config_dir), Path(args.ros_config_dir))
    for n in notes:
        print("NOTE :", n)
    for e in errors:
        print("ERROR:", e)
    print("configuration OK" if not errors else f"{len(errors)} error(s)")
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
