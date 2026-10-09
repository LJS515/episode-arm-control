"""Shared helpers for the examples.

Safety convention: every example runs against the Mock by default. Talking to a
real upper computer requires BOTH ``--host`` and ``--real-robot``; the script then
asks for an interactive confirmation that the workspace is clear and that the
hardware E-stop is within reach.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from episode_hardware.config_loader import load_client, load_controller_config  # noqa: E402
from episode_hardware.robot_controller import SafeRobotController  # noqa: E402
from episode_hardware.vendor_adapter import VendorTcpAdapter  # noqa: E402
from episode_sdk import EpisodeClient  # noqa: E402
from episode_sdk.logging_config import setup_logging  # noqa: E402
from episode_sdk.mock_server import MockEpisodeServer  # noqa: E402


def parser(desc: str) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--host", default=None, help="official upper computer host (omit to use the Mock)")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--real-robot", action="store_true", help="required together with --host")
    ap.add_argument("--log-level", default="INFO")
    return ap


def confirm_real() -> None:
    print("=" * 70)
    print(" REAL ROBOT MODE")
    print("  * workspace clear of people and objects?")
    print("  * HARDWARE emergency stop / power switch within reach?")
    print("  * speed_ratio capped at the commissioning value (safety.yaml)?")
    print("=" * 70)
    if input("type 'yes' to continue: ").strip().lower() != "yes":
        sys.exit("aborted by operator")


def connect(args) -> Tuple[EpisodeClient, Optional[MockEpisodeServer], str]:
    setup_logging(args.log_level)
    if args.host:
        if not args.real_robot:
            sys.exit("refusing to talk to a real host without --real-robot")
        confirm_real()
        return load_client(host=args.host, port=args.port), None, "REAL HARDWARE"
    srv = MockEpisodeServer().start()
    client = load_client(host=srv.host, port=srv.port)
    return client, srv, "Synthetic / Mock Data"


def controller_for(client: EpisodeClient) -> SafeRobotController:
    return SafeRobotController(VendorTcpAdapter(client), load_controller_config())


def fmt(values) -> list:
    return [None if v is None else round(v, 2) for v in values]
