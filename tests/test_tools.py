"""Tools and configuration (synthetic inputs only)."""

import csv
import json
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import can_trace_parser  # noqa: E402
import generate_mock_report  # noqa: E402
import latency_analyzer  # noqa: E402
import tcp_trace  # noqa: E402
import trajectory_analyzer  # noqa: E402
import validate_configuration  # noqa: E402

from episode_hardware.config_loader import load_client, load_controller_config, load_mapping  # noqa: E402
from episode_sdk import EpisodeClient  # noqa: E402


def test_config_files_validate():
    errors, notes = validate_configuration.validate(ROOT / "config", ROOT / "episode_ros2" / "config")
    assert errors == []
    assert any("KNOWN" in n for n in notes)


def test_config_loader_objects():
    cfg = load_controller_config()
    assert cfg.safety.homing_timeout_s == 60.0
    assert load_mapping().name == "ros_model"
    c = load_client(host="127.0.0.1", port=1)
    assert c.mode.value == "legacy"


def test_can_parser_candump_and_trc():
    lines = [
        "(1700000000.000000) can0 141#A100000000000000",
        "(1700000000.010000) can0 141#A100000000000000",
        "(1700000000.020000) can0 141#A100000000000000",
        "(1700000000.015000) can0 1F1#01",
        "     1)         0.0  Rx         0201  2  01 02",
        "     2       10.000 DT     0201 Rx 2  01 02",
        "; comment",
        "garbage line",
    ]
    frames = can_trace_parser.parse(lines)
    assert len(frames) == 6
    st = can_trace_parser.stats(frames)
    assert st[0x141].count == 3 and st[0x201].count == 2
    rep = can_trace_parser.report(sorted(frames[:4], key=lambda f: f.t))
    assert "0x141" in rep and "TBD (bit rate not measured)" in rep
    assert can_trace_parser.bus_load_estimate(frames[:3], 500000) is not None


def test_latency_summary_math():
    res = latency_analyzer.summarize([1, 2, 3, 4, 100], 1, "Synthetic / Mock Data", "per_command")
    assert res["p50_ms"] == 3 and res["max_ms"] == 100 and res["n_error"] == 1
    assert res["environment"].startswith("Synthetic")


def test_trajectory_analyzer(tmp_path):
    p = tmp_path / "traj.csv"
    with p.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t"] + [f"desired_{j}" for j in range(1, 7)] + [f"actual_{j}" for j in range(1, 7)])
        for i in range(11):
            d = i
            w.writerow([i * 0.1] + [d] * 6 + [max(0, d - 1)] * 6)
    res = trajectory_analyzer.analyse(trajectory_analyzer.load(str(p)), tol=1.0)
    assert res["joints"]["J1"]["max_tracking_error_deg"] == 1
    assert res["joints"]["J1"]["final_error_deg"] == 1


def test_mock_report_contains_label():
    import logging

    logging.disable(logging.CRITICAL)
    try:
        results = [(sid, name, fn()) for sid, name, fn in generate_mock_report.SCENARIOS]
    finally:
        logging.disable(logging.NOTSET)
    text = generate_mock_report.render(results)
    assert "Synthetic / Mock Data" in text
    rows = {sid: o for sid, _, o in results}
    assert rows["F01"]["rejected"] == "NotHomedError"
    assert rows["F02"]["ticket"] == "SUCCEEDED"
    assert rows["F05"]["state"] == "ESTOP"
    assert "READ_FAILURES" in rows["F06"]["faults"]
    assert rows["F09"]["faults"] == "FINAL_ERROR_EXCEEDED"


def test_tcp_trace_proxy_logs_frames(legacy_server):
    records = []
    tracer = records.append
    stop = threading.Event()
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    th = threading.Thread(target=tcp_trace.serve, args=(("127.0.0.1", port), legacy_server.address, tracer, stop), daemon=True)
    th.start()
    time.sleep(0.2)
    c = EpisodeClient("127.0.0.1", port)
    assert c.get_motor_angles()[0] == pytest.approx(180)
    time.sleep(0.2)
    stop.set()
    th.join(2)
    frames = [r for r in records if "len" in r]
    assert frames[0]["dir"] == "c2s" and frames[0]["json"]["action"] == "get_motor_angles"
    assert frames[1]["dir"] == "s2c" and len(frames[1]["json"]) == 6


def test_sniffer_handles_non_json_without_deserialising():
    out = []
    sn = tcp_trace.FrameSniffer("c2s", out.append, 1)
    payload = b"\x80\x04\x95pickle-bytes"
    sn.feed((len(payload)).to_bytes(8, "big") + payload[:3])
    sn.feed(payload[3:])
    assert out[0]["non_json_hex_prefix"].startswith("8004")
