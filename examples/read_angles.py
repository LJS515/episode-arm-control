#!/usr/bin/env python3
"""Read motor and model joint angles (read-only; safe on hardware)."""

import time

from _common import connect, fmt, parser

from episode_hardware.config_loader import load_mapping


def main() -> None:
    ap = parser(__doc__)
    ap.add_argument("-n", type=int, default=5)
    args = ap.parse_args()
    client, srv, env = connect(args)
    mapping = load_mapping()
    print(f"[{env}] mapping profile: {mapping.name} (evidence {mapping.evidence})")
    try:
        for _ in range(args.n):
            t0 = time.perf_counter()
            motor = client.get_motor_angles()
            dt = (time.perf_counter() - t0) * 1000
            print(f"[{env}] motor={fmt(motor)} model={fmt(mapping.motor_to_model_deg(motor))} rtt={dt:.1f}ms")
            time.sleep(0.2)
    finally:
        client.close()
        if srv:
            srv.stop()


if __name__ == "__main__":
    main()
