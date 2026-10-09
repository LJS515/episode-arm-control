#!/usr/bin/env python3
"""Cartesian move via the official IK (reference IK in Mock mode), completion judged from feedback."""

from _common import connect, controller_for, fmt, parser

from episode_sdk.exceptions import NoSolutionError
from episode_sdk.models import Pose


def main() -> None:
    ap = parser(__doc__)
    ap.add_argument("--xyz", type=float, nargs=3, default=[250, 50, 380])
    ap.add_argument("--rot", type=float, nargs=3, default=[90, 0, 180])
    ap.add_argument("--linear", action="store_true")
    args = ap.parse_args()
    client, srv, env = connect(args)
    ctl = controller_for(client)
    try:
        ctl.configure()
        ctl.poll_once()
        ctl.declare_homed("example-operator")
        ctl.start()
        try:
            t = ctl.move_pose(Pose(*args.xyz, *args.rot, order="zyx"), linear=args.linear)
        except NoSolutionError:
            print(f"[{env}] no IK solution for {args.xyz} {args.rot}")
            return
        t.wait(30)
        print(f"[{env}] {t.command_id}: {t.status.value} ({t.reason})")
        print(f"[{env}] pose now: {fmt(client.get_pose())}")
    finally:
        ctl.shutdown()
        if srv:
            srv.stop()


if __name__ == "__main__":
    main()
