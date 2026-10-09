#!/usr/bin/env python3
"""Joint move through the full safety stack (supervisor + arbiter + monitor)."""

from _common import connect, controller_for, parser

from episode_hardware.command_arbiter import TicketStatus


def main() -> None:
    ap = parser(__doc__)
    ap.add_argument("--target", type=float, nargs=6, default=[190, 95, 83, 30, 110, 40],
                    help="motor degrees J1..J6")
    ap.add_argument("--speed", type=float, default=0.2)
    args = ap.parse_args()
    client, srv, env = connect(args)
    ctl = controller_for(client)
    try:
        ctl.configure()
        ctl.poll_once()
        # The official server homes on start-up unless started with --resume;
        # a human has to confirm that, our layer cannot observe it.
        ctl.declare_homed("example-operator")
        ctl.start()
        ticket = ctl.move_joints(args.target, args.speed)
        print(f"[{env}] {ticket.command_id} accepted, state={ctl.state.value}")
        ticket.wait(30)
        res = ctl.results.get(ticket.command_id)
        print(f"[{env}] result: {ticket.status.value} reason={ticket.reason}")
        if res:
            print(f"[{env}] final error per joint (deg): {[round(e, 3) for e in res.final_error_deg]}")
        if ticket.status is not TicketStatus.SUCCEEDED:
            print(f"[{env}] faults: {ctl.supervisor.snapshot()['active_faults']}")
    finally:
        ctl.shutdown()
        if srv:
            srv.stop()


if __name__ == "__main__":
    main()
