#!/usr/bin/env python3
"""Send a software emergency stop.

This is NOT a replacement for the hardware E-stop / power switch. If this
script reports a transport error, cut motor power immediately.
"""

import sys

from _common import connect, parser

from episode_sdk.exceptions import EpisodeError


def main() -> None:
    args = parser(__doc__).parse_args()
    client, srv, env = connect(args)
    try:
        client.emergency_stop()
        print(f"[{env}] emergency_stop acknowledged")
    except EpisodeError as exc:
        print(f"[{env}] emergency_stop FAILED: {exc} -> USE THE HARDWARE E-STOP", file=sys.stderr)
        sys.exit(2)
    finally:
        client.close()
        if srv:
            srv.stop()


if __name__ == "__main__":
    main()
