# Security & Safety Policy

## Physical safety first

This code moves a real arm (≈ 12 V / 10 A supply, steppers with planetary
gearboxes). Software checks reduce risk; they do not make the system safe.

* A **hardware E-stop / power switch** must be within reach whenever the arm is
  powered. The software `emergency_stop` travels TCP → upper computer → CAN and
  can fail with any of them.
* First runs: `speed_ratio ≤ 0.3`, clear workspace, one person at the controls.
* `set_free_mode(True)` makes joints back-drivable — support the arm first.

## Network exposure of the official upper computer

* The historical public server (`episodeServer.py`, repo `enpeizhao/episode_robot`
  @9ede794, evidence C) **unpickled data received from the network**. `pickle`
  deserialisation of untrusted input allows arbitrary code execution on the host.
  The current official SDK is reported to use JSON (evidence B) — verify the
  version you run.
* The TCP API has **no authentication**. Anyone who can reach the port can move
  the robot. Bind it to `127.0.0.1` or an isolated network; never expose it to a
  shared LAN or the internet. Use `tools/tcp_trace.py` only on trusted hosts.
* This project's SDK never uses pickle; JSON decoding rejects NaN/Infinity and
  frames above `max_message_size` (default 1 MiB) are refused before allocation.

## Reporting

Report vulnerabilities or unsafe behaviour of *this repository's* code through
a private issue / e-mail to the maintainer. Issues in the official software
should be reported to the vendor (enpeicv). Please include version, steps,
and whether real hardware was involved.
