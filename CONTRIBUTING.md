# Contributing

## Ground rules

1. **Evidence levels are mandatory.** Every factual statement about the robot
   or the official software carries A/B/C/D/E (see `EVIDENCE_MATRIX.md`).
   A new claim without a source is E.
2. **No invented numbers.** CAN IDs, bit rates, latencies, accuracies and
   trajectory errors are either measured (A, with date, setup and raw data in
   `reports/`) or written as `TBD` / `待真机测量`.
3. **Mock results are labelled** `Synthetic / Mock Data` wherever they appear.
4. **Official vs. ours.** Do not describe vendor capabilities (mechanics,
   drivers, FOC, upper computer, IK, control core) as project work.
5. **Safety-relevant changes** (state machine, supervisor, arbiter, estop path,
   limits) need a test that fails without the change.

## Workflow

```bash
python -m venv .venv && . .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements-dev.txt
python -m pytest -q
python tools/validate_configuration.py
python examples/mock_demo.py
```

* Keep the core (`episode_sdk`, `episode_hardware`) free of ROS imports so it
  stays testable without ROS.
* Hardware access goes through `SafeRobotController.hardware_lock`; only the
  E-stop path may bypass it.
* Motion commands are never retried automatically.

## Recording a hardware result

Add `reports/YYYY-MM-DD_<topic>.md` with: robot serial/firmware (e.g. driver
V5.1.5D), upper-computer version, host OS, cable/termination, procedure, raw
data file, analysis script + version, and who was present. Then update
`EVIDENCE_MATRIX.md` (E → A) and the relevant technical report.
