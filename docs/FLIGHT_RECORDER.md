# Passive Flight Recorder

`flight_recorder.py` is for collecting evidence from a manually flown
Stabilize/PosHold test. It never arms, changes modes, writes parameters, or
sends movement commands. It does send the normal 1 Hz companion heartbeat and
requests additional diagnostic telemetry while it is connected.

It records one private, timestamped directory under `flight_logs/` containing:

- `mavlink-001.tlog` — raw MAVLink telemetry, directly usable in Mission Planner.
- `telemetry.jsonl` — every decoded MAVLink message with an ISO-UTC receive time.
- `events.jsonl` — arming/mode changes, pre-arm messages, failsafe/safety text,
  EKF status, and fence events.
- `parameters.json` — a read-only FC parameter snapshot, updated while received.
- `recorder.log` and `metadata.json` — recorder lifecycle/configuration details.

It explicitly requests attitude, GPS, position, EKF, vibration, raw magnetometer
(`RAW_IMU`, `SCALED_IMU2`, `SCALED_IMU3`), battery, RC, radio, home, and fence
telemetry. This gives us the information needed to compare manual and PosHold
behaviour, compass field/yaw, EKF transitions, RC quality, battery sag, and
failsafe events.

## Install once

```bash
sudo install -m 0644 ardupilot-flight-recorder.service /etc/systemd/system/ardupilot-flight-recorder.service
sudo systemctl daemon-reload
```

Do not enable the recorder at boot: it is intended only for a deliberate test
session and conflicts with the autonomous mission service.

## Record a flight

```bash
sudo systemctl start ardupilot-flight-recorder.service
journalctl -fu ardupilot-flight-recorder.service
```

Wait until the journal says `recorder_ready`, then fly manually/PosHold. The
unit conflicts with `ardupilot-mission.service`, so it stops that autonomous
daemon before acquiring the FC serial link. Stop the recorder only after the
aircraft is disarmed:

```bash
sudo systemctl stop ardupilot-flight-recorder.service
```

The completed directory will be printed in the journal and stored under:

```bash
ls -dt /home/aahswarm/ardupilot_testing/flight_logs/flight-* | head -1
```

To return to autonomous operation after reviewing the log:

```bash
sudo systemctl start ardupilot-mission.service
```

Never run the recorder and autonomous daemon together. The shared file lock and
the systemd conflict are deliberate safeguards against corrupted MAVLink data.
