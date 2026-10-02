# NIDAR RescueSwarm

Companion-computer autonomous flight for an ArduCopter drone. A Raspberry Pi 5
talks MAVLink directly to the flight controller (pymavlink, ArduCopter **GUIDED**
mode) and flies a GPS-polygon raster ("lawnmower") coverage pattern, with
companion-side geofencing, pilot takeover on the transmitter, and an ultra-slow
soft landing.

> ⚠️ This code arms and flies a real aircraft. Fly only in a clear area with the
> pilot holding the transmitter and ready to take over on CH8. Never enable
> `BENCH_MODE` or `AUTOSTART_WITHOUT_RC` with propellers fitted.

## Hardware

| Component | Used here |
| --- | --- |
| Flight controller | MicoAir743v2, ArduCopter 4.6.x, USB MAVLink at 115200 baud |
| Companion computer | Raspberry Pi 5, Ubuntu 24.04 ARM64 |
| Radio | RadioMaster TX15 / ExpressLRS (CH8 = Switch SC) |
| GPS | 3D fix required |
| Camera (optional) | Microsoft LifeCam HD-3000, recorded with ffmpeg |

The flight controller is found automatically through `/dev/serial/by-id/`, so
the `ttyACM` number is never hard-coded.

## Setup

```bash
python3 -m venv ~/px4_mavsdk_env          # venv name is historical; no PX4/MAVSDK is used
source ~/px4_mavsdk_env/bin/activate
pip install -r requirements.txt           # pymavlink, pyserial
sudo apt install ffmpeg                   # only needed for video recording
```

The launchers and systemd units use absolute paths under `/home/aahswarm/`.
Edit them if you clone the project somewhere else.

## Raster coverage mission

`ardupilot_raster_mission.py` is the main mission. It reuses the base daemon
(`ardupilot_horizontal_geofence_mission.py`) for connection, telemetry,
takeoff, failsafes and landing, and replaces only the horizontal flight leg.

1. Validate both polygons and build the raster path **before** connecting to the FC.
2. Take off and stabilise.
3. Record one GPS + local-NED snapshot and use it to convert waypoints into the FC's frame.
4. Fly three phases: `TRANSIT_TO_RASTER` → `RASTER` → `RETURN_TO_ORIGIN`.
5. Soft-land with the base mission's landing sequence.

**Two GPS polygons** (four corners each, in perimeter order):

- **Inner** (`RASTER_CORNERS`): the area to cover. The aircraft must stay inside it during the `RASTER` phase.
- **Outer** (`OUTER_GEOFENCE_CORNERS`): where the aircraft may be at all, enforced in every phase with an inward margin.

Leaving either one zeroes horizontal velocity and triggers a safety landing.
Both fences run in the Pi's Python process, not as ArduPilot `FENCE_*`
parameters. If the Pi process dies, the fences stop and the pilot and the FC's
own failsafes are what remain.

### Preview a plan offline (no FC needed)

```bash
python raster_plan_preview.py --spacing 1.0 --speed 0.5 \
  --corners 'latA,lonA;latB,lonB;latC,lonC;latD,lonD' \
  --outer   'latG1,lonG1;latG2,lonG2;latG3,lonG3;latG4,lonG4'
```

This writes `raster_preview.geojson`, which you can drop onto
[geojson.io](https://geojson.io) to check the path on a map before flying.

### Run it

```bash
sudo cp ardupilot-raster-mission.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start ardupilot-raster-mission.service
journalctl -fu ardupilot-raster-mission.service
```

The daemon waits in STANDBY. A mission starts only when it sees a **fresh
switch into GUIDED**. Being in GUIDED already when the daemon starts does not
launch it. To dry-run with telemetry only (no arm, mode or motion commands),
set `RASTER_DRY_RUN=1`.

Main settings, in `start_ardupilot_raster_mission.sh`:

| Variable | Value | Meaning |
| --- | --- | --- |
| `RASTER_PASS_SPACING_M` | 1.0 | distance between passes |
| `RASTER_SPEED_MPS` | 0.50 | pattern speed (limited by overshoot at pass ends) |
| `RASTER_TOTAL_TIMEOUT_S` | 650 | upper limit on total mission time |
| `RASTER_ALT_MIN_M` / `MAX_M` | 1.5 / 2.5 | altitude band, abort outside |
| `RASTER_MIN_BATTERY_PERCENT` | 25 | companion battery floor |

## Pilot control (CH8 / Switch SC)

| CH8 PWM | Mode |
| --- | --- |
| < 1250 | **MANUAL**: the companion stops all commands and the pilot has control |
| 1250–1750 | **GUIDED**: autonomous mission allowed |
| > 1750 | **LAND**: immediate safety landing |

Loss of an RC link that was present also aborts and lands. The daemon sends a
1 Hz GCS heartbeat (sysid 255) so `FS_GCS_ENABLE=5` lands the aircraft if the Pi dies.

## Simulation

Runs the real mission code against `simulator.py` over loopback UDP. It never
touches the serial port.

```bash
./run_simulation.sh            # everything
./run_simulation.sh raster     # 17 raster flight tests (breaches, stale telemetry, EKF, altitude, battery…)
./run_simulation.sh dryrun     # dry-run tests; also checks that no flight command was sent
./run_simulation.sh geometry   # fast offline polygon-refusal tests
```

## Files

| File | Purpose |
| --- | --- |
| `ardupilot_raster_mission.py` | Polygon raster coverage mission (main) |
| `ardupilot_horizontal_geofence_mission.py` | Base daemon: connection, telemetry, takeoff, failsafes, soft landing |
| `ardupilot_3m_soft_bounce_mission.py` | Variant: fly out to a 3 m circle, bounce back, land |
| `ardupilot_fly and land.py` | Earlier simple mission: take off, fly forward, land |
| `raster_plan_preview.py` | Offline raster planner, standard library only (shared with the mission) |
| `fc_telemetry_logger.py` | Verbose FC log hooked onto the mission's own connection |
| `flight_recorder.py` | Passive recorder for manual flights (see `FLIGHT_RECORDER.md`) |
| `preflight_check.py` | Read-only readiness check; exit 0 = ready |
| `make_flight_ready.py` | Applies safe params, clears latched failsafes; never arms |
| `compass_calibrate.py` | Onboard compass calibration over MAVLink |
| `ardupilot_connection_test.py` | Quick link/telemetry diagnostic |
| `video_recorder.py` | ffmpeg MJPEG capture with camera auto-discovery |
| `simulator.py`, `run_simulation.sh` | Simulated FC and test scenarios |
| `*.service`, `start_*.sh` | systemd units and launchers |

Only one process may own the FC serial link. The mission, preflight check,
compass calibration and recorder share a lock file
(`/tmp/ardupilot_mission.lock`) and refuse to run together.

## Further reading

- `KNOWN_GOOD_FLIGHT_RUNBOOK.md`: field procedure and step-by-step recovery
- `FLIGHT_RECORDER.md`: recording manual diagnostic flights
- `PROJECT_STATUS.txt`: hardware notes and the compass/EKF history
