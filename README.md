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

The launchers and systemd units expect the repository at
`/home/aahswarm/NIDAR_RescueSwarm` and run everything from `mission/`. Edit
`PROJECT_PATH` in `mission/start_*.sh` and the paths in `deploy/systemd/*.service`
if you clone it somewhere else.

## Project layout

```
NIDAR_RescueSwarm/
├── mission/          Flight code that runs on the Pi. Kept in one folder because the
│                     modules import each other and write their logs next to themselves.
├── sim/              Simulated flight controller and the scenario test suite
├── tests/            Offline planner and fixture tests (no FC, no simulator)
├── tools/            Standalone bench utilities: link test, preflight, compass calibration
├── vision/           Post-flight cone detection + geotagging on the onboard video (OpenCV)
├── deploy/systemd/   systemd units for the mission daemons and recorders
├── docs/             Runbook, recorder guide, failure analyses, controller-design notes
└── data/             Recorded flights, organised by type
    ├── flight_logs/    raw MAVLink .tlog files and passive-recorder sessions
    ├── telemetry/      verbose FC telemetry logs, named by the time range they cover
    ├── mission_logs/   mission daemon logs, named by the time range they cover
    ├── recordings/     onboard camera video (.mkv)
    └── plans/          planner output (raster_preview.geojson)
```

Logs written by a running daemon land in `mission/` and are ignored by git.
Copy the ones worth keeping into `data/`.

## Raster coverage mission

`mission/ardupilot_raster_mission.py` is the main mission. It reuses the base daemon
(`mission/ardupilot_horizontal_geofence_mission.py`) for connection, telemetry,
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
cd mission
python raster_plan_preview.py --spacing 1.0 --speed 0.5 \
  --corners 'latA,lonA;latB,lonB;latC,lonC;latD,lonD' \
  --outer   'latG1,lonG1;latG2,lonG2;latG3,lonG3;latG4,lonG4'
```

This writes `raster_preview.geojson`, which you can drop onto
[geojson.io](https://geojson.io) to check the path on a map before flying.

### Run it

```bash
sudo cp deploy/systemd/ardupilot-raster-mission.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start ardupilot-raster-mission.service
journalctl -fu ardupilot-raster-mission.service
```

The daemon waits in STANDBY. A mission starts only when it sees a **fresh
switch into GUIDED**. Being in GUIDED already when the daemon starts does not
launch it. To dry-run with telemetry only (no arm, mode or motion commands),
set `RASTER_DRY_RUN=1`.

Main settings, in `mission/start_ardupilot_raster_mission.sh`:

| Variable | Value | Meaning |
| --- | --- | --- |
| `RASTER_PASS_SPACING_M` | 1.0 | distance between passes |
| `RASTER_AXIS` | short | sweep along the short side of the polygon |
| `RASTER_SPEED_MPS` | 1.00 | pattern speed (limited by overshoot at pass ends) |
| `RASTER_TOTAL_TIMEOUT_S` | 650 | upper limit on total mission time |
| `TAKEOFF_ALTITUDE_M` | 2.5 | hover and pattern altitude |
| `RASTER_ALT_MIN_M` / `MAX_M` | 1.80 / 3.00 | altitude band, abort outside |
| `RASTER_MIN_BATTERY_PERCENT` | 25 | companion battery floor |

## Post-flight cone detection

`vision/` finds red, yellow and green cones in a downward-camera recording with
HSV colour thresholds (no ML), tracks them across frames, and geotags each one
from the FC telemetry log of the same flight. Cones seen again on a later raster
pass are merged. It runs after landing, on the laptop or the Pi:

```bash
pip install -r vision/requirements.txt
cd vision
python detect_cones_video.py ../data/recordings/flight_<ts>.mkv \
  --telemetry ../data/telemetry/fc_telemetry_<range>.log
```

Output goes to `flight_<ts>_cones/`; `cones.geojson` can go straight onto
[geojson.io](https://geojson.io). See `vision/README.md` for the camera
assumptions and the field checklist (time sync and camera orientation matter most).

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
sim/run_simulation.sh            # everything
sim/run_simulation.sh raster     # 17 raster flight tests (breaches, stale telemetry, EKF, altitude, battery…)
sim/run_simulation.sh dryrun     # dry-run tests; also checks that no flight command was sent
sim/run_simulation.sh geometry   # fast offline polygon-refusal tests
```

Offline tests, no simulator needed:

```bash
python tests/test_raster_entry_and_axis.py
python tests/test_simulation_fixtures.py   # run after changing either polygon
python tests/test_cone_detection.py        # vision/ (needs vision/requirements.txt)
```

## Files

| File | Purpose |
| --- | --- |
| `mission/ardupilot_raster_mission.py` | Polygon raster coverage mission (main) |
| `mission/ardupilot_horizontal_geofence_mission.py` | Base daemon: connection, telemetry, takeoff, failsafes, soft landing |
| `mission/ardupilot_3m_soft_bounce_mission.py` | Variant: fly out to a 3 m circle, bounce back, land |
| `mission/ardupilot_fly_and_land.py` | Earlier simple mission: take off, fly forward, land |
| `mission/raster_plan_preview.py` | Offline raster planner, standard library only (shared with the mission) |
| `mission/fc_telemetry_logger.py` | Verbose FC log hooked onto the mission's own connection |
| `mission/flight_recorder.py` | Passive recorder for manual flights (see `docs/FLIGHT_RECORDER.md`) |
| `tools/preflight_check.py` | Read-only readiness check; exit 0 = ready |
| `tools/make_flight_ready.py` | Applies safe params, clears latched failsafes; never arms |
| `tools/compass_calibrate.py` | Onboard compass calibration over MAVLink |
| `tools/ardupilot_connection_test.py` | Quick link/telemetry diagnostic |
| `mission/video_recorder.py` | ffmpeg MJPEG capture with camera auto-discovery; writes a `.json` start-time sidecar |
| `mission/flight_video_recorder.py` | Separate service that records video from takeoff to landing |
| `mission/fc_status.py` | Read-only snapshot of the FC, the mission daemon and its live config (`--json` available) |
| `sim/simulator.py`, `sim/run_simulation.sh` | Simulated FC and test scenarios |
| `deploy/systemd/*.service`, `mission/start_*.sh` | systemd units and launchers |
| `vision/detect_cones_video.py` | Post-flight cone detection, tracking and geotagging (see `vision/README.md`) |

Only one process may own the FC serial link. The mission, preflight check,
compass calibration and recorder share a lock file
(`/tmp/ardupilot_mission.lock`) and refuse to run together.

## Further reading

- `docs/KNOWN_GOOD_FLIGHT_RUNBOOK.md`: field procedure and step-by-step recovery
- `docs/FLIGHT_RECORDER.md`: recording manual diagnostic flights
- `docs/PROJECT_STATUS.txt`: hardware notes and the compass/EKF history
- `docs/WHY_THE_PATTERN_FAILED.md`: analysis of the 2026-09-15 raster flight and the fixes
- `docs/LQR_JERK_SNAP_DATA_REQUIREMENTS.md`: data needed to add a jerk/snap-limited trajectory and LQR tracking
