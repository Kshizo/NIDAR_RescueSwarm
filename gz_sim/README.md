# Mission 1 Gazebo simulation: two-drone lawnmower, detection, dip and geotag

Simulates NIDAR RescueSwarm Mission 1 without payloads. It uses the same stack as the
real aircraft: ArduCopter in GUIDED, driven over MAVLink by `pymavlink`, and the
lawnmower comes from `mission/raster_plan_preview.py`.

- **World** (`world_gen.py`): POIs are placed at random on every launch. Each POI is a
  person lying down (Rescue Randy), a standing person, or a hard hat on its own, and
  persons wear a hard hat 50% of the time. The world also has clutter that is not a POI
  (tarps, crates, debris, bushes), the search-area and geofence lines, and a launch pad.
- **Drones** (`generated/models/nidar_drone_<i>`): Iris airframes. Each has a fixed
  downward camera (640×480, 69° HFOV) and a matching ground-truth bounding-box camera.
  Each also has a downward 1-D lidar that the ArduPilot plugin feeds to the FC as
  `RNGFND1` (type 100). The FC streams it back as `DISTANCE_SENSOR`, as a real TF-Luna
  wired to the FC would.
- **Mission** (`swarm_mission.py`): one process runs every drone, so it is a single
  mission system. It does the following:
  1. Splits the search polygon into one sector per drone and plans each sector's
     lawnmower. Pass spacing comes from the camera footprint at survey height minus
     25% sidelap.
  2. Holds altitude on the lidar: `vz = Kz · (target AGL − range·cosφ·cosθ)`, using a
     5-sample median. Drones fly 6 m and 7 m AGL (`--alt`, `--alt-step`).
  3. Flies each pass with along-track speed and cross-track correction. Heading is
     fixed along the passes, so the wide side of the image is across track.
  4. Runs the detector on every frame and projects each box centre to the ground. The
     projection uses the attitude and lidar height. A detection seen in 2 frames becomes
     a POI in a registry the drones share.
  5. The drone that found the POI breaks off the lawnmower and runs
     **APPROACH → CENTER → DESCEND to 3 m → DROP hold 3 s → CLIMB**. The drop is only
     simulated and nothing is released. The drone then geotags the POI from the median
     of the level, still "nadir" frames it took on top of it. Finally it **REJOINS**
     the lawnmower at the exact point where it left.
  6. A POI that another drone has already claimed is only re-observed, never visited
     twice.
  7. At the end, each drone returns to its pad and lands (ArduCopter LAND).

## Run

```bash
cd gz_sim
./launch_sim.sh                       # terminal 1: new random world + Gazebo + 2 SITL
python3 swarm_mission.py              # terminal 2: fly the mission (live view window)
```

The live view shows both camera feeds with their boxes, plus a map with the lawnmowers,
the tracks and the POIs. Press `q` to send every drone home.

Each run writes these files to `runs/<time>/`:

| File | Contents |
| --- | --- |
| `geotags.csv` | each POI: classes, lat/lon, finder, times, nadir frame count, matched ground-truth id and error (m) |
| `map.png` | planned vs flown lawnmower, ground truth, geotags |
| `mission.geojson` | the same, ready for geojson.io / QGIS |
| `tracks.csv` | flown tracks, lat/lon/AGL/state at 10 Hz |
| `summary.txt` | found / missed / false positives, geotag error, time |
| `mission.log` | the console log |

### Options

| | |
| --- | --- |
| `SEED=7 ./launch_sim.sh` | reproducible world |
| `WORLD_ARGS="--pois 10 --field 316x316" ./launch_sim.sh` | full 10 ha search area, 10 POIs |
| `WORLD_ARGS="--heading 30 --hat-prob 0.8"` | rotated field, more hard hats (see `world_gen.py -h`) |
| `HEADLESS=1 ./launch_sim.sh` | no Gazebo window |
| `NO_REGEN=1 ./launch_sim.sh` | reuse the last world |
| `swarm_mission.py --alt 6 --speed 8 --drop-agl 3 --drop-hold 5` | survey height, speed, dip height and hold |
| `swarm_mission.py --detector yolo --weights best.pt --conf 0.4` | use a real model instead of ground truth |
| `swarm_mission.py --dump-dataset ~/nidar_sim_ds` | save frames + YOLO labels (person=0, hardhat=1) while flying |
| `swarm_mission.py --plan-only` | print both lawnmower plans and exit |

To watch the swarm in QGroundControl or MAVProxy, connect to `tcp:127.0.0.1:5762` for
drone0 and `5772` for drone1. Geotags are also sent there as STATUSTEXT.

## Detectors

- **`gt`** (the default) reads Gazebo's bounding-box camera, so its labels are perfect.
  Use it to test the flight logic: the lawnmower, the interrupt and resume, the dip, the
  geotag maths and the deconfliction between drones.
- **`yolo`** runs Ultralytics on the RGB frame. Classes named person/people/pedestrian/human
  count as person, and classes named hat/helmet count as hardhat. COCO `yolov8n.pt`
  has no hard-hat class and does poorly on top-down views. Train your own model
  instead: generate data with `--dump-dataset`, which already writes a `data.yaml`,
  ideally mixed with real aerial data (VisDrone/HERIDAL in `~/nidar/datasets`).

## Survey height and the 30-minute limit

At 6 m AGL the 69° camera sees about 8 × 6 m of ground, so passes are only about 6 m
apart. On the full 10 ha (316 × 316 m) with 2 drones, each drone has about 8.3 km of
lawnmower:

| speed | longest sector, lawnmower only |
| --- | --- |
| 5 m/s | ~28 min (over budget once dips, transit and landing are added) |
| 8 m/s | ~17 min |
| 10 m/s | ~14 min |

The ways to buy time are a wider camera FOV, a higher survey height, less sidelap,
more speed, or a third drone. `--plan-only` prints the timing for any combination
and warns when one sector alone uses more than 80% of `--time-limit`. Set
`CAMERA["hfov_rad"]` in `world_gen.py` to your real camera's FOV so the footprint,
and with it the pass spacing, matches the aircraft.

## Known simplifications

- SITL GPS is perfect by default, so the geotag error in sim is centimetres. Add
  `SIM_GPS1_NOISE` / `SIM_GPS1_GLTCH_*` to `params/nidar_copter.parm` to see the effect
  of real GPS (expect 1–2 m).
- The lidar reads whatever is under it. Passing over a standing person reads about 1.8 m
  short for a moment, and the 5-sample median only partly hides this, as on the real
  aircraft.
- Deconfliction comes from disjoint sectors plus a 3 m altitude step. There is no
  avoidance between drones.
- The ground is flat. To exercise terrain following on the lidar, add slopes to the world.

## Requirements

These were already set up on this machine: ArduPilot SITL (`~/ardupilot`, `./waf copter`),
`ardupilot_gazebo` built at `~/ardupilot_gazebo/build`, Gazebo Harmonic with the Python
bindings (`gz.transport13`, `gz.msgs10`), and `pymavlink`, `opencv-python`, `numpy`,
`matplotlib` (plus `ultralytics` for `--detector yolo`). Paths are set in `env.sh`.
The human meshes in `models/` are local copies of the OpenRobotics Fuel models
(CC-BY 4.0).
