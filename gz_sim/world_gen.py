#!/usr/bin/env python3
"""
Build a randomised NIDAR RescueSwarm Mission 1 world for Gazebo Harmonic.

Writes into --out (default gz_sim/generated/):

    nidar_field.sdf      world: ground, field boundary, launch pad, POIs, distractors, drones
    models/nidar_drone_<i>/
                         one iris per SITL instance (FDM port 9002 + 10*i), each with a
                         downward camera, a matching ground-truth bounding-box camera and
                         a downward 1-D lidar wired into the ArduPilot plugin as rangefinder 1
    mission_area.json    the "mission file": search polygon + geofence as GPS corners
    ground_truth.json    exact position of every POI, for scoring the geotags afterwards

Points of interest (POIs) are placed uniformly at random inside the search polygon,
with a minimum spacing, and mix three kinds:

    person_lying      Rescue Randy lying on the ground          (label 1 = person)
    person_standing   Rescue Randy / standing person             (label 1 = person)
    hardhat           a hard hat on its own on the ground        (label 2 = hardhat)

Persons wear a hard hat with probability --hat-prob, so the camera sees persons alone,
persons with hats, and lone hats. Labels feed the bounding-box camera used by the
"gt" detector; the plain RGB camera is what a YOLO model sees.

    python3 world_gen.py                       # random seed each run
    python3 world_gen.py --seed 7 --pois 10 --field 316x316
"""

import argparse
import json
import math
import random
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
R_EARTH_M = 6378137.0          # same constant as mission/raster_plan_preview.py

LABEL_PERSON = 1
LABEL_HARDHAT = 2

# Camera / lidar geometry shared with the mission (it reads these back from
# mission_area.json so the two never disagree).
CAMERA = {"width": 640, "height": 480, "hfov_rad": 1.20, "rate_hz": 10}
LIDAR = {"min_m": 0.05, "max_m": 40.0, "rate_hz": 20}

HAT_COLOURS = {
    "yellow": (1.00, 0.82, 0.05),
    "orange": (1.00, 0.45, 0.05),
    "white": (0.95, 0.95, 0.95),
    "red": (0.85, 0.08, 0.08),
    "blue": (0.10, 0.30, 0.85),
}


# ---------------------------------------------------------------------------
# Geometry helpers. Gazebo world frame is ENU: x = east, y = north.
# ---------------------------------------------------------------------------
def enu_to_gps(x_east, y_north, origin):
    lat0, lon0 = origin["lat"], origin["lon"]
    lat = lat0 + math.degrees(y_north / R_EARTH_M)
    lon = lon0 + math.degrees(x_east / (R_EARTH_M * math.cos(math.radians(lat0))))
    return round(lat, 8), round(lon, 8)


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def field_corners(width, height, gap, heading_deg):
    """Search rectangle in perimeter order A->B->C->D, rotated about the launch pad.

    A is the south-west corner. width runs east, height runs north, and the
    whole field sits `gap` metres north of the launch pad at the origin.
    """
    yaw = math.radians(-heading_deg)          # compass heading -> ENU rotation
    local = [(-width / 2, gap), (width / 2, gap), (width / 2, gap + height), (-width / 2, gap + height)]
    return [rotate(x, y, yaw) for x, y in local], yaw


def to_field_local(x, y, yaw):
    return rotate(x, y, -yaw)


# ---------------------------------------------------------------------------
# SDF snippets
# ---------------------------------------------------------------------------
def label_plugin(label):
    return (f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
            f'<label>{label}</label></plugin>')


def hardhat_model(name, pose, colour):
    r, g, b = HAT_COLOURS[colour]
    px, py, pz, roll, pitch, yaw = pose
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{px:.3f} {py:.3f} {pz:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <visual name="shell">
          <geometry><ellipsoid><radii>0.150 0.125 0.130</radii></ellipsoid></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.4 0.4 0.4 1</specular></material>
        </visual>
        <visual name="ridge">
          <pose>0 0 0.105 0 0 0</pose>
          <geometry><box><size>0.22 0.035 0.05</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
        <visual name="brim">
          <pose>0.035 0 0.0 0 0 0</pose>
          <geometry><cylinder><radius>0.175</radius><length>0.012</length></cylinder></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
      {label_plugin(LABEL_HARDHAT)}
    </model>"""


def person_model(name, mesh, pose):
    px, py, pz, roll, pitch, yaw = pose
    return f"""
    <include>
      <uri>model://{mesh}</uri>
      <name>{name}</name>
      <pose>{px:.3f} {py:.3f} {pz:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      {label_plugin(LABEL_PERSON)}
    </include>"""


def distractor_model(name, kind, x, y, yaw, rng):
    if kind == "tarp":
        size, z, rgb = f"{rng.uniform(1.5, 3):.2f} {rng.uniform(1.5, 3):.2f} 0.02", 0.01, (0.10, 0.25, 0.75)
    elif kind == "crate":
        s = rng.uniform(0.5, 1.0)
        size, z, rgb = f"{s:.2f} {s:.2f} {s:.2f}", s / 2, (0.55, 0.40, 0.20)
    elif kind == "debris":
        size, z, rgb = f"{rng.uniform(1, 3):.2f} {rng.uniform(0.2, 0.5):.2f} 0.15", 0.075, (0.35, 0.33, 0.30)
    else:  # bush
        r = rng.uniform(0.8, 1.6)
        return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {r * 0.5:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link"><visual name="v">
        <geometry><ellipsoid><radii>{r:.2f} {r:.2f} {r * 0.6:.2f}</radii></ellipsoid></geometry>
        <material><ambient>0.12 0.35 0.10 1</ambient><diffuse>0.12 0.35 0.10 1</diffuse></material>
      </visual></link>
    </model>"""
    r, g, b = rgb
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{size}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>
      </link>
    </model>"""


def boundary_lines(corners, name, rgb, width=0.25):
    """Flat painted strips along a polygon (visual only)."""
    r, g, b = rgb
    out = [f'    <model name="{name}"><static>true</static><link name="link">']
    for i, (x0, y0) in enumerate(corners):
        x1, y1 = corners[(i + 1) % len(corners)]
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        out.append(
            f'      <visual name="edge{i}"><pose>{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} 0.005 0 0 {yaw:.4f}</pose>'
            f'<geometry><box><size>{length:.3f} {width} 0.01</size></box></geometry>'
            f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>')
    out.append("    </link></model>")
    return "\n".join(out)


DRONE_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <include>
      <uri>model://iris_with_standoffs</uri>
    </include>

    <!-- Downward sensor bay, rigidly fixed to the airframe (no gimbal). Image up = nose. -->
    <link name="sensor_bay">
      <pose>0 0 0.02 0 0 0</pose>
      <inertial><mass>0.02</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>

      <sensor name="down_camera" type="camera">
        <pose>0 0 0 0 1.5708 0</pose>
        <always_on>1</always_on>
        <update_rate>{cam_rate}</update_rate>
        <topic>/nidar/drone{idx}/camera</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>200</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>

      <!-- Same optics as down_camera: emits ground-truth boxes for labelled models. -->
      <sensor name="down_bbox" type="boundingbox_camera">
        <pose>0 0 0 0 1.5708 0</pose>
        <always_on>1</always_on>
        <update_rate>{cam_rate}</update_rate>
        <topic>/nidar/drone{idx}/boxes</topic>
        <camera>
          <box_type>visible_2d</box_type>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height></image>
          <clip><near>0.1</near><far>200</far></clip>
        </camera>
      </sensor>

      <!-- 1-D downward lidar (TF-Luna / TFmini class). Fed to ArduPilot as RNGFND1. -->
      <sensor name="down_lidar" type="gpu_lidar">
        <pose>0 0 -0.01 0 1.5708 0</pose>
        <always_on>1</always_on>
        <update_rate>{lidar_rate}</update_rate>
        <topic>/nidar/drone{idx}/lidar</topic>
        <lidar>
          <scan>
            <horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range><min>{lidar_min}</min><max>{lidar_max}</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.02</stddev></noise>
        </lidar>
      </sensor>
    </link>
    <joint name="sensor_bay_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>sensor_bay</child>
    </joint>

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
{lift_drag}
{joint_force}
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>{fdm_port}</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>iris_with_standoffs::imu_link::imu_sensor</imuName>
      <sensor>
        <type>ray</type>
        <index>1</index>
        <topic>/nidar/drone{idx}/lidar</topic>
      </sensor>
{controls}
    </plugin>
  </model>
</sdf>
"""


def iris_plugin_blocks():
    """Lift-drag, joint-force and motor-control blocks copied from iris_with_ardupilot."""
    import re
    import os
    roots = [Path(p) for p in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if p]
    roots.append(Path.home() / "ardupilot_gazebo" / "models")
    for root in roots:
        src = root / "iris_with_ardupilot" / "model.sdf"
        if src.exists():
            text = src.read_text()
            break
    else:
        raise SystemExit("iris_with_ardupilot/model.sdf not found; set GZ_SIM_RESOURCE_PATH "
                         "to include ardupilot_gazebo/models")
    lift = "\n".join(re.findall(r'    <plugin filename="gz-sim-lift-drag-system".*?</plugin>', text, re.S))
    force = "\n".join(re.findall(r'    <plugin filename="gz-sim-apply-joint-force-system".*?</plugin>', text, re.S))
    controls = "\n".join(re.findall(r'      <control channel=.*?</control>', text, re.S))
    if not (lift and force and controls.count("<control channel") == 4):
        raise SystemExit(f"could not parse plugin blocks from {src}")
    return lift, force, controls


WORLD_HEAD = """<?xml version="1.0" ?>
<!-- Generated by gz_sim/world_gen.py  seed={seed}  {stamp}. Do not edit; regenerate. -->
<sdf version="1.9">
  <world name="nidar_field">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <scene>
      <ambient>0.6 0.6 0.6</ambient>
      <background>0.70 0.80 0.92</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{alt}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>{sun_x:.2f} {sun_y:.2f} -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
          <material><ambient>0.30 0.42 0.20 1</ambient><diffuse>0.30 0.42 0.20 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>

    <model name="launch_pad">
      <static>true</static>
      <pose>0 0 0.005 0 0 0</pose>
      <link name="link"><visual name="v"><geometry><box><size>12 5 0.01</size></box></geometry>
        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material></visual></link>
    </model>
"""


def place_pois(args, corners_local_w, corners_local_h, gap, yaw, rng):
    """Uniform random POI positions inside the field, honouring min spacing."""
    margin = args.edge_margin
    placed = []
    attempts = 0
    while len(placed) < args.pois:
        attempts += 1
        if attempts > 20000:
            raise SystemExit(f"could only place {len(placed)} POIs with --min-sep {args.min_sep}; "
                             "use a bigger field or a smaller spacing")
        lx = rng.uniform(-corners_local_w / 2 + margin, corners_local_w / 2 - margin)
        ly = rng.uniform(gap + margin, gap + corners_local_h - margin)
        if all(math.hypot(lx - px, ly - py) >= args.min_sep for px, py in placed):
            placed.append((lx, ly))
    return [rotate(lx, ly, yaw) for lx, ly in placed]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=None, help="random seed (default: time-based)")
    ap.add_argument("--pois", type=int, default=8, help="number of points of interest")
    ap.add_argument("--field", default="160x100", help="search area WIDTHxHEIGHT in metres (10 ha = 316x316)")
    ap.add_argument("--heading", type=float, default=0.0, help="rotate the field by this compass heading (deg)")
    ap.add_argument("--gap", type=float, default=12.0, help="launch pad to field edge distance (m)")
    ap.add_argument("--edge-margin", type=float, default=4.0, help="keep POIs this far inside the boundary (m)")
    ap.add_argument("--min-sep", type=float, default=12.0, help="minimum spacing between POIs (m)")
    ap.add_argument("--mix", default="0.45,0.35,0.20",
                    help="fractions person_lying,person_standing,hardhat")
    ap.add_argument("--hat-prob", type=float, default=0.5, help="chance that a person wears a hard hat")
    ap.add_argument("--distractors", type=int, default=12, help="non-POI clutter objects")
    ap.add_argument("--drones", type=int, default=2)
    ap.add_argument("--origin", default="12.971600,77.594600,920",
                    help="lat,lon,alt of the launch pad (Gazebo world origin == SITL home)")
    ap.add_argument("--out", default=str(HERE / "generated"))
    args = ap.parse_args()

    seed = args.seed if args.seed is not None else int(time.time()) % 100000
    rng = random.Random(seed)
    lat0, lon0, alt0 = (float(v) for v in args.origin.split(","))
    origin = {"lat": lat0, "lon": lon0, "alt": alt0}
    width, height = (float(v) for v in args.field.lower().split("x"))
    mix = [float(v) for v in args.mix.split(",")]

    out = Path(args.out)
    (out / "models").mkdir(parents=True, exist_ok=True)

    corners, yaw = field_corners(width, height, args.gap, args.heading)
    # Geofence: field grown by 15 m on every side, stretched south to take in the launch pad.
    fence_local = [(-width / 2 - 15, -10), (width / 2 + 15, -10),
                   (width / 2 + 15, args.gap + height + 15), (-width / 2 - 15, args.gap + height + 15)]
    fence = [rotate(x, y, yaw) for x, y in fence_local]

    # ---- POIs -------------------------------------------------------------
    positions = place_pois(args, width, height, args.gap, yaw, rng)
    kinds = rng.choices(["person_lying", "person_standing", "hardhat"], weights=mix, k=len(positions))
    poi_sdf, truth = [], []
    for i, ((x, y), kind) in enumerate(zip(positions, kinds)):
        heading = rng.uniform(-math.pi, math.pi)
        colour = rng.choice(list(HAT_COLOURS))
        entry = {"id": f"poi_{i}", "kind": kind, "hardhat": False}
        if kind == "hardhat":
            poi_sdf.append(hardhat_model(f"poi_{i}_hardhat", (x, y, 0.0, 0, 0, heading), colour))
            entry.update(hardhat=True, hat_colour=colour)
            cx, cy = x, y
        elif kind == "person_lying":
            # Randy is modelled standing along +z with his feet at the origin. Rolling by
            # -90 deg lays him on his back with his head along the model's +y axis.
            direction = rotate(0.0, 1.0, heading)
            poi_sdf.append(person_model(f"poi_{i}_person", "rescue_randy",
                                        (x, y, 0.18, -math.pi / 2, 0, heading)))
            cx, cy = x + 0.9 * direction[0], y + 0.9 * direction[1]
            if rng.random() < args.hat_prob:
                hx, hy = x + 1.70 * direction[0], y + 1.70 * direction[1]
                poi_sdf.append(hardhat_model(f"poi_{i}_hardhat", (hx, hy, 0.18, -math.pi / 2, 0, heading), colour))
                entry.update(hardhat=True, hat_colour=colour)
        else:
            mesh = rng.choice(["rescue_randy", "standing_person"])
            poi_sdf.append(person_model(f"poi_{i}_person", mesh, (x, y, 0.0, 0, 0, heading)))
            cx, cy = x, y
            entry["mesh"] = mesh
            if rng.random() < args.hat_prob:
                top = 1.71 if mesh == "rescue_randy" else 1.80
                hx, hy = (x, y) if mesh == "rescue_randy" else (x + rotate(0, -0.03, heading)[0],
                                                                y + rotate(0, -0.03, heading)[1])
                poi_sdf.append(hardhat_model(f"poi_{i}_hardhat", (hx, hy, top, 0, 0, heading), colour))
                entry.update(hardhat=True, hat_colour=colour)
        lat, lon = enu_to_gps(cx, cy, origin)
        entry.update(x_east=round(cx, 3), y_north=round(cy, 3), lat=lat, lon=lon)
        truth.append(entry)

    # ---- clutter that is not a POI (false-positive bait for a real detector) -
    clutter = []
    for k in range(args.distractors):
        lx = rng.uniform(-width / 2, width / 2)
        ly = rng.uniform(args.gap, args.gap + height)
        x, y = rotate(lx, ly, yaw)
        if any(math.hypot(x - t["x_east"], y - t["y_north"]) < 5 for t in truth):
            continue
        clutter.append(distractor_model(f"clutter_{k}", rng.choice(["tarp", "crate", "debris", "bush"]),
                                        x, y, rng.uniform(0, math.pi), rng))

    # ---- drones -------------------------------------------------------------
    lift, force, controls = iris_plugin_blocks()
    drone_poses, drone_sdf = [], []
    for i in range(args.drones):
        name = f"nidar_drone_{i}"
        mdir = out / "models" / name
        mdir.mkdir(parents=True, exist_ok=True)
        (mdir / "model.config").write_text(
            f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
            f'<sdf version="1.9">model.sdf</sdf></model>\n')
        (mdir / "model.sdf").write_text(DRONE_SDF.format(
            name=name, idx=i, fdm_port=9002 + 10 * i, cam_rate=CAMERA["rate_hz"], hfov=CAMERA["hfov_rad"],
            w=CAMERA["width"], h=CAMERA["height"], lidar_rate=LIDAR["rate_hz"], lidar_min=LIDAR["min_m"],
            lidar_max=LIDAR["max_m"], lift_drag=lift, joint_force=force, controls=controls))
        dx = (i - (args.drones - 1) / 2) * 4.0
        px, py = rotate(dx, 0.0, yaw)
        drone_poses.append({"x_east": round(px, 3), "y_north": round(py, 3)})
        # Gazebo yaw 90deg - heading so the nose points along the field's "north" edge.
        drone_sdf.append(f"""
    <include>
      <uri>model://{name}</uri>
      <name>drone{i}</name>
      <pose>{px:.3f} {py:.3f} 0.195 0 0 {math.pi / 2 + yaw:.4f}</pose>
    </include>""")

    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    sun = rotate(-0.4, 0.2, rng.uniform(-math.pi, math.pi))
    world = [WORLD_HEAD.format(seed=seed, stamp=stamp, lat=lat0, lon=lon0, alt=alt0, sun_x=sun[0], sun_y=sun[1])]
    world.append(boundary_lines(corners, "search_area_boundary", (0.95, 0.85, 0.10)))
    world.append(boundary_lines(fence, "geofence_boundary", (0.85, 0.10, 0.10), width=0.4))
    world += poi_sdf + clutter + drone_sdf
    world.append("  </world>\n</sdf>\n")
    (out / "nidar_field.sdf").write_text("\n".join(world))

    mission = {
        "generated": stamp,
        "seed": seed,
        "origin": origin,
        "search_area": [list(enu_to_gps(x, y, origin)) for x, y in corners],
        "geofence": [list(enu_to_gps(x, y, origin)) for x, y in fence],
        "launch": [dict(p, lat=enu_to_gps(p["x_east"], p["y_north"], origin)[0],
                        lon=enu_to_gps(p["x_east"], p["y_north"], origin)[1]) for p in drone_poses],
        "field_heading_deg": args.heading,
        "camera": CAMERA,
        "lidar": LIDAR,
        "labels": {str(LABEL_PERSON): "person", str(LABEL_HARDHAT): "hardhat"},
    }
    (out / "mission_area.json").write_text(json.dumps(mission, indent=2) + "\n")
    (out / "ground_truth.json").write_text(json.dumps({"seed": seed, "origin": origin, "pois": truth}, indent=2) + "\n")

    counts = {k: kinds.count(k) for k in ("person_lying", "person_standing", "hardhat")}
    hats = sum(t["hardhat"] for t in truth)
    print(f"[world_gen] seed={seed} field={width:.0f}x{height:.0f} m ({width * height / 1e4:.2f} ha) "
          f"POIs={len(truth)} {counts} with-hardhat={hats} clutter={len(clutter)}")
    print(f"[world_gen] wrote {out}/nidar_field.sdf, mission_area.json, ground_truth.json")


if __name__ == "__main__":
    main()
