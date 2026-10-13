#!/usr/bin/env python3
"""
NIDAR RescueSwarm Mission 1 - two-drone scout + rescue simulation (no payload).

One process is the whole mission system: it reads the mission file written by
world_gen.py, splits the search polygon into one sector per drone, builds each
sector's lawnmower with the real planner (mission/raster_plan_preview.py), and
flies every drone over MAVLink in ArduCopter GUIDED with velocity setpoints.

Per drone:
  * Altitude is held on the downward 1-D lidar (RNGFND1 -> DISTANCE_SENSOR), not
    on the barometer: vertical velocity = Kz * (target AGL - tilt-corrected range).
  * The downward camera feeds a person / hard-hat detector. A new detection seen in
    2+ frames becomes a point of interest (POI) in a registry shared by all drones.
  * The finder breaks off the lawnmower, flies over the POI, centres on it, descends
    to the drop height, holds (where the kit would be released), climbs back, and
    geotags it from the frames taken while hovering on top. Then it rejoins the
    lawnmower exactly where it left it.
  * A POI another drone has already claimed is only re-observed, never re-visited.

At the end both drones return to their launch pads and land, and a report goes to
runs/<time>/: geotags.csv, mission.geojson, map.png, tracks.csv, summary.txt.

    python3 swarm_mission.py                       # gt detector, live view
    python3 swarm_mission.py --detector yolo --weights best.pt
    python3 swarm_mission.py --no-gui --dump-dataset /tmp/nidar_ds
"""

import argparse
import json
import math
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from pymavlink import mavutil

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "mission"))
import raster_plan_preview as rpp  # noqa: E402  the same planner the real aircraft uses

from detector import CameraFeed, DatasetDumper, make_detector  # noqa: E402

# The flight planner's polygon limits were set for small test plots; the sim
# searches up to 10 ha, so lift the side cap (geometry checks are unchanged).
rpp.RASTER_MAX_SIDE_M = 2000.0

LOOP_HZ = 10.0
VEL_MASK = 0b0000_1001_1100_0111      # use vx, vy, vz and yaw; ignore pos, accel, yaw rate


# ---------------------------------------------------------------------------
# Small geometry helpers. Mission frame: metres (north, east) from the launch origin.
# ---------------------------------------------------------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def norm(v):
    return math.hypot(v[0], v[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def rot_body_to_ned(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


def point_in_convex(p, poly):
    """poly: list of (n, e) in either winding."""
    sign = 0
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cross) < 1e-9:
            continue
        s = 1 if cross > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return True


class Frame:
    """GPS <-> mission-frame conversions around one origin."""

    def __init__(self, lat, lon):
        self.ref = rpp.GpsPoint(lat, lon)

    def ne(self, lat, lon):
        p = rpp.gps_to_ne(rpp.GpsPoint(lat, lon), self.ref)
        return (p.north, p.east)

    def gps(self, ne):
        return rpp.ne_to_gps(ne[0], ne[1], self.ref)


# ---------------------------------------------------------------------------
# Shared POI registry
# ---------------------------------------------------------------------------
@dataclass
class Poi:
    id: int
    ne: tuple
    classes: set
    found_by: int
    found_t: float
    status: str = "pending"            # pending -> investigating -> delivered | lost
    owner: int = None
    obs: list = field(default_factory=list)          # (ne, phase, cls, t)
    geotag: tuple = None
    delivered_by: int = None
    delivered_t: float = None

    def estimate(self, nadir_only=False):
        pts = [o[0] for o in self.obs if (o[1] == "nadir" or not nadir_only)]
        if not pts:
            pts = [o[0] for o in self.obs] or [self.ne]
        return (statistics.median(p[0] for p in pts), statistics.median(p[1] for p in pts))


class Registry:
    def __init__(self, merge_radius):
        self.merge_radius = merge_radius
        self.pois = []
        self.lock = threading.Lock()

    def nearest(self, ne, radius=None):
        radius = self.merge_radius if radius is None else radius
        best, best_d = None, radius
        for p in self.pois:
            if p.status == "lost":
                continue
            d = norm(sub(p.estimate(), ne))
            if d <= best_d:
                best, best_d = p, d
        return best

    def add(self, ne, cls, drone, t):
        poi = Poi(len(self.pois), ne, {cls}, drone, t)
        self.pois.append(poi)
        return poi


# ---------------------------------------------------------------------------
# One aircraft
# ---------------------------------------------------------------------------
class Drone:
    def __init__(self, idx, args, frame, mission, plan, sector, registry, log):
        self.i = idx
        self.args = args
        self.frame = frame
        self.registry = registry
        self.log = log
        self.plan = plan
        self.sector = sector
        self.survey_agl = args.alt + idx * args.alt_step
        self.yaw_deg = math.degrees(math.atan2(plan.u_hat[1], plan.u_hat[0])) % 360
        launch = mission["launch"][idx]
        self.home_ne = frame.ne(launch["lat"], launch["lon"])
        cam = mission["camera"]
        self.cam_w, self.cam_h = cam["width"], cam["height"]
        self.fx = (cam["width"] / 2) / math.tan(cam["hfov_rad"] / 2)
        self.lidar_max = mission["lidar"]["max_m"]

        # Lawnmower legs in the mission frame: passes and the transitions between them.
        wps = [frame.ne(w.lat, w.lon) for w in plan.waypoints]
        self.legs = [(wps[k], wps[k + 1]) for k in range(len(wps) - 1)]
        self.lawn_pts = wps
        self.leg = 0
        self.leg_start_override = None

        self.state = "INIT"
        self.state_t = time.time()
        self.target = None                 # Poi being investigated
        self.resume_ne = None
        self.hold_ok_since = None
        self.last_seen_t = 0.0
        self.done = False
        self.track = []                    # (t, n, e, agl, state)
        self.candidates = []               # [ne, cls, hits, first_t, last_t]
        self.last_frame_seq = -1
        self.last_dets = []
        self.dips = 0

        # telemetry
        self.t = {"lat": None, "lon": None, "vn": 0.0, "ve": 0.0, "rel_alt": 0.0, "roll": 0.0,
                  "pitch": 0.0, "yaw": 0.0, "range": None, "range_t": 0.0, "armed": False,
                  "mode": "", "batt": None}
        self.ranges = deque(maxlen=5)
        self.lidar_fallback_logged = False

        port = 5760 + 10 * idx
        self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=240 + idx,
                                              source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.log(f"[drone{idx}] waiting for heartbeat on tcp:127.0.0.1:{port}")
        if self.mav.wait_heartbeat(timeout=60) is None or self.mav.target_system == 0:
            raise SystemExit(f"[drone{idx}] no heartbeat on tcp:127.0.0.1:{port}. Is launch_sim.sh running, "
                             "and is no other swarm_mission.py already connected? SITL takes one client per port.")
        self.sys = self.mav.target_system
        for msg_id, hz in ((33, 20), (30, 20), (132, 20), (1, 2), (24, 2)):
            self.mav.mav.command_long_send(self.sys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                           msg_id, 1e6 / hz, 0, 0, 0, 0, 0)
        threading.Thread(target=self._rx, daemon=True).start()
        self.log(f"[drone{idx}] sysid {self.sys}, sector {plan.pass_count} passes "
                 f"{plan.path_length_m:.0f} m, AGL {self.survey_agl:.0f} m, heading {self.yaw_deg:.0f} deg")

    # ---- telemetry --------------------------------------------------------
    def _rx(self):
        while True:
            m = self.mav.recv_match(blocking=True, timeout=1.0)
            if m is None:
                continue
            k = m.get_type()
            t = self.t
            if k == "GLOBAL_POSITION_INT":
                t["lat"], t["lon"] = m.lat * 1e-7, m.lon * 1e-7
                t["vn"], t["ve"] = m.vx / 100.0, m.vy / 100.0
                t["rel_alt"] = m.relative_alt / 1000.0
            elif k == "ATTITUDE":
                t["roll"], t["pitch"], t["yaw"] = m.roll, m.pitch, m.yaw
            elif k == "DISTANCE_SENSOR" and m.orientation == 25:
                r = m.current_distance / 100.0
                if m.min_distance / 100.0 < r < m.max_distance / 100.0 - 0.05:
                    self.ranges.append(r)
                    t["range"], t["range_t"] = r, time.time()
            elif k == "HEARTBEAT" and m.get_srcSystem() == self.sys and m.get_srcComponent() == 1:
                t["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                t["mode"] = mavutil.mode_string_v10(m)
            elif k == "SYS_STATUS":
                t["batt"] = m.battery_remaining
            elif k == "STATUSTEXT" and m.get_srcSystem() == self.sys:
                text = m.text.rstrip("\x00")
                if self.args.verbose or "PreArm" in text and self.state == "ARMING" and \
                        time.time() - self.state_t > 30:
                    self.log(f"[drone{self.i}] FC: {text}")

    @property
    def pos(self):
        return self.frame.ne(self.t["lat"], self.t["lon"])

    def agl(self):
        """Tilt-corrected height above whatever the lidar sees, median of the last 5 returns."""
        t = self.t
        if self.ranges and time.time() - t["range_t"] < 0.5:
            return statistics.median(self.ranges) * math.cos(t["roll"]) * math.cos(t["pitch"])
        if not self.lidar_fallback_logged and self.state not in ("INIT", "ARMING", "TAKEOFF", "LAND", "DONE"):
            self.log(f"[drone{self.i}] WARNING lidar has no return; holding altitude on the barometer")
            self.lidar_fallback_logged = True
        return t["rel_alt"]

    # ---- commands ---------------------------------------------------------
    def set_mode(self, name):
        self.mav.set_mode(self.mav.mode_mapping()[name])

    def arm(self):
        self.mav.mav.command_long_send(self.sys, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 0, 0, 0, 0, 0, 0)

    def takeoff(self, alt):
        self.mav.mav.command_long_send(self.sys, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                       0, 0, 0, 0, 0, 0, 0, alt)

    def send_velocity(self, vn, ve, vd):
        self.mav.mav.set_position_target_local_ned_send(
            0, self.sys, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_MASK,
            0, 0, 0, vn, ve, vd, 0, 0, 0, math.radians(self.yaw_deg), 0)

    def statustext(self, text):
        self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, text[:50].encode())

    # ---- control laws -----------------------------------------------------
    def vz_for(self, target_agl):
        err = target_agl - self.agl()
        return -clamp(self.args.kz * err, -self.args.vz_max, self.args.vz_max)   # NED: down is +

    def goto_cmd(self, target, vmax):
        d = sub(target, self.pos)
        dist = norm(d)
        speed = min(vmax, math.sqrt(2 * self.args.accel * dist), self.args.kp_xy * dist)
        if dist < 1e-3:
            return (0.0, 0.0), dist
        return (d[0] / dist * speed, d[1] / dist * speed), dist

    def leg_cmd(self, start, end, speed):
        """Along-track speed with a slowdown into the end point, plus cross-track correction."""
        seg = sub(end, start)
        length = norm(seg)
        if length < 1e-3:
            return self.goto_cmd(end, speed)
        u = (seg[0] / length, seg[1] / length)
        n = (-u[1], u[0])
        rel = sub(self.pos, start)
        along = rel[0] * u[0] + rel[1] * u[1]
        cross = rel[0] * n[0] + rel[1] * n[1]
        remaining = length - along
        v_along = clamp(min(speed, math.sqrt(2 * self.args.accel * max(remaining, 0.0)) + 0.3), 0.3, speed)
        v_cross = clamp(-self.args.kp_xy * cross, -2.0, 2.0)
        return (u[0] * v_along + n[0] * v_cross, u[1] * v_along + n[1] * v_cross), remaining

    # ---- perception -------------------------------------------------------
    def pixel_to_ne(self, u, v):
        """Ray through pixel (u, v) of the fixed nadir camera, intersected with the ground."""
        t = self.t
        ray_body = np.array([-(v - (self.cam_h - 1) / 2) / self.fx, (u - (self.cam_w - 1) / 2) / self.fx, 1.0])
        ray = rot_body_to_ned(t["roll"], t["pitch"], t["yaw"]) @ ray_body
        if ray[2] < 0.2:
            return None
        h = self.agl()
        n0, e0 = self.pos
        return (n0 + h / ray[2] * ray[0], e0 + h / ray[2] * ray[1])

    def perceive(self, feed, detector, dumper):
        seq, frame = feed.latest()
        if frame is None or seq == self.last_frame_seq or not detector.ready(frame):
            return
        self.last_frame_seq = seq
        self.last_image = frame.image
        if dumper:
            dumper.maybe_save(self.i, frame)
        if self.state not in ("SURVEY", "TRANSIT", "REJOIN", "APPROACH", "CENTER", "DESCEND", "DROP", "CLIMB"):
            self.last_dets = []
            return
        dets = detector.detect(frame)
        self.last_dets = dets
        now = time.time()
        speed = norm((self.t["vn"], self.t["ve"]))
        tilt = max(abs(self.t["roll"]), abs(self.t["pitch"]))
        phase = "nadir" if (self.state in ("CENTER", "DESCEND", "DROP") and speed < 0.6 and tilt < 0.06) else "pass"
        for det in dets:
            ne = self.pixel_to_ne(det.u, det.v)
            if ne is None:
                continue
            with self.registry.lock:
                poi = self.registry.nearest(ne)
                if poi is not None:
                    poi.obs.append((ne, phase, det.cls, now))
                    poi.classes.add(det.cls)
                    if poi is self.target:
                        self.last_seen_t = now
                    continue
            self._candidate(ne, det.cls, now)
        self.candidates = [c for c in self.candidates if now - c[4] < 1.5]

    def _candidate(self, ne, cls, now):
        for c in self.candidates:
            if norm(sub(c[0], ne)) < self.registry.merge_radius:
                c[0] = ((c[0][0] * c[2] + ne[0]) / (c[2] + 1), (c[0][1] * c[2] + ne[1]) / (c[2] + 1))
                c[1].add(cls)
                c[2] += 1
                c[4] = now
                if c[2] >= self.args.confirm_frames:
                    with self.registry.lock:
                        if self.registry.nearest(c[0]) is None:
                            poi = self.registry.add(c[0], cls, self.i, now)
                            poi.classes |= c[1]
                            poi.obs.append((c[0], "pass", cls, now))
                            lat, lon = self.frame.gps(poi.ne)
                            self.log(f"[drone{self.i}] DETECTED POI {poi.id} {'+'.join(sorted(poi.classes))} "
                                     f"near {lat:.7f}, {lon:.7f}")
                            self.statustext(f"POI{poi.id} found d{self.i}")
                    self.candidates.remove(c)
                return
        self.candidates.append([ne, {cls}, 1, now, now])

    # ---- state machine ----------------------------------------------------
    def go(self, state):
        if state != self.state:
            self.log(f"[drone{self.i}] {self.state} -> {state}")
        self.state, self.state_t = state, time.time()
        self.hold_ok_since = None

    def next_job(self):
        """Nearest pending POI this drone found, else nothing."""
        with self.registry.lock:
            mine = [p for p in self.registry.pois if p.status == "pending" and p.found_by == self.i]
            if not mine:
                return None
            poi = min(mine, key=lambda p: norm(sub(p.estimate(), self.pos)))
            poi.status, poi.owner = "investigating", self.i
            return poi

    def step(self, mission_start, deadline):
        a = self.args
        now = time.time()
        in_state = now - self.state_t
        if self.t["lat"] is None:
            return
        if self.state not in ("INIT", "ARMING", "TAKEOFF", "LAND", "DONE"):
            self.track.append((now - mission_start, *self.pos, self.agl(), self.state))
            if now > deadline and self.state not in ("RETURN",):
                self.log(f"[drone{self.i}] mission time limit reached")
                self.go("RETURN")
            if not point_in_convex(self.pos, self.fence):
                self.log(f"[drone{self.i}] GEOFENCE BREACH at {self.pos}; RTL")
                self.set_mode("RTL")
                self.go("LAND")
                return

        s = self.state
        if s == "INIT":
            self.set_mode("GUIDED")
            self.go("ARMING")
        elif s == "ARMING":
            if self.t["armed"]:
                self.takeoff(min(self.survey_agl, 10.0))
                self.go("TAKEOFF")
            elif in_state > 0 and int(in_state * LOOP_HZ) % 20 == 0:
                if self.t["mode"] != "GUIDED":
                    self.set_mode("GUIDED")
                self.arm()
        elif s == "TAKEOFF":
            if self.t["rel_alt"] > min(self.survey_agl, 10.0) * 0.85:
                self.go("TRANSIT")
            elif in_state > 40:
                self.log(f"[drone{self.i}] takeoff timed out; landing")
                self.go("LAND")
        elif s == "TRANSIT":
            # Climb to survey height first, then fly to the first lawnmower point.
            if abs(self.agl() - self.survey_agl) > 2.0 and in_state < 20:
                self.send_velocity(0, 0, self.vz_for(self.survey_agl))
                return
            v, dist = self.goto_cmd(self.legs[0][0], a.speed)
            self.send_velocity(*v, self.vz_for(self.survey_agl))
            if dist < a.wp_radius:
                self.go("SURVEY")
        elif s == "SURVEY":
            if self.target is None:
                job = self.next_job()
                if job is not None:
                    self.resume_ne = self.pos
                    self.leg_start_override = None
                    self.start_investigation(job)
                    return
            if self.leg >= len(self.legs):
                self.log(f"[drone{self.i}] lawnmower complete")
                self.go("RETURN")
                return
            start, end = self.legs[self.leg]
            if self.leg_start_override is not None:
                start = self.leg_start_override
            v, remaining = self.leg_cmd(start, end, a.speed)
            self.send_velocity(*v, self.vz_for(self.survey_agl))
            if remaining < a.wp_radius:
                self.leg += 1
                self.leg_start_override = None
        elif s == "APPROACH":
            tgt = self.target.estimate()
            v, dist = self.goto_cmd(tgt, a.speed)
            self.send_velocity(*v, self.vz_for(self.survey_agl))
            if dist < 1.5:
                self.go("CENTER")
        elif s == "CENTER":
            # Hold over the live estimate until steady, so the nadir frames that make the
            # geotag are taken level and still.
            tgt = self.target.estimate(nadir_only=len([o for o in self.target.obs if o[1] == "nadir"]) >= 5)
            v, dist = self.goto_cmd(tgt, 2.0)
            self.send_velocity(*v, self.vz_for(self.survey_agl))
            steady = dist < 0.5 and norm((self.t["vn"], self.t["ve"])) < 0.3
            self.hold_ok_since = (self.hold_ok_since or now) if steady else None
            if self.hold_ok_since and now - self.hold_ok_since > 1.5:
                self.go("DESCEND")
            elif now - self.last_seen_t > 6.0 and in_state > 6.0:
                self.log(f"[drone{self.i}] POI {self.target.id} not re-seen from above; marking lost")
                with self.registry.lock:
                    self.target.status = "lost"
                self.finish_investigation()
            elif in_state > 20:
                self.go("DESCEND")
        elif s == "DESCEND":
            tgt = self.target.estimate(nadir_only=True)
            v, _ = self.goto_cmd(tgt, 1.0)
            self.send_velocity(*v, self.vz_for(a.drop_agl))
            if abs(self.agl() - a.drop_agl) < 0.4 or in_state > 25:
                self.go("DROP")
        elif s == "DROP":
            tgt = self.target.estimate(nadir_only=True)
            v, _ = self.goto_cmd(tgt, 1.0)
            self.send_velocity(*v, self.vz_for(a.drop_agl))
            if in_state > a.drop_hold:
                lat, lon = self.frame.gps(self.pos)
                self.log(f"[drone{self.i}] KIT DROP (simulated) over POI {self.target.id} "
                         f"at {lat:.7f}, {lon:.7f}, AGL {self.agl():.1f} m")
                self.go("CLIMB")
        elif s == "CLIMB":
            tgt = self.target.estimate(nadir_only=True)
            v, _ = self.goto_cmd(tgt, 1.0)
            self.send_velocity(*v, self.vz_for(self.survey_agl))
            if abs(self.agl() - self.survey_agl) < 1.0 or in_state > 25:
                with self.registry.lock:
                    p = self.target
                    p.status, p.delivered_by, p.delivered_t = "delivered", self.i, now
                    p.geotag = self.frame.gps(p.estimate(nadir_only=True))
                n_nadir = sum(1 for o in p.obs if o[1] == "nadir")
                self.log(f"[drone{self.i}] GEOTAG POI {p.id} {'+'.join(sorted(p.classes))}: "
                         f"{p.geotag[0]:.7f}, {p.geotag[1]:.7f}  ({n_nadir} nadir frames)")
                self.statustext(f"POI{p.id} {p.geotag[0]:.6f},{p.geotag[1]:.6f}")
                self.dips += 1
                self.finish_investigation()
        elif s == "REJOIN":
            v, dist = self.goto_cmd(self.resume_ne, a.speed)
            self.send_velocity(*v, self.vz_for(self.survey_agl))
            if dist < a.wp_radius:
                self.leg_start_override = self.resume_ne
                self.go("SURVEY")
        elif s == "RETURN":
            v, dist = self.goto_cmd(self.home_ne, a.speed)
            self.send_velocity(*v, self.vz_for(self.survey_agl))
            if dist < 1.0 and norm((self.t["vn"], self.t["ve"])) < 0.5:
                self.set_mode("LAND")
                self.go("LAND")
        elif s == "LAND":
            if not self.t["armed"] and in_state > 2:
                self.log(f"[drone{self.i}] landed and disarmed")
                self.go("DONE")
                self.done = True
            elif in_state > 120:
                self.done = True

    def start_investigation(self, poi):
        self.target = poi
        self.last_seen_t = time.time()
        self.log(f"[drone{self.i}] investigating POI {poi.id}")
        self.go("APPROACH")

    def finish_investigation(self):
        self.target = None
        job = self.next_job()
        if job is not None:
            self.start_investigation(job)
        else:
            self.go("REJOIN")


# ---------------------------------------------------------------------------
# Mission setup
# ---------------------------------------------------------------------------
def split_sectors(corners, n):
    """Split a 4-corner convex polygon A,B,C,D into n strips across its longer side pair."""
    a, b, c, d = corners
    if math.dist(a, b) < math.dist(a, d):        # make AB / DC the long pair
        a, b, c, d = b, c, d, a

    def lerp(p, q, t):
        return (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)

    out = []
    for k in range(n):
        t0, t1 = k / n, (k + 1) / n
        out.append([lerp(a, b, t0), lerp(a, b, t1), lerp(d, c, t1), lerp(d, c, t0)])
    return out


def footprint(agl, mission):
    cam = mission["camera"]
    w = 2 * agl * math.tan(cam["hfov_rad"] / 2)
    return w, w * cam["height"] / cam["width"]


def build_plans(args, mission, frame):
    sectors_gps = split_sectors([tuple(c) for c in mission["search_area"]], args.drones)
    # Give each drone the sector nearest its launch pad.
    launch_ne = [frame.ne(l["lat"], l["lon"]) for l in mission["launch"]]
    centres = [frame.ne(sum(p[0] for p in s) / 4, sum(p[1] for p in s) / 4) for s in sectors_gps]
    order = sorted(range(args.drones), key=lambda k: (centres[k][1], centres[k][0]))
    by_east = sorted(range(args.drones), key=lambda k: launch_ne[k][1])
    assign = {drone: order[rank] for rank, drone in enumerate(by_east)}

    plans, sectors = [], []
    for i in range(args.drones):
        agl = args.alt + i * args.alt_step
        across, _ = footprint(agl, mission)
        spacing = across * (1 - args.sidelap)
        sector = sectors_gps[assign[i]]
        plan = rpp.plan_raster(sector, spacing_m=spacing, inset_m=args.inset, speed_mps=args.speed, axis="long")
        plans.append(plan)
        sectors.append(sector)
    return plans, sectors


# ---------------------------------------------------------------------------
# Live view
# ---------------------------------------------------------------------------
STATE_COLOURS = {"pending": (0, 200, 255), "investigating": (0, 140, 255), "delivered": (0, 220, 0),
                 "lost": (128, 128, 128)}


class LiveView:
    def __init__(self, drones, registry, frame, mission, fence_ne):
        self.drones, self.registry, self.frame = drones, registry, frame
        self.field = [frame.ne(*c) for c in mission["search_area"]]
        self.fence = fence_ne
        pts = self.fence
        self.n_lo, self.n_hi = min(p[0] for p in pts), max(p[0] for p in pts)
        self.e_lo, self.e_hi = min(p[1] for p in pts), max(p[1] for p in pts)
        self.size = 480
        self.scale = (self.size - 20) / max(self.n_hi - self.n_lo, self.e_hi - self.e_lo)
        cv2.namedWindow("NIDAR RescueSwarm - live", cv2.WINDOW_NORMAL)

    def px(self, ne):
        return (int(10 + (ne[1] - self.e_lo) * self.scale), int(self.size - 10 - (ne[0] - self.n_lo) * self.scale))

    def draw(self, elapsed):
        panels = []
        for d in self.drones:
            img = getattr(d, "last_image", None)
            img = np.zeros((d.cam_h, d.cam_w, 3), np.uint8) if img is None else img.copy()
            for det in d.last_dets:
                x0, y0, x1, y1 = map(int, det.box)
                col = (0, 255, 255) if det.cls == "hardhat" else (0, 0, 255)
                cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
                cv2.putText(img, f"{det.cls} {det.conf:.2f}", (x0, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, col, 1)
            cv2.drawMarker(img, (d.cam_w // 2, d.cam_h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
            agl = d.agl() if d.t["lat"] is not None else 0
            tgt = f" -> POI {d.target.id}" if d.target else ""
            for row, text in enumerate([f"drone{d.i}  {d.state}{tgt}",
                                        f"lidar AGL {agl:5.1f} m  leg {min(d.leg + 1, len(d.legs))}/{len(d.legs)}"]):
                cv2.putText(img, text, (8, 22 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(img, text, (8, 22 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            panels.append(cv2.resize(img, (int(d.cam_w * self.size / d.cam_h), self.size)))

        m = np.full((self.size, self.size, 3), (40, 70, 45), np.uint8)
        cv2.polylines(m, [np.int32([self.px(p) for p in self.fence])], True, (40, 40, 200), 1)
        cv2.polylines(m, [np.int32([self.px(p) for p in self.field])], True, (0, 220, 255), 1)
        colours = [(255, 160, 0), (255, 0, 200), (0, 255, 160)]
        for d in self.drones:
            col = colours[d.i % 3]
            cv2.polylines(m, [np.int32([self.px(p) for p in d.lawn_pts])], False, tuple(c // 2 for c in col), 1)
            if len(d.track) > 1:
                cv2.polylines(m, [np.int32([self.px(p[1:3]) for p in d.track[::5]])], False, col, 1)
            if d.t["lat"] is not None:
                cv2.circle(m, self.px(d.pos), 5, col, -1)
        with self.registry.lock:
            for p in self.registry.pois:
                c = STATE_COLOURS[p.status]
                cv2.drawMarker(m, self.px(p.estimate()), c, cv2.MARKER_DIAMOND, 12, 2)
                cv2.putText(m, str(p.id), self.px(p.estimate()), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1)
            n_del = sum(p.status == "delivered" for p in self.registry.pois)
            n_all = sum(p.status != "lost" for p in self.registry.pois)
        cv2.putText(m, f"t={elapsed:5.0f}s  POIs {n_all}  served {n_del}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.imshow("NIDAR RescueSwarm - live", np.hstack(panels + [m]))
        return cv2.waitKey(1) & 0xFF


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def write_report(out, args, mission, truth, frame, drones, registry, sectors, elapsed):
    out.mkdir(parents=True, exist_ok=True)
    gt = [(t, frame.ne(t["lat"], t["lon"])) for t in truth["pois"]]
    pois = [p for p in registry.pois if p.status != "lost"]

    # Greedy one-to-one match of geotags to ground truth.
    pairs = sorted(((norm(sub(p.estimate(True), g_ne)), p.id, k) for p in pois for k, (_, g_ne) in enumerate(gt)))
    match, used_p, used_g = {}, set(), set()
    for dist, pid, k in pairs:
        if dist <= args.match_radius and pid not in used_p and k not in used_g:
            match[pid] = (k, dist)
            used_p.add(pid)
            used_g.add(k)

    rows = ["poi_id,classes,status,lat,lon,found_by,found_s,delivered_by,delivered_s,nadir_frames,"
            "gt_id,gt_kind,gt_hardhat,error_m"]
    t0 = drones[0].mission_start
    for p in pois:
        lat, lon = p.geotag or frame.gps(p.estimate(True))
        k, err = match.get(p.id, (None, None))
        g = gt[k][0] if k is not None else {}
        rows.append(",".join(str(x) for x in [
            p.id, "+".join(sorted(p.classes)), p.status, f"{lat:.8f}", f"{lon:.8f}", f"drone{p.found_by}",
            f"{p.found_t - t0:.1f}", "" if p.delivered_by is None else f"drone{p.delivered_by}",
            "" if p.delivered_t is None else f"{p.delivered_t - t0:.1f}",
            sum(1 for o in p.obs if o[1] == "nadir"), g.get("id", ""), g.get("kind", ""), g.get("hardhat", ""),
            "" if err is None else f"{err:.2f}"]))
    (out / "geotags.csv").write_text("\n".join(rows) + "\n")

    with open(out / "tracks.csv", "w") as f:
        f.write("drone,t_s,lat,lon,agl_m,state\n")
        for d in drones:
            for t, n, e, agl, st in d.track:
                lat, lon = frame.gps((n, e))
                f.write(f"{d.i},{t:.2f},{lat:.8f},{lon:.8f},{agl:.2f},{st}\n")

    errs = [m[1] for m in match.values()]
    missed = [gt[k][0]["id"] for k in range(len(gt)) if k not in used_g]
    false_pos = [p.id for p in pois if p.id not in match]
    lines = [
        f"NIDAR RescueSwarm sim run  seed={mission['seed']}  detector={args.detector}",
        f"mission time            {elapsed:.0f} s",
        f"ground-truth POIs       {len(gt)}",
        f"POIs found              {len(pois)}  (matched {len(match)}, false positives {len(false_pos)})",
        f"POIs served (dip+tag)   {sum(p.status == 'delivered' for p in pois)}",
        f"missed                  {', '.join(missed) or '-'}",
    ]
    if errs:
        lines.append(f"geotag error  mean {statistics.mean(errs):.2f} m  median {statistics.median(errs):.2f} m  "
                     f"max {max(errs):.2f} m")
    for d in drones:
        lines.append(f"drone{d.i}: {d.dips} dips, lawnmower legs {min(d.leg, len(d.legs))}/{len(d.legs)}, "
                     f"plan {d.plan.path_length_m:.0f} m at {d.survey_agl:.0f} m AGL")
    (out / "summary.txt").write_text("\n".join(lines) + "\n")

    # GeoJSON for geojson.io / QGIS
    def ring(pts):
        return [[lon, lat] for lat, lon in list(pts) + [pts[0]]]
    feats = [{"type": "Feature", "properties": {"name": "search area"},
              "geometry": {"type": "Polygon", "coordinates": [ring(mission["search_area"])]}},
             {"type": "Feature", "properties": {"name": "geofence", "stroke": "#d00"},
              "geometry": {"type": "Polygon", "coordinates": [ring(mission["geofence"])]}}]
    for i, (d, s) in enumerate(zip(drones, sectors)):
        feats.append({"type": "Feature", "properties": {"name": f"drone{i} sector"},
                      "geometry": {"type": "Polygon", "coordinates": [ring(s)]}})
        feats.append({"type": "Feature", "properties": {"name": f"drone{i} planned lawnmower"},
                      "geometry": {"type": "LineString", "coordinates": [[w.lon, w.lat] for w in d.plan.waypoints]}})
        feats.append({"type": "Feature", "properties": {"name": f"drone{i} flown track"},
                      "geometry": {"type": "LineString", "coordinates": [
                          list(reversed(frame.gps((n, e)))) for _, n, e, _, _ in d.track[::5]]}})
    for t, _ in gt:
        feats.append({"type": "Feature", "properties": {"name": f"truth {t['id']}", "kind": t["kind"],
                                                        "hardhat": t["hardhat"], "marker-color": "#888"},
                      "geometry": {"type": "Point", "coordinates": [t["lon"], t["lat"]]}})
    for p in pois:
        lat, lon = p.geotag or frame.gps(p.estimate(True))
        feats.append({"type": "Feature", "properties": {
            "name": f"POI {p.id}", "classes": sorted(p.classes), "status": p.status,
            "found_by": p.found_by, "error_m": round(match[p.id][1], 2) if p.id in match else None,
            "marker-color": "#0a0" if p.status == "delivered" else "#fa0"},
            "geometry": {"type": "Point", "coordinates": [lon, lat]}})
    (out / "mission.geojson").write_text(json.dumps({"type": "FeatureCollection", "features": feats}, indent=1))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 8))
        for poly, style, label in ((mission["geofence"], "r--", "geofence"),
                                   (mission["search_area"], "y-", "search area")):
            pts = [frame.ne(*c) for c in poly] + [frame.ne(*poly[0])]
            ax.plot([p[1] for p in pts], [p[0] for p in pts], style, lw=1.5, label=label)
        cols = ["tab:blue", "tab:purple", "tab:cyan"]
        for d in drones:
            ax.plot([p[1] for p in d.lawn_pts], [p[0] for p in d.lawn_pts], ":", color=cols[d.i % 3], lw=1,
                    label=f"drone{d.i} plan")
            ax.plot([p[2] for p in d.track], [p[1] for p in d.track], "-", color=cols[d.i % 3], lw=1.2,
                    label=f"drone{d.i} flown")
        for t, g_ne in gt:
            ax.plot(g_ne[1], g_ne[0], "s", mfc="none", mec="k", ms=10)
            ax.annotate(t["id"].replace("poi_", "gt") + (" H" if t["hardhat"] else ""), (g_ne[1], g_ne[0]),
                        textcoords="offset points", xytext=(6, -10), fontsize=7)
        for p in pois:
            e = p.estimate(True)
            ax.plot(e[1], e[0], "*", color="green" if p.status == "delivered" else "orange", ms=12)
            txt = f"POI{p.id} {'+'.join(sorted(p.classes))}"
            if p.id in match:
                txt += f" ({match[p.id][1]:.1f} m)"
            ax.annotate(txt, (e[1], e[0]), textcoords="offset points", xytext=(6, 4), fontsize=7)
        ax.plot([], [], "s", mfc="none", mec="k", label="ground truth")
        ax.plot([], [], "*", color="green", label="geotag (served)")
        ax.set_aspect("equal")
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)")
        ax.set_title(f"NIDAR RescueSwarm sim  seed {mission['seed']}  |  {len(match)}/{len(gt)} found  |  "
                     f"{elapsed:.0f} s")
        ax.legend(loc="upper left", fontsize=7, bbox_to_anchor=(1.01, 1))
        fig.tight_layout()
        fig.savefig(out / "map.png", dpi=130)
        plt.close(fig)
    except ImportError:
        pass
    return lines


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mission", default=str(HERE / "generated" / "mission_area.json"))
    ap.add_argument("--truth", default=str(HERE / "generated" / "ground_truth.json"))
    ap.add_argument("--drones", type=int, default=2)
    ap.add_argument("--alt", type=float, default=6.0, help="survey height above ground (lidar), drone0 (m)")
    ap.add_argument("--alt-step", type=float, default=1.0, help="extra survey height per drone, for separation (m)")
    ap.add_argument("--speed", type=float, default=5.0, help="lawnmower ground speed (m/s)")
    ap.add_argument("--sidelap", type=float, default=0.25, help="camera footprint overlap between passes")
    ap.add_argument("--inset", type=float, default=1.0, help="lawnmower inset from the sector edge (m)")
    ap.add_argument("--drop-agl", type=float, default=3.0, help="height to descend to over a POI (m)")
    ap.add_argument("--drop-hold", type=float, default=3.0, help="seconds to hold at drop height")
    ap.add_argument("--detector", choices=["gt", "yolo"], default="gt")
    ap.add_argument("--weights", help="YOLO weights for --detector yolo (default ~/models/yolov8n.pt)")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--confirm-frames", type=int, default=2, help="frames before a detection becomes a POI")
    ap.add_argument("--merge-radius", type=float, default=4.0, help="detections closer than this are one POI (m)")
    ap.add_argument("--match-radius", type=float, default=5.0, help="geotag-to-truth match radius for scoring (m)")
    ap.add_argument("--time-limit", type=float, default=1800, help="mission time limit (s); brief allows 30 min")
    ap.add_argument("--stagger", type=float, default=4.0, help="seconds between drone launches")
    ap.add_argument("--accel", type=float, default=1.5)
    ap.add_argument("--kp-xy", type=float, default=0.8)
    ap.add_argument("--kz", type=float, default=0.8)
    ap.add_argument("--vz-max", type=float, default=2.0)
    ap.add_argument("--wp-radius", type=float, default=1.5)
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--dump-dataset", metavar="DIR", help="save camera frames + YOLO labels from ground truth")
    ap.add_argument("--plan-only", action="store_true", help="print the lawnmower plans and exit")
    ap.add_argument("--out", default=str(HERE / "runs" / time.strftime("%Y%m%d_%H%M%S")))
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    logf = open(out / "mission.log", "w")
    t_start = time.time()

    def log(msg):
        line = f"{time.time() - t_start:7.1f}s {msg}"
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()

    if not 1.0 < args.drop_agl < args.alt - 0.5:
        ap.error(f"--drop-agl {args.drop_agl} must be between 1 m and the survey height {args.alt} m")
    mission = json.loads(Path(args.mission).read_text())
    truth = json.loads(Path(args.truth).read_text())
    frame = Frame(mission["origin"]["lat"], mission["origin"]["lon"])
    plans, sectors = build_plans(args, mission, frame)
    for i, p in enumerate(plans):
        fw, fh = footprint(args.alt + i * args.alt_step, mission)
        log(f"[plan] drone{i}: {p.pass_count} passes, spacing {p.pass_spacing_actual_m:.1f} m "
            f"(footprint {fw:.1f} x {fh:.1f} m), path {p.path_length_m:.0f} m, "
            f"~{p.path_length_m / args.speed:.0f} s at {args.speed} m/s, sweep {p.axis_description}")
    worst = max(p.path_length_m for p in plans) / args.speed
    if worst > 0.8 * args.time_limit:
        log(f"[plan] WARNING: the longest sector alone needs ~{worst / 60:.1f} min of the "
            f"{args.time_limit / 60:.0f} min limit before transit, dips and landing; "
            f"raise --speed, --alt or --sidelap, or add drones")
    if args.plan_only:
        return

    from gz.transport13 import Node
    node = Node()
    feeds = [CameraFeed(node, i) for i in range(args.drones)]
    detector = make_detector(args.detector, args.weights, args.conf)
    dumper = DatasetDumper(args.dump_dataset) if args.dump_dataset else None
    registry = Registry(args.merge_radius)
    fence_ne = [frame.ne(*c) for c in mission["geofence"]]

    drones = [Drone(i, args, frame, mission, plans[i], sectors[i], registry, log) for i in range(args.drones)]
    for d in drones:
        d.fence = fence_ne
    view = None if args.no_gui else LiveView(drones, registry, frame, mission, fence_ne)

    log(f"[mission] {len(drones)} drones, detector={args.detector}. Arming (SITL GPS/EKF takes ~30 s)...")
    mission_start = time.time()
    for d in drones:
        d.mission_start = mission_start
    deadline = mission_start + args.time_limit
    period = 1.0 / LOOP_HZ
    try:
        while not all(d.done for d in drones):
            tick = time.time()
            for d in drones:
                if d.state == "INIT" and tick - mission_start < d.i * args.stagger:
                    continue
                d.perceive(feeds[d.i], detector, dumper)
                d.step(mission_start, deadline)
            if view and view.draw(tick - mission_start) == ord("q"):
                log("[mission] q pressed: returning all drones")
                for d in drones:
                    if d.state not in ("LAND", "DONE", "RETURN"):
                        d.go("RETURN")
            time.sleep(max(0.0, period - (time.time() - tick)))
    except KeyboardInterrupt:
        log("[mission] Ctrl-C: switching all drones to RTL")
        for d in drones:
            d.set_mode("RTL")

    elapsed = time.time() - mission_start
    lines = write_report(out, args, mission, truth, frame, drones, registry, sectors, elapsed)
    log("[mission] " + "\n            ".join(lines))
    if dumper:
        log(f"[mission] dataset: {dumper.saved} labelled frames in {args.dump_dataset}")
    log(f"[mission] report written to {out}/")
    if view:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
