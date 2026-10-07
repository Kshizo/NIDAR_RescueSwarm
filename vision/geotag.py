"""Geotag image detections using the mission's FC telemetry log.

Telemetry: the fc_telemetry_*.log written by mission/fc_telemetry_logger.py.
Each GLOBAL_POSITION_INT line carries the Pi wall-clock time, lat/lon,
relative_alt (mm above home) and hdg (centi-degrees). Video frame times come
from the recording's filename (flight_YYYY-MM-DD_HH-MM-SS) plus each frame's
timestamp, on the same Pi clock.

Camera model: pinhole camera pointing straight down (nadir), flat ground.
The log has no ATTITUDE, so roll/pitch are ignored: at ~5 deg tilt and 2.5 m
that is ~0.2 m of error. By default the top of the image is the drone's nose;
use cam_yaw_deg if the camera is mounted rotated.
"""
import bisect
import math
import re
from dataclasses import dataclass
from datetime import datetime

EARTH_R = 6378137.0
# Microsoft LifeCam HD-3000: 68.5 deg diagonal FOV -> ~61.4 deg horizontal at 16:9.
LIFECAM_HFOV_DEG = 61.4

_LINE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}) \| GLOBAL_POSITION_INT")
_FIELD = re.compile(r"(\w+)=(-?\d+)")


@dataclass
class Pose:
    t: float        # unix seconds (Pi local clock, naive)
    lat: float
    lon: float
    alt: float      # metres above home
    hdg: float      # degrees, 0 = north, clockwise


class Telemetry:
    def __init__(self, paths):
        poses = []
        for path in paths:
            with open(path, errors="replace") as f:
                for line in f:
                    m = _LINE.match(line)
                    if not m:
                        continue
                    d = dict(_FIELD.findall(line))
                    if int(d.get("hdg", 65535)) == 65535 or int(d["lat"]) == 0:
                        continue
                    t = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f").timestamp()
                    poses.append(Pose(t, int(d["lat"]) / 1e7, int(d["lon"]) / 1e7,
                                      int(d["relative_alt"]) / 1000.0, int(d["hdg"]) / 100.0))
        poses.sort(key=lambda p: p.t)
        self.poses = poses
        self.times = [p.t for p in poses]

    def covers(self, t):
        return bool(self.times) and self.times[0] <= t <= self.times[-1]

    def at(self, t, max_gap=1.0):
        """Pose interpolated at time t, or None if there is no telemetry close enough."""
        i = bisect.bisect_left(self.times, t)
        if i == 0 or i == len(self.times):
            return None
        a, b = self.poses[i - 1], self.poses[i]
        if b.t - a.t > max_gap:
            return None
        k = (t - a.t) / (b.t - a.t)
        dh = (b.hdg - a.hdg + 180) % 360 - 180
        return Pose(t, a.lat + k * (b.lat - a.lat), a.lon + k * (b.lon - a.lon),
                    a.alt + k * (b.alt - a.alt), (a.hdg + k * dh) % 360)


class Camera:
    def __init__(self, width, height, hfov_deg=LIFECAM_HFOV_DEG, cam_yaw_deg=0.0):
        self.w, self.h = width, height
        self.cx, self.cy = width / 2, height / 2
        self.f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)   # square pixels
        self.cam_yaw = cam_yaw_deg

    def pixel_to_ne(self, u, v, pose):
        """Ground offset (north, east) in metres of pixel (u, v) from the drone."""
        right = (u - self.cx) * pose.alt / self.f
        fwd = -(v - self.cy) * pose.alt / self.f
        yaw = math.radians(pose.hdg + self.cam_yaw)
        north = fwd * math.cos(yaw) - right * math.sin(yaw)
        east = fwd * math.sin(yaw) + right * math.cos(yaw)
        return north, east

    def pixel_to_latlon(self, u, v, pose):
        n, e = self.pixel_to_ne(u, v, pose)
        return offset_latlon(pose.lat, pose.lon, n, e)

    def expected_size_px(self, size_m, alt):
        return size_m * self.f / alt


def offset_latlon(lat, lon, north, east):
    return (lat + math.degrees(north / EARTH_R),
            lon + math.degrees(east / (EARTH_R * math.cos(math.radians(lat)))))


def distance_m(lat1, lon1, lat2, lon2):
    n = math.radians(lat2 - lat1) * EARTH_R
    e = math.radians(lon2 - lon1) * EARTH_R * math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(n, e)


def merge_cones(items, radius_m):
    """Group per-track positions into unique cones.

    items: dicts with color, lat, lon, weight. Same-colour items closer than
    radius_m (to the running group centre) are merged; returns groups with a
    weighted-mean position and the member list.
    """
    groups = []
    for it in sorted(items, key=lambda x: -x["weight"]):
        best, best_d = None, radius_m
        for g in groups:
            if g["color"] != it["color"]:
                continue
            d = distance_m(g["lat"], g["lon"], it["lat"], it["lon"])
            if d <= best_d:
                best, best_d = g, d
        if best is None:
            groups.append({"color": it["color"], "lat": it["lat"], "lon": it["lon"],
                           "weight": it["weight"], "members": [it]})
        else:
            w = best["weight"] + it["weight"]
            best["lat"] = (best["lat"] * best["weight"] + it["lat"] * it["weight"]) / w
            best["lon"] = (best["lon"] * best["weight"] + it["lon"] * it["weight"]) / w
            best["weight"] = w
            best["members"].append(it)
    return groups
