"""Post-flight cone detection on a downward-camera recording (classical CV, no ML).

Runs the HSV detector from cone_threshold.py on every frame, links detections
across frames with a nearest-centroid tracker, and keeps only tracks seen in
enough frames, so each physical cone pass is reported once and single-frame
colour noise is dropped.

With --telemetry (the mission's fc_telemetry_*.log), every detection is
geotagged (see geotag.py) and tracks of the same colour that land within
--merge-radius-m of each other are merged, so a cone seen again on the next
raster pass is reported once.

Outputs (in --out-dir, default <video stem>_cones/):
  detections.csv   every confirmed per-frame detection (+ lat/lon with telemetry)
  cones.json       one entry per track, plus merged unique cones with telemetry
  cones.geojson    unique cones as map points (drop onto geojson.io)
  crops/           best crop of each track
  annotated.mp4    the video with boxes and track ids (skip with --no-video)
"""
import argparse
import csv
import json
import math
import os
import re
from datetime import datetime, timedelta

import statistics

import cv2

from cone_threshold import BOX_COLORS, detect
from geotag import Camera, Telemetry, merge_cones

FILENAME_TIME = re.compile(r"flight_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})")


class Track:
    def __init__(self, tid, color, box, frame_idx):
        self.id = tid
        self.color = color
        self.box = box
        self.first = self.last = frame_idx
        self.hits = 1
        self.misses = 0
        self.best_area = 0
        self.best_frame = frame_idx
        self.best_crop = None
        self.rows = []

    @property
    def center(self):
        x, y, w, h = self.box
        return x + w / 2, y + h / 2


class Tracker:
    """Greedy nearest-centroid matching, per colour."""

    def __init__(self, max_dist_frac, max_misses, min_hits):
        self.max_dist_frac = max_dist_frac
        self.max_misses = max_misses
        self.min_hits = min_hits
        self.active = []
        self.finished = []
        self.next_id = 1

    def update(self, dets, frame_idx, frame_diag):
        max_dist = self.max_dist_frac * frame_diag
        pairs = []
        for ti, t in enumerate(self.active):
            tx, ty = t.center
            for di, (color, x, y, w, h, _) in enumerate(dets):
                if color != t.color:
                    continue
                d = math.hypot(x + w / 2 - tx, y + h / 2 - ty)
                if d <= max_dist:
                    pairs.append((d, ti, di))
        pairs.sort()
        used_t, used_d, matched = set(), set(), []
        for _, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            matched.append((self.active[ti], dets[di]))

        for t, (color, x, y, w, h, area) in matched:
            t.box = (x, y, w, h)
            t.last = frame_idx
            t.hits += 1
            t.misses = 0
        for ti, t in enumerate(self.active):
            if ti not in used_t:
                t.misses += 1
        for di, (color, x, y, w, h, area) in enumerate(dets):
            if di not in used_d:
                t = Track(self.next_id, color, (x, y, w, h), frame_idx)
                self.next_id += 1
                self.active.append(t)
                matched.append((t, dets[di]))

        alive = []
        for t in self.active:
            (self.finished if t.misses > self.max_misses else alive).append(t)
        self.active = alive
        return matched

    def confirmed(self, include_active=True):
        tracks = self.finished + (self.active if include_active else [])
        return sorted((t for t in tracks if t.hits >= self.min_hits), key=lambda t: t.first)


def video_start_time(path):
    """Recording start on the Pi clock.

    Prefers the flight_<ts>.json sidecar written by mission/video_recorder.py
    (millisecond resolution); falls back to the 1 s timestamp in the filename.
    """
    sidecar = os.path.splitext(path)[0] + ".json"
    if os.path.exists(sidecar):
        try:
            with open(sidecar) as f:
                return datetime.fromtimestamp(json.load(f)["start_epoch"])
        except (ValueError, KeyError, OSError):
            pass
    m = FILENAME_TIME.search(os.path.basename(path))
    return datetime.strptime(m.group(1), "%Y-%m-%d_%H-%M-%S") if m else None


def touches_border(box, shape, margin=2):
    x, y, w, h = box
    H, W = shape[:2]
    return x <= margin or y <= margin or x + w >= W - margin or y + h >= H - margin


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--out-dir")
    ap.add_argument("--min-hits", type=int, default=5,
                    help="frames a track needs before it counts as a cone (default 5)")
    ap.add_argument("--max-misses", type=int, default=10,
                    help="frames a track may go undetected before it ends (default 10)")
    ap.add_argument("--max-dist", type=float, default=0.08,
                    help="max centroid jump per frame, as a fraction of the frame diagonal")
    ap.add_argument("--stride", type=int, default=1, help="process every Nth frame")
    ap.add_argument("--no-video", action="store_true", help="skip writing annotated.mp4")
    geo = ap.add_argument_group("geotagging")
    geo.add_argument("--telemetry", nargs="+", metavar="LOG",
                     help="fc_telemetry_*.log file(s) covering the flight")
    geo.add_argument("--hfov-deg", type=float, default=61.4,
                     help="camera horizontal field of view (LifeCam HD-3000 at 16:9: 61.4)")
    geo.add_argument("--cam-yaw-deg", type=float, default=0.0,
                     help="camera rotation vs the drone nose; 0 = image top is forward")
    geo.add_argument("--time-offset-s", type=float, default=0.0,
                     help="added to video timestamps to line them up with telemetry")
    geo.add_argument("--min-alt-m", type=float, default=1.0,
                     help="ignore detections while the drone is below this altitude")
    geo.add_argument("--cone-size-m", type=float,
                     help="cone base width; if set, blobs far from the expected pixel "
                          "size at the current altitude are rejected")
    geo.add_argument("--merge-radius-m", type=float, default=1.0,
                     help="same-colour tracks closer than this are one cone")
    args = ap.parse_args()

    stem = os.path.splitext(os.path.basename(args.video))[0]
    out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(args.video)),
                                           stem + "_cones")
    os.makedirs(os.path.join(out_dir, "crops"), exist_ok=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    t0 = video_start_time(args.video)

    telem = cam = None
    if args.telemetry:
        if t0 is None:
            raise SystemExit("geotagging needs the start time in the filename "
                             "(flight_YYYY-MM-DD_HH-MM-SS.mkv)")
        telem = Telemetry(args.telemetry)
        start = t0.timestamp() + args.time_offset_s
        if not telem.covers(start):
            raise SystemExit(f"telemetry does not cover the video start ({t0})")
        print(f"telemetry: {len(telem.poses)} positions")

    tracker = Tracker(args.max_dist, args.max_misses, args.min_hits)
    writer = None
    frame_idx = -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1
        if frame_idx % args.stride:
            continue
        H, W = frame.shape[:2]
        # Per-frame container timestamp; fall back to frame count if missing.
        pos_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
        ts = pos_ms / 1000.0 if pos_ms > 0 or frame_idx == 0 else frame_idx / fps
        pose = None
        if telem:
            if cam is None:
                cam = Camera(W, H, args.hfov_deg, args.cam_yaw_deg)
            pose = telem.at(t0.timestamp() + args.time_offset_s + ts)
            if pose and pose.alt < args.min_alt_m:
                pose = None
        dets = detect(frame)
        if args.cone_size_m and pose:
            exp = cam.expected_size_px(args.cone_size_m, pose.alt)
            dets = [d for d in dets if 0.5 * exp <= max(d[3], d[4]) <= 2.2 * exp]
        matched = tracker.update(dets, frame_idx, math.hypot(W, H))

        for t, (color, x, y, w, h, area) in matched:
            lat = lon = None
            if pose:
                lat, lon = cam.pixel_to_latlon(x + w / 2, y + h / 2, pose)
            t.rows.append((frame_idx, ts, x, y, w, h, pose, lat, lon))
            # Best view: largest blob fully inside the frame.
            if area > t.best_area and not touches_border((x, y, w, h), frame.shape):
                pad = int(0.5 * max(w, h))
                t.best_area = area
                t.best_frame = frame_idx
                t.best_crop = frame[max(y - pad, 0):y + h + pad,
                                    max(x - pad, 0):x + w + pad].copy()

        if not args.no_video:
            if writer is None:
                writer = cv2.VideoWriter(os.path.join(out_dir, "annotated.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                         fps / args.stride, (W, H))
            vis = frame.copy()
            for t, (color, x, y, w, h, _) in matched:
                c = BOX_COLORS[color]
                thick = 2 if t.hits >= args.min_hits else 1
                cv2.rectangle(vis, (x, y), (x + w, y + h), c, thick)
                cv2.putText(vis, f"#{t.id} {color}", (x, max(y - 5, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
            cv2.putText(vis, f"frame {frame_idx}", (10, H - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            writer.write(vis)
    cap.release()
    if writer:
        writer.release()

    cones = tracker.confirmed()
    with open(os.path.join(out_dir, "detections.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["frame", "video_time_s", "wall_time", "track_id", "color",
                     "x", "y", "w", "h", "cx", "cy", "drone_lat", "drone_lon",
                     "drone_alt_m", "drone_hdg", "cone_lat", "cone_lon"])
        for t in cones:
            for fi, ts, x, y, w, h, pose, lat, lon in t.rows:
                wall = (t0 + timedelta(seconds=ts)).isoformat(timespec="milliseconds") if t0 else ""
                geo_cols = ([f"{pose.lat:.7f}", f"{pose.lon:.7f}", f"{pose.alt:.2f}",
                             f"{pose.hdg:.1f}", f"{lat:.7f}", f"{lon:.7f}"]
                            if pose else [""] * 6)
                wr.writerow([fi, f"{ts:.3f}", wall, t.id, t.color, x, y, w, h,
                             x + w // 2, y + h // 2] + geo_cols)

    summary = []
    for t in cones:
        crop_name = None
        if t.best_crop is not None and t.best_crop.size:
            crop_name = f"crops/cone_{t.id:03d}_{t.color}.jpg"
            cv2.imwrite(os.path.join(out_dir, crop_name), t.best_crop)
        entry = {
            "track_id": t.id, "color": t.color, "frames_detected": t.hits,
            "first_frame": t.first, "last_frame": t.last,
            "first_time_s": round(t.first / fps, 3), "last_time_s": round(t.last / fps, 3),
            "best_frame": t.best_frame, "crop": crop_name,
        }
        if t0:
            entry["best_wall_time"] = (t0 + timedelta(seconds=t.best_frame / fps)).isoformat(
                timespec="milliseconds")
        geo_rows = [r for r in t.rows if r[7] is not None]
        if geo_rows:
            # Median is robust to the odd frame with a bad pose or partial blob.
            entry["lat"] = round(statistics.median(r[7] for r in geo_rows), 7)
            entry["lon"] = round(statistics.median(r[8] for r in geo_rows), 7)
            entry["geotagged_frames"] = len(geo_rows)
        summary.append(entry)

    report = {"video": os.path.abspath(args.video), "fps": fps, "frames": frame_idx + 1,
              "tracks": summary}
    if telem:
        groups = merge_cones([{"color": e["color"], "lat": e["lat"], "lon": e["lon"],
                               "weight": e["geotagged_frames"], "track_id": e["track_id"]}
                              for e in summary if "lat" in e], args.merge_radius_m)
        unique = [{"cone_id": i + 1, "color": g["color"], "lat": round(g["lat"], 7),
                   "lon": round(g["lon"], 7), "frames": g["weight"],
                   "tracks": sorted(m["track_id"] for m in g["members"])}
                  for i, g in enumerate(sorted(groups, key=lambda g: min(
                      m["track_id"] for m in g["members"])))]
        report["unique_cones"] = unique
        report["camera"] = {"hfov_deg": args.hfov_deg, "cam_yaw_deg": args.cam_yaw_deg,
                            "time_offset_s": args.time_offset_s}
        with open(os.path.join(out_dir, "cones.geojson"), "w") as f:
            json.dump({"type": "FeatureCollection", "features": [
                {"type": "Feature",
                 "geometry": {"type": "Point", "coordinates": [c["lon"], c["lat"]]},
                 "properties": {"cone_id": c["cone_id"], "color": c["color"],
                                "marker-color": {"red": "#e53935", "yellow": "#fdd835",
                                                 "green": "#43a047"}[c["color"]],
                                "frames": c["frames"]}} for c in unique]}, f, indent=2)
        found = unique
    else:
        found = summary

    counts = {}
    for c in found:
        counts[c["color"]] = counts.get(c["color"], 0) + 1
    report["counts"] = counts
    with open(os.path.join(out_dir, "cones.json"), "w") as f:
        json.dump(report, f, indent=2)

    rejected = len(tracker.finished) + len(tracker.active) - len(cones)
    label = "unique cones" if telem else "cones"
    print(f"{frame_idx + 1} frames, {len(cones)} tracks, {len(found)} {label} {counts}, "
          f"{rejected} short tracks rejected -> {out_dir}")
    for c in found:
        if "lat" in c:
            print(f"  {c['color']:6s} {c['lat']:.7f}, {c['lon']:.7f}  ({c.get('frames', '')} frames)")


if __name__ == "__main__":
    main()
