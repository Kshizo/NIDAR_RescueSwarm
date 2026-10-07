#!/usr/bin/env python3
"""
Offline tests for vision/: colour detection, geotag maths, and an end-to-end
run on a synthetic flight rendered along the real 2026-10-05 telemetry.

    python tests/test_cone_detection.py

Needs opencv-python and numpy (vision/requirements.txt). No FC, no simulator.
Takes about a minute (the end-to-end render dominates).
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
VISION = os.path.join(ROOT, "vision")
sys.path.insert(0, VISION)

import cv2                                        # noqa: E402
from cone_threshold import detect                 # noqa: E402
from geotag import Camera, Pose, distance_m       # noqa: E402

TELEMETRY = os.path.join(ROOT, "data", "telemetry",
                         "fc_telemetry_2026-10-05_2015_to_2026-10-05_2045.log")
# Cones visible in each sample photo (IMG_2393's yellow is cut off at the top edge).
EXPECTED = {
    "IMG_2388": ["green", "red", "red", "yellow"],
    "IMG_2389": ["red"], "IMG_2390": ["green"], "IMG_2392": ["red"],
    "IMG_2393": ["red", "yellow"], "IMG_2394": ["green"], "IMG_2395": ["red"],
    "IMG_2396": ["red", "yellow"],
}

FAILURES = []


def check(name, condition, detail=""):
    print(f"  {'PASS' if condition else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    if not condition:
        FAILURES.append(name)


def test_sample_photos():
    print("sample photos")
    for stem, want in EXPECTED.items():
        got = sorted(d[0] for d in detect(cv2.imread(os.path.join(VISION, "samples", stem + ".jpg"))))
        check(stem, got == want, f"got {got}")


def test_projection():
    print("camera projection")
    cam = Camera(1280, 720, hfov_deg=60.0)
    north = Pose(0, 13.0, 74.0, 2.5, 0.0)
    n, e = cam.pixel_to_ne(640, 360, north)
    check("image centre is under the drone", abs(n) < 1e-9 and abs(e) < 1e-9)
    half_w = 2.5 * 0.5773502692                   # alt * tan(30 deg)
    n, e = cam.pixel_to_ne(1280, 360, north)
    check("right edge, heading 0 -> east", abs(n) < 1e-6 and abs(e - half_w) < 1e-6, f"{n:.3f},{e:.3f}")
    n, e = cam.pixel_to_ne(640, 0, Pose(0, 13.0, 74.0, 2.5, 90.0))
    check("top edge, heading 90 -> east", abs(n) < 1e-6 and e > 0, f"{n:.3f},{e:.3f}")
    lat, lon = cam.pixel_to_latlon(1280, 360, north)
    check("lat/lon offset matches metres", abs(distance_m(13.0, 74.0, lat, lon) - half_w) < 0.01)


def test_synthetic_flight():
    print("end-to-end synthetic flight (real 2026-10-05 telemetry)")
    if not os.path.exists(TELEMETRY):
        check("telemetry log present", False, TELEMETRY)
        return
    with tempfile.TemporaryDirectory() as tmp:
        run = lambda *a: subprocess.run([sys.executable, *a], cwd=VISION, check=True,
                                        capture_output=True, text=True)
        run("make_synthetic_flight.py", "samples", TELEMETRY, tmp,
            "--start", "2026-10-05 20:42:49", "--duration", "143", "--cones", "6")
        video = os.path.join(tmp, "flight_2026-10-05_20-42-49.avi")
        out = os.path.join(tmp, "out")
        run("detect_cones_video.py", video, "--telemetry", TELEMETRY, "--out-dir", out, "--no-video")
        truth = json.load(open(os.path.join(tmp, "truth.json")))
        found = json.load(open(os.path.join(out, "cones.json")))["unique_cones"]
        errs = []
        for t in truth:
            d = min((distance_m(t["lat"], t["lon"], f["lat"], f["lon"])
                     for f in found if f["color"] == t["color"]), default=99)
            errs.append(d)
        check("every cone found within 0.3 m", max(errs) < 0.3, f"max {max(errs):.2f} m")
        check("no duplicate or false cones", len(found) == len(truth),
              f"{len(found)} found vs {len(truth)} placed")


if __name__ == "__main__":
    test_sample_photos()
    test_projection()
    test_synthetic_flight()
    print(f"\n{'ALL PASS' if not FAILURES else f'{len(FAILURES)} FAILED: {FAILURES}'}")
    sys.exit(1 if FAILURES else 0)
