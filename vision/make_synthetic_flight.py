"""End-to-end geotagging test: render a fake downward-camera video along a real flight.

Builds a ground texture from the sample cone photos, places cone sprites at
known GPS positions along the flown path, and renders what the camera model in
geotag.py would see at every telemetry pose. The output video is named
flight_<start>.avi so detect_cones_video.py can line it up with the same
telemetry log; truth.json holds the true cone positions for comparison.

  python make_synthetic_flight.py samples \
      ../data/telemetry/fc_telemetry_2026-10-05_2015_to_2026-10-05_2045.log /tmp/geo_test \
      --start "2026-10-05 20:42:49" --duration 143
"""
import argparse
import glob
import json
import math
import os
from datetime import datetime

import cv2
import numpy as np

from cone_threshold import detect
from geotag import Camera, Telemetry, distance_m, offset_latlon

RES = 0.004            # ground texture metres per pixel (photos are ~4 mm/px)
W, H = 1280, 720


def build_ground(photos, shape, rng):
    """Tile random cone-free patches from the photos into one ground texture."""
    patches = []
    for img in photos:
        boxes = detect(img)
        for _ in range(40):
            y = rng.integers(0, img.shape[0] - 160)
            x = rng.integers(0, img.shape[1] - 160)
            if any(x - 20 < bx + bw and bx < x + 180 and y - 20 < by + bh and by < y + 180
                   for _, bx, by, bw, bh, _ in boxes):
                continue
            patches.append(img[y:y + 160, x:x + 160])
    ground = np.zeros(shape + (3,), np.uint8)
    for y in range(0, shape[0], 150):
        for x in range(0, shape[1], 150):
            p = patches[rng.integers(len(patches))]
            p = cv2.flip(p, int(rng.integers(-1, 2))) if rng.random() < 0.75 else p
            h, w = min(160, shape[0] - y), min(160, shape[1] - x)
            ground[y:y + h, x:x + w] = p[:h, :w]
    return cv2.GaussianBlur(ground, (3, 3), 0)


def cone_sprites(photo_dir):
    """Top-down cone crops from the sample photos: (color, image)."""
    sprites = []
    for name in ["IMG_2389", "IMG_2392", "IMG_2390", "IMG_2394", "IMG_2396"]:
        img = cv2.imread(os.path.join(photo_dir, name + ".jpg"))
        for color, x, y, w, h, _ in detect(img):
            if x < 5 or y < 5 or x + w > img.shape[1] - 5 or y + h > img.shape[0] - 5:
                continue
            p = int(0.25 * max(w, h))
            sprites.append((color, img[y - p:y + h + p, x - p:x + w + p].copy()))
    return sprites


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("photo_dir")
    ap.add_argument("telemetry")
    ap.add_argument("out_dir")
    ap.add_argument("--start", required=True, help='video start, "YYYY-MM-DD HH:MM:SS"')
    ap.add_argument("--duration", type=float, default=60)
    ap.add_argument("--fps", type=float, default=10)
    ap.add_argument("--cones", type=int, default=8)
    ap.add_argument("--cone-size-m", type=float, default=0.20)
    args = ap.parse_args()

    rng = np.random.default_rng(1)
    t0 = datetime.strptime(args.start, "%Y-%m-%d %H:%M:%S")
    telem = Telemetry([args.telemetry])
    poses = [telem.at(t0.timestamp() + k / args.fps)
             for k in range(int(args.duration * args.fps))]
    poses = [p for p in poses if p and p.alt > 1.0]
    if not poses:
        raise SystemExit("no airborne telemetry in that window")

    # Local frame: metres north/east of the first pose.
    lat0, lon0 = poses[0].lat, poses[0].lon
    k_n = math.radians(1) * 6378137.0
    k_e = k_n * math.cos(math.radians(lat0))
    ne = np.array([((p.lat - lat0) * k_n, (p.lon - lon0) * k_e) for p in poses])
    margin = 3.0
    n_max, e_min = ne[:, 0].max() + margin, ne[:, 1].min() - margin
    rows = int((ne[:, 0].ptp() + 2 * margin) / RES)
    cols = int((ne[:, 1].ptp() + 2 * margin) / RES)

    photos = [cv2.imread(p) for p in sorted(glob.glob(os.path.join(args.photo_dir, "*.jpg")))
              if "(1)" not in p and "2388" not in p]       # 2388 is an oblique shot
    ground = build_ground(photos, (rows, cols), rng)

    # Cones near the flown path, at least 1.5 m apart.
    sprites = cone_sprites(args.photo_dir)
    truth = []
    for _ in range(500):
        if len(truth) == args.cones:
            break
        i = rng.integers(len(poses))
        n = ne[i, 0] + rng.uniform(-0.6, 0.6)
        e = ne[i, 1] + rng.uniform(-0.6, 0.6)
        lat, lon = offset_latlon(lat0, lon0, n, e)
        if any(distance_m(lat, lon, c["lat"], c["lon"]) < 1.5 for c in truth):
            continue
        color, spr = sprites[len(truth) % len(sprites)]
        spr = cv2.rotate(spr, int(rng.integers(0, 3))) if rng.random() < 0.7 else spr
        size_px = args.cone_size_m * 1.5 / RES                # sprite includes padding
        spr = cv2.resize(spr, (int(size_px), int(size_px)))
        r, c = int((n_max - n) / RES - size_px / 2), int((e - e_min) / RES - size_px / 2)
        ground[r:r + spr.shape[0], c:c + spr.shape[1]] = spr
        truth.append({"color": color, "lat": lat, "lon": lon})

    os.makedirs(args.out_dir, exist_ok=True)
    video = os.path.join(args.out_dir, f"flight_{t0:%Y-%m-%d_%H-%M-%S}.avi")
    writer = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"MJPG"), args.fps, (W, H))
    cam = Camera(W, H)
    for k in range(int(args.duration * args.fps)):
        pose = telem.at(t0.timestamp() + k / args.fps)
        if pose is None:
            frame = np.zeros((H, W, 3), np.uint8)
        else:
            # Image pixel -> ground texture pixel is affine for a nadir camera.
            src, dst = [], []
            for u, v in [(0, 0), (W, 0), (0, H)]:
                dn, de = cam.pixel_to_ne(u, v, pose)
                n = (pose.lat - lat0) * k_n + dn
                e = (pose.lon - lon0) * k_e + de
                src.append((u, v))
                dst.append(((e - e_min) / RES, (n_max - n) / RES))
            M = cv2.getAffineTransform(np.float32(src), np.float32(dst))
            frame = cv2.warpAffine(ground, M, (W, H), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                   borderMode=cv2.BORDER_REFLECT)
            frame = np.clip(frame + rng.normal(0, 3, frame.shape), 0, 255).astype(np.uint8)
        writer.write(frame)
    writer.release()
    with open(os.path.join(args.out_dir, "truth.json"), "w") as f:
        json.dump(truth, f, indent=2)
    print(f"wrote {video} and truth.json ({len(truth)} cones)")


if __name__ == "__main__":
    main()
