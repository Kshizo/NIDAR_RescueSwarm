"""HSV colour-threshold cone detector for top-down drone frames.

Used to auto-label frames for YOLO training and as a fallback detector.
Thresholds are tuned on the sample photos in samples/ (red, green, yellow
cones on reddish soil and grass).
"""
import argparse
import os

import cv2
import numpy as np

# OpenCV HSV: H 0-179, S/V 0-255. Each colour is a list of (lower, upper) ranges.
COLOR_RANGES = {
    "red":    [((0, 110, 90), (9, 255, 255)), ((165, 110, 90), (179, 255, 255))],
    "yellow": [((20, 70, 170), (38, 255, 255))],
    "green":  [((58, 100, 100), (90, 255, 255))],
}
BOX_COLORS = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0)}
CLASS_IDS = {"red": 0, "yellow": 1, "green": 2}

# Blob-level checks on the median colour of the whole blob. These separate cones
# from things whose individual pixels pass the ranges above: skin (median hue
# 7-9) vs red cones (176-2), and grass (median S ~108, V ~115) vs green cones.
# Red hue is checked after shifting by +90 so the 179->0 wrap is contiguous.
BLOB_RULES = {
    "red":    {"hue": (80, 96), "min_s": 135, "min_v": 120},   # shifted: 170..6
    "yellow": {"hue": (22, 34), "min_s": 80,  "min_v": 190},
    "green":  {"hue": (70, 90), "min_s": 130, "min_v": 140},
}
# The cone must be clearly more saturated than the ground right around it.
# (Washed-out yellow on reddish soil is the weakest case, at ~24.)
MIN_RING_CONTRAST = 20

MIN_AREA_FRAC = 0.0004   # blob must cover at least this fraction of the image
MAX_AREA_FRAC = 0.05
MIN_FILL = 0.35          # blob area / bounding-box area
MAX_ASPECT = 3.0


def color_mask(hsv, color):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in COLOR_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    return mask


def blob_ok(hsv, contour, color, box):
    """Median-colour and local-contrast checks for one candidate blob."""
    x, y, w, h = box
    pad = max(w, h) // 2
    H, W = hsv.shape[:2]
    x0, y0, x1, y1 = max(x - pad, 0), max(y - pad, 0), min(x + w + pad, W), min(y + h + pad, H)
    roi = hsv[y0:y1, x0:x1]
    inside = np.zeros(roi.shape[:2], np.uint8)
    cv2.drawContours(inside, [contour - [x0, y0]], -1, 255, cv2.FILLED)
    ring = cv2.dilate(inside, np.ones((3, 3), np.uint8), iterations=max(pad // 2, 3)) & ~cv2.dilate(
        inside, np.ones((3, 3), np.uint8), iterations=2)
    px = roi[inside > 0]
    if len(px) == 0 or not ring.any():
        return False
    rule = BLOB_RULES[color]
    hue = px[:, 0].astype(int)
    if color == "red":
        hue = (hue + 90) % 180
    med_h, med_s, med_v = np.median(hue), np.median(px[:, 1]), np.median(px[:, 2])
    if not (rule["hue"][0] <= med_h <= rule["hue"][1]
            and med_s >= rule["min_s"] and med_v >= rule["min_v"]):
        return False
    return med_s - np.median(roi[ring > 0][:, 1]) >= MIN_RING_CONTRAST


def detect(img):
    """Return a list of (color, x, y, w, h, area) boxes in pixel coordinates."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    img_area = img.shape[0] * img.shape[1]
    out = []
    for color in COLOR_RANGES:
        contours, _ = cv2.findContours(color_mask(hsv, color), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if not MIN_AREA_FRAC * img_area <= area <= MAX_AREA_FRAC * img_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if area / (w * h) < MIN_FILL or max(w, h) / min(w, h) > MAX_ASPECT:
                continue
            if not blob_ok(hsv, c, color, (x, y, w, h)):
                continue
            out.append((color, x, y, w, h, area))
    return out


def draw(img, dets):
    vis = img.copy()
    for color, x, y, w, h, _ in dets:
        cv2.rectangle(vis, (x, y), (x + w, y + h), BOX_COLORS[color], 2)
        cv2.putText(vis, color, (x, max(y - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    BOX_COLORS[color], 2)
    return vis


def yolo_lines(dets, shape, pad=0.1):
    """YOLO-format label lines, boxes padded by `pad` to include the cone's edge."""
    H, W = shape[:2]
    lines = []
    for color, x, y, w, h, _ in dets:
        cx, cy = (x + w / 2) / W, (y + h / 2) / H
        bw, bh = min(w * (1 + 2 * pad) / W, 1), min(h * (1 + 2 * pad) / H, 1)
        lines.append(f"{CLASS_IDS[color]} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("images", nargs="+")
    ap.add_argument("--vis-dir", help="write annotated images here")
    ap.add_argument("--label-dir", help="write YOLO .txt labels here")
    args = ap.parse_args()
    for d in (args.vis_dir, args.label_dir):
        if d:
            os.makedirs(d, exist_ok=True)
    for path in args.images:
        img = cv2.imread(path)
        if img is None:
            continue
        dets = detect(img)
        stem = os.path.splitext(os.path.basename(path))[0]
        print(f"{os.path.basename(path)}: " +
              (", ".join(f"{c}@({x},{y},{w}x{h})" for c, x, y, w, h, _ in dets) or "none"))
        if args.vis_dir:
            cv2.imwrite(os.path.join(args.vis_dir, stem + ".jpg"), draw(img, dets))
        if args.label_dir:
            with open(os.path.join(args.label_dir, stem + ".txt"), "w") as f:
                f.write("\n".join(yolo_lines(dets, img.shape)))


if __name__ == "__main__":
    main()
