"""Compare detect_cones_video.py output against a synthetic truth.json."""
import json
import sys

from geotag import distance_m


def main():
    truth = json.load(open(sys.argv[1]))
    found = json.load(open(sys.argv[2]))["unique_cones"]
    unmatched = list(found)
    errs = []
    for t in truth:
        cands = [f for f in unmatched if f["color"] == t["color"]]
        best = min(cands, key=lambda f: distance_m(t["lat"], t["lon"], f["lat"], f["lon"]),
                   default=None)
        d = distance_m(t["lat"], t["lon"], best["lat"], best["lon"]) if best else None
        if best and d < 1.0:
            unmatched.remove(best)
            errs.append(d)
            print(f"  {t['color']:6s} found, error {d:.2f} m (tracks {best['tracks']})")
        else:
            print(f"  {t['color']:6s} MISSED")
    for f in unmatched:
        print(f"  {f['color']:6s} FALSE POSITIVE at {f['lat']:.7f}, {f['lon']:.7f}")
    if errs:
        print(f"found {len(errs)}/{len(truth)}, false positives {len(unmatched)}, "
              f"error mean {sum(errs) / len(errs):.2f} m, max {max(errs):.2f} m")


if __name__ == "__main__":
    main()
