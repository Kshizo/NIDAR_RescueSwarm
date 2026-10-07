# Cone detection + geotagging (classical CV)

Post-flight detection of red / yellow / green cones in the downward LifeCam
recordings from NIDAR_RescueSwarm, geotagged with the mission's FC telemetry.
OpenCV + NumPy only, no ML.

```bash
pip install -r vision/requirements.txt
cd vision
python detect_cones_video.py ../data/recordings/flight_2026-10-05_20-42-49.mkv \
    --telemetry ../data/telemetry/fc_telemetry_2026-10-05_2015_to_2026-10-05_2045.log
```

Writes `<video>_cones/`: `cones.json` (tracks + merged unique cones with lat/lon),
`cones.geojson` (drop onto geojson.io), `detections.csv`, `crops/`, `annotated.mp4`.
Without `--telemetry` it only detects and tracks.

| File | Purpose |
| --- | --- |
| `cone_threshold.py` | HSV colour detector; also runs on still images and writes YOLO labels |
| `detect_cones_video.py` | Video pipeline: detect, track, geotag, merge revisits |
| `geotag.py` | Telemetry parser, nadir pinhole camera model, cone merging |
| `make_synthetic_flight.py` | Renders a fake flight along real telemetry with cones at known GPS |
| `compare_truth.py` | Scores pipeline output against the synthetic truth |
| `samples/` | Hand-held top-down cone photos the colour thresholds were tuned on |

Tests: `python tests/test_cone_detection.py` (sample photos, projection maths, and an
end-to-end synthetic flight along the real 2026-10-05 telemetry; about a minute).

## Field checklist
- **Time sync matters most**: 1 s of video/telemetry offset is ~1 m of error at 1 m/s
  and makes revisits stop merging. `mission/video_recorder.py` writes a
  `flight_<ts>.json` sidecar with the millisecond start time, which is used when
  present; older recordings fall back to the 1 s filename time. Correct any
  remaining offset (e.g. camera start-up delay) with `--time-offset-s`.
- **Frame rate**: frames are timed from the container timestamps, not an assumed
  30 fps; the 2026-10-05 night recording actually ran at ~7.5 fps.
- **Camera orientation**: default assumes the top of the image is the drone nose.
  Fix with `--cam-yaw-deg` (e.g. 90, 180).
- **Cone size**: pass `--cone-size-m` to reject blobs of the wrong size for the altitude.
- Assumes flat ground and a level camera (the log has no ATTITUDE; ~0.2 m at 5 deg tilt).
- Yellow is the weakest colour (washes out in sun, close to lit pale surfaces).
