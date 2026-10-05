#!/usr/bin/env python3
"""
Flight video recorder — records the USB webcam from takeoff until landing.

How it knows the aircraft is flying
-----------------------------------
It does NOT open the flight controller. The serial link belongs to the mission
daemon (ardupilot_raster_mission.py), and a second reader on the same port would
steal its messages. Instead this follows the 1 Hz SUMMARY lines the daemon's
telemetry logger already writes to fc_telemetry.log:

    ... | SUMMARY mode=GUIDED ARMED alt=2.45m ned=(...) ...

State machine
-------------
    WAITING    -> RECORDING  when ARMED and alt >= VIDEO_START_ALT_M on
                             VIDEO_START_CONFIRM consecutive summaries (takeoff)
    RECORDING  -> WAITING    when the FC reports disarmed (landed), or when no
                             summary has arrived for VIDEO_STALE_S seconds (link
                             lost / daemon stopped) so the file gets finalized.

If ffmpeg dies mid-flight (camera unplugged and replugged, USB glitch) a new file
is started on the next summary that still shows the aircraft airborne.

The camera itself is handled by video_recorder.py (MJPEG 1280x720 @ 30 fps,
stream copy — no re-encoding, near-zero CPU).

IMPORTANT: only one process can hold the camera. The mission daemon also calls
video_recorder at arming; start_ardupilot_raster_mission.sh exports
VIDEO_RECORDER_ENABLED=0 so the daemon leaves the camera to this script.

Usage:
    python3 flight_video_recorder.py            # runs until Ctrl+C / SIGTERM
Environment overrides:
    FC_LOG_FILE, VIDEO_RECORDINGS_DIR, VIDEO_START_ALT_M, VIDEO_START_CONFIRM,
    VIDEO_STALE_S
"""
import os
import re
import sys
import time
import signal
import logging

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

FC_LOG_FILE = os.getenv("FC_LOG_FILE", os.path.join(PROJECT_DIR, "fc_telemetry.log"))
RECORDINGS_DIR = os.getenv("VIDEO_RECORDINGS_DIR", os.path.join(PROJECT_DIR, "recordings"))
# Height above home (m) that counts as "taken off". The mission climbs to 2.5 m;
# 1.0 m is clear of ground effect and of baro noise on the pad (~0.5 m seen
# while disarmed in the 2026-10-04 log).
START_ALT_M = float(os.getenv("VIDEO_START_ALT_M", "1.0"))
# Consecutive 1 Hz summaries above START_ALT_M before recording starts, so a
# single noisy altitude sample on the ground cannot start a recording.
START_CONFIRM = int(os.getenv("VIDEO_START_CONFIRM", "2"))
# No summary for this long while recording -> finalize the file.
STALE_S = float(os.getenv("VIDEO_STALE_S", "15"))
POLL_S = 0.2

# This process owns the camera, whatever the daemon's environment says.
os.environ["VIDEO_RECORDER_ENABLED"] = "1"
os.environ["VIDEO_RECORDINGS_DIR"] = RECORDINGS_DIR
sys.path.insert(0, PROJECT_DIR)
import video_recorder  # noqa: E402

SUMMARY_RE = re.compile(r"SUMMARY mode=(\S+) (ARMED|disarmed|\?) alt=(\S+)m")

logger = logging.getLogger("flight_video")
logger.setLevel(logging.INFO)
_fmt = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
for _h in (logging.StreamHandler(),
           logging.FileHandler(os.path.join(PROJECT_DIR, "flight_video_recorder.log"))):
    _h.setFormatter(_fmt)
    logger.addHandler(_h)


class LogFollower:
    """`tail -F` for the telemetry log: survives RotatingFileHandler rollovers."""

    def __init__(self, path):
        self.path = path
        self.fh = None
        self.inode = None

    def _open(self, from_end):
        try:
            fh = open(self.path, "r", errors="replace")
        except OSError:
            return False
        if from_end:
            fh.seek(0, os.SEEK_END)     # ignore history from earlier flights
        self.fh, self.inode = fh, os.fstat(fh.fileno()).st_ino
        return True

    def lines(self):
        if self.fh is None:
            if not self._open(from_end=True):
                return []
        out = []
        while True:
            line = self.fh.readline()
            if not line:
                break
            if line.endswith("\n"):
                out.append(line)
            else:                       # partial line: re-read it next poll
                self.fh.seek(self.fh.tell() - len(line))
                break
        # Rolled over (new inode) or truncated: switch to the new file, from its start.
        try:
            st = os.stat(self.path)
            if st.st_ino != self.inode or st.st_size < self.fh.tell():
                self.fh.close()
                self._open(from_end=False)
        except OSError:
            pass
        return out


def parse_summary(line):
    m = SUMMARY_RE.search(line)
    if not m:
        return None
    try:
        alt = float(m.group(3))
    except ValueError:
        alt = float("nan")
    return m.group(1), m.group(2) == "ARMED", alt


def stop(reason):
    logger.info(f"Stopping recording: {reason}")
    path = video_recorder.stop_recording()
    if path:
        size = os.path.getsize(path) / 1e6 if os.path.exists(path) else 0
        logger.info(f"Saved {path} ({size:.1f} MB)")


def start(mode, alt):
    logger.info(f"Takeoff detected (mode={mode}, alt={alt:.2f}m) -> starting recording")
    try:
        _, path = video_recorder.start_recording()
    except Exception as e:
        logger.error(f"Could not start recording: {e}")
        return False
    if not path:
        logger.error("Could not start recording (camera missing or ffmpeg unavailable; "
                     "see warning above). Will retry while airborne.")
        return False
    return True


def main():
    running = True

    def on_signal(sig, _frame):
        nonlocal running
        logger.info(f"Received signal {sig}, shutting down")
        running = False

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    logger.info(f"Flight video recorder ready. Following {FC_LOG_FILE}; recording starts when "
                f"ARMED and alt >= {START_ALT_M:.1f} m for {START_CONFIRM} s, stops on disarm. "
                f"Files -> {RECORDINGS_DIR}")
    cam = video_recorder.resolve_camera_device()
    logger.info(f"Camera: {cam}" if cam else "Camera: NOT DETECTED yet (will check again at takeoff)")

    follower = LogFollower(FC_LOG_FILE)
    recording = False
    above_count = 0
    last_summary = time.monotonic()

    while running:
        for line in follower.lines():
            parsed = parse_summary(line)
            if parsed is None:
                continue
            mode, armed, alt = parsed
            last_summary = time.monotonic()

            if not recording:
                above_count = above_count + 1 if (armed and alt >= START_ALT_M) else 0
                if above_count >= START_CONFIRM:
                    recording = start(mode, alt)
            elif not armed:
                stop(f"aircraft disarmed (mode={mode}, alt={alt:.2f}m)")
                recording, above_count = False, 0
            elif not video_recorder.is_recording():
                logger.warning("ffmpeg stopped while airborne; starting a new file")
                video_recorder.stop_recording()
                recording = start(mode, alt)

        if recording and time.monotonic() - last_summary > STALE_S:
            stop(f"no telemetry for {STALE_S:.0f}s (daemon stopped or FC link lost)")
            recording, above_count = False, 0

        time.sleep(POLL_S)

    if recording:
        stop("recorder shutting down")


if __name__ == "__main__":
    main()
