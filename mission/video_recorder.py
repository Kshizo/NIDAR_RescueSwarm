import os
import time
import datetime
import subprocess
import logging
import signal
import shutil
import atexit
import tempfile
import json

logger = logging.getLogger("video_recorder")
logger.setLevel(logging.INFO)

# Provide a default stream handler if none exists, so it works standalone or imported
if not logger.handlers:
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(
        logging.Formatter("%(asctime)s | %(levelname)s | %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
    )
    logger.addHandler(stream_handler)

RECORDINGS_DIR = os.getenv("VIDEO_RECORDINGS_DIR", "/home/aahswarm/ardupilot_testing/recordings/")

# Only one process can hold the camera. The mission daemon runs with
# VIDEO_RECORDER_ENABLED=0 so that flight_video_recorder.py (which records from
# takeoff instead of from arming) owns it.
ENABLED = os.getenv("VIDEO_RECORDER_ENABLED", "1") != "0"

# Camera device. Empty means "auto-discover".
#
# /dev/video0 was hardcoded, which does not survive this hardware: a Raspberry Pi 5
# enumerates its internal ISP as /dev/video19..29, and a USB webcam takes whatever
# number is free when it is plugged in. /dev/v4l/by-id/ carries a stable symlink per
# physical USB camera and lists nothing for the Pi's internal blocks, so it picks the
# real camera and skips the ISP nodes.
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "")
V4L_BY_ID_DIR = "/dev/v4l/by-id"

# Warn about a missing camera once per process rather than on every mission.
_camera_warning_issued = False


def resolve_camera_device() -> str | None:
    """Returns the capture device to record from, or None if no camera is attached."""
    if CAMERA_DEVICE:
        return CAMERA_DEVICE if os.path.exists(CAMERA_DEVICE) else None

    try:
        entries = sorted(os.listdir(V4L_BY_ID_DIR))
    except OSError:
        entries = []

    # "-index0" is the capture interface; later indices are metadata nodes.
    for name in entries:
        if name.endswith("index0"):
            return os.path.join(V4L_BY_ID_DIR, name)
    if entries:
        return os.path.join(V4L_BY_ID_DIR, entries[0])
    return None

_ffmpeg_process = None
_current_file_path = None
_ffmpeg_err_file = None


def _check_dependencies() -> str:
    """Ensures ffmpeg is available and returns the resolved camera device."""
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is not installed or not found in PATH.")
    device = resolve_camera_device()
    if device is None:
        raise RuntimeError("No camera detected (nothing in /dev/v4l/by-id/). "
                           "Plug in the USB camera, or set CAMERA_DEVICE explicitly.")
    return device


def is_recording() -> bool:
    """Returns True if FFmpeg video recording is currently active."""
    global _ffmpeg_process
    return _ffmpeg_process is not None and _ffmpeg_process.poll() is None


def start_recording():
    """
    Starts FFmpeg in the background to record video.
    Returns:
        tuple: (subprocess.Popen object, str output_file_path) or (None, None) on failure
    """
    global _ffmpeg_process, _current_file_path, _ffmpeg_err_file

    if is_recording():
        logger.warning("Recording is already in progress.")
        return _ffmpeg_process, _current_file_path

    global _camera_warning_issued
    if not ENABLED:
        if not _camera_warning_issued:
            logger.info("In-process video recording disabled (VIDEO_RECORDER_ENABLED=0); "
                        "flight_video_recorder.py records the flight instead.")
            _camera_warning_issued = True
        return None, None
    try:
        device = _check_dependencies()
    except Exception as e:
        if not _camera_warning_issued:
            logger.warning(f"Video recording unavailable: {e}")
            _camera_warning_issued = True
        return None, None
    _camera_warning_issued = False

    os.makedirs(RECORDINGS_DIR, exist_ok=True)

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"flight_{timestamp}.mkv"
    _current_file_path = os.path.join(RECORDINGS_DIR, filename)

    # Note: '-c:v copy' ensures we do not re-encode the MJPEG stream, saving CPU
    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-loglevel", "error",
        "-nostats",
        "-f", "v4l2",
        "-input_format", "mjpeg",
        "-video_size", "1280x720",
        "-framerate", "30",
        "-i", device,
        "-c:v", "copy",
        _current_file_path
    ]

    try:
        _ffmpeg_err_file = tempfile.TemporaryFile(mode="w+")

        start_wall = time.time()
        _ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=_ffmpeg_err_file,
            text=True
        )
        
        # Short sleep to catch immediate startup errors (e.g., device busy or permission denied)
        time.sleep(0.5)
        if _ffmpeg_process.poll() is not None:
            _ffmpeg_err_file.seek(0)
            stderr_output = _ffmpeg_err_file.read()
            _ffmpeg_err_file.close()
            _ffmpeg_err_file = None
            raise RuntimeError(f"FFmpeg process terminated unexpectedly on startup. Stderr:\n{stderr_output}")

        logger.info(f"Started recording video to: {_current_file_path} (camera: {device})")
        _write_start_sidecar(_current_file_path, start_wall)
        return _ffmpeg_process, _current_file_path

    except Exception as e:
        logger.error(f"Failed to start recording: {e}")
        _ffmpeg_process = None
        _current_file_path = None
        if _ffmpeg_err_file is not None:
            _ffmpeg_err_file.close()
            _ffmpeg_err_file = None
        raise


def _write_start_sidecar(video_path, start_wall):
    """Write flight_<ts>.json next to the video with the millisecond start time.

    The filename only has 1 s resolution; vision/detect_cones_video.py uses this
    to line frames up with the telemetry log. Never allowed to break recording.
    """
    try:
        with open(os.path.splitext(video_path)[0] + ".json", "w") as f:
            json.dump({
                "video": os.path.basename(video_path),
                "start_epoch": round(start_wall, 3),
                "start_local": datetime.datetime.fromtimestamp(start_wall).isoformat(
                    timespec="milliseconds"),
            }, f)
    except Exception as e:
        logger.warning(f"Could not write video start-time sidecar: {e}")


def stop_recording():
    """
    Stops FFmpeg gracefully and waits for it to finalize the video file.
    Returns:
        str: The saved file path, or None if no recording was active.
    """
    global _ffmpeg_process, _current_file_path, _ffmpeg_err_file
    
    if _ffmpeg_process is None:
        return None

    saved_path = _current_file_path
    logger.info("Stopping video recording gracefully...")
    
    if _ffmpeg_process.poll() is not None:
        logger.warning("FFmpeg process was already stopped unexpectedly.")
    else:
        try:
            # Send 'q' to gracefully stop ffmpeg and finalize the MKV header
            _ffmpeg_process.communicate(input='q\n', timeout=5.0)
        except subprocess.TimeoutExpired:
            logger.warning("FFmpeg did not quit gracefully within timeout. Sending SIGINT...")
            _ffmpeg_process.send_signal(signal.SIGINT)
            try:
                _ffmpeg_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                logger.error("FFmpeg still hanging. Forcing kill (SIGKILL)...")
                _ffmpeg_process.kill()
                _ffmpeg_process.wait()
        except Exception as e:
            logger.error(f"Error while stopping FFmpeg: {e}")

    logger.info(f"Recording stopped and finalized: {saved_path}")

    _ffmpeg_process = None
    _current_file_path = None
    if _ffmpeg_err_file is not None:
        _ffmpeg_err_file.close()
        _ffmpeg_err_file = None
    
    return saved_path


# Register the cleanup function to ensure we don't leave ffmpeg running on exit
atexit.register(stop_recording)
