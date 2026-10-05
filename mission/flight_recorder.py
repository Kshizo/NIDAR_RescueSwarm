#!/usr/bin/env python3
"""
Passive ArduPilot flight-data recorder.

Designed for diagnosing manual/Stabilize and PosHold flights.  It never arms,
changes flight mode, sends setpoints, writes parameters, or starts missions.
It only:
  * keeps the configured companion/GCS heartbeat alive;
  * requests diagnostic telemetry streams (temporary MAVLink stream rates);
  * records raw MAVLink telemetry (.tlog), decoded messages (.jsonl), events,
    and a non-invasive parameter snapshot.

Only one process may read the FC serial port.  This recorder deliberately uses
the same flock as ardupilot_mission.py, so run it *instead of* the mission
daemon during a manually flown diagnostic flight.
"""

from __future__ import annotations

import fcntl
import json
import logging
import os
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pymavlink import mavutil


PROJECT_DIR = Path(__file__).resolve().parent
SERIAL_BY_ID_DIR = Path("/dev/serial/by-id")
LOCK_FILE = Path(os.getenv("ARDUPILOT_LOCK_FILE", "/tmp/ardupilot_mission.lock"))
CONNECTION_STRING = os.getenv("ARDUPILOT_CONNECTION", "")
BAUD_RATE = int(os.getenv("ARDUPILOT_BAUD", "115200"))
LOG_ROOT = Path(os.getenv("FLIGHT_LOG_DIR", str(PROJECT_DIR / "flight_logs")))
RECONNECT_DELAY_S = 3.0
HEARTBEAT_PERIOD_S = 1.0

# These requests only control what the FC transmits over MAVLink for this
# connection; they do not alter persistent FC parameters or flight behaviour.
STREAMS_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2,
    mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 2,
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION: 1,
    mavutil.mavlink.MAVLINK_MSG_ID_FENCE_STATUS: 2,
    mavutil.mavlink.MAVLINK_MSG_ID_POWER_STATUS: 2,
}

STOP_REQUESTED = False


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def resolve_connection() -> str | None:
    """Find the FC by its stable udev symlink on every reconnect."""
    if CONNECTION_STRING:
        return CONNECTION_STRING
    try:
        entries = sorted(SERIAL_BY_ID_DIR.iterdir())
    except OSError:
        entries = []
    for entry in entries:
        if "ArduPilot" in entry.name or "MicoAir" in entry.name:
            return str(entry)
    if entries:
        return str(entries[0])
    try:
        return next(str(path) for path in sorted(Path("/dev").glob("ttyACM*")))
    except StopIteration:
        return None


def acquire_lock() -> Any:
    handle = LOCK_FILE.open("w")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise RuntimeError(
            "The mission daemon (or another serial client) owns the FC link. "
            "Stop ardupilot-mission.service before starting the recorder."
        )
    return handle


def json_safe(value: Any) -> Any:
    """Convert MAVLink values, including byte arrays, to JSON-safe values."""
    if isinstance(value, bytes):
        return value.rstrip(b"\x00").decode("utf-8", errors="replace")
    if isinstance(value, bytearray):
        return list(value)
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


class FlightRecorder:
    def __init__(self) -> None:
        run_stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.run_dir = LOG_ROOT / f"flight-{run_stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.events_path = self.run_dir / "events.jsonl"
        self.decoded_path = self.run_dir / "telemetry.jsonl"
        self.params_path = self.run_dir / "parameters.json"
        self.metadata_path = self.run_dir / "metadata.json"
        self.event_file = self.events_path.open("a", encoding="utf-8", buffering=1)
        self.decoded_file = self.decoded_path.open("a", encoding="utf-8", buffering=1)
        self.parameters: dict[str, dict[str, Any]] = {}
        self.parameter_total: int | None = None
        self.last_armed: bool | None = None
        self.last_mode: str | None = None
        self.last_heartbeat_sent = 0.0
        self.connection_number = 0
        self.logger = logging.getLogger("flight_recorder")
        self._write_metadata()

    def _write_metadata(self) -> None:
        payload = {
            "started_utc": utc_now(),
            "purpose": "passive manual/PosHold flight diagnosis",
            "connection": CONNECTION_STRING or "auto-discover",
            "baud_rate": BAUD_RATE,
            "streams_hz": {str(msg_id): rate for msg_id, rate in STREAMS_HZ.items()},
            "safety": [
                "No arming commands", "No mode-change commands", "No movement commands",
                "No persistent parameter writes", "GCS heartbeat only",
            ],
        }
        self.metadata_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def event(self, kind: str, **details: Any) -> None:
        row = {"utc": utc_now(), "kind": kind, **json_safe(details)}
        self.event_file.write(json.dumps(row, sort_keys=True) + "\n")
        self.logger.info("%s %s", kind, json.dumps(json_safe(details), sort_keys=True))

    def record_message(self, msg: Any) -> None:
        row = {
            "utc": utc_now(),
            "message_type": msg.get_type(),
            "source_system": msg.get_srcSystem(),
            "source_component": msg.get_srcComponent(),
            "fields": json_safe(msg.to_dict()),
        }
        self.decoded_file.write(json.dumps(row, sort_keys=True) + "\n")

    def record_parameter(self, msg: Any) -> None:
        raw_name = msg.param_id
        if isinstance(raw_name, bytes):
            name = raw_name.rstrip(b"\x00").decode("ascii", errors="replace")
        else:
            name = str(raw_name).rstrip("\x00")
        self.parameters[name] = {
            "value": msg.param_value,
            "type": msg.param_type,
            "index": msg.param_index,
        }
        if msg.param_count >= 0:
            self.parameter_total = msg.param_count

    def flush_parameters(self, complete: bool = False) -> None:
        payload = {
            "captured_utc": utc_now(),
            "complete": complete,
            "reported_parameter_count": self.parameter_total,
            "captured_parameter_count": len(self.parameters),
            "parameters": self.parameters,
        }
        temp_path = self.params_path.with_suffix(".json.tmp")
        temp_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temp_path.replace(self.params_path)

    def close(self) -> None:
        self.flush_parameters(
            self.parameter_total is not None and len(self.parameters) >= self.parameter_total
        )
        self.event("recorder_stopped")
        self.event_file.close()
        self.decoded_file.close()


def setup_process_logging(run_dir: Path) -> logging.Logger:
    logger = logging.getLogger("flight_recorder")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
    file_handler = logging.FileHandler(run_dir / "recorder.log", encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)
    return logger


def mode_name(master: Any, heartbeat: Any) -> str:
    mapping = master.mode_mapping() or {}
    reverse_mapping = {value: name for name, value in mapping.items()}
    return reverse_mapping.get(heartbeat.custom_mode, f"CUSTOM_{heartbeat.custom_mode}")


def send_gcs_heartbeat(master: Any) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def request_streams(master: Any, recorder: FlightRecorder) -> None:
    for message_id, hz in STREAMS_HZ.items():
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            int(1_000_000 / hz),
            0, 0, 0, 0, 0,
        )
        time.sleep(0.015)
    recorder.event("diagnostic_streams_requested", stream_count=len(STREAMS_HZ))


def request_parameter_snapshot(master: Any, recorder: FlightRecorder) -> None:
    # This is a read-only MAVLink request.  The stream arrives asynchronously and
    # is saved as it is received, without holding up normal telemetry recording.
    master.mav.param_request_list_send(master.target_system, master.target_component)
    recorder.event("parameter_snapshot_requested")


def status_text(msg: Any) -> str:
    value = msg.text
    if isinstance(value, bytes):
        return value.rstrip(b"\x00").decode("utf-8", errors="replace")
    return str(value).rstrip("\x00")


def handle_message(master: Any, recorder: FlightRecorder, msg: Any) -> None:
    recorder.record_message(msg)
    kind = msg.get_type()
    if kind == "PARAM_VALUE":
        recorder.record_parameter(msg)
        return
    if kind == "HEARTBEAT":
        if msg.get_srcSystem() != master.target_system:
            return
        if msg.get_srcComponent() not in (0, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
            return
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mode_name(master, msg)
        if armed != recorder.last_armed:
            recorder.event("arming_state", armed=armed, mode=mode)
            recorder.last_armed = armed
        if mode != recorder.last_mode:
            recorder.event("flight_mode", mode=mode, armed=armed)
            recorder.last_mode = mode
        return
    if kind == "STATUSTEXT":
        text = status_text(msg)
        lowered = text.lower()
        severity = int(getattr(msg, "severity", -1))
        event_kind = "fc_status"
        if lowered.startswith(("prearm", "arm:")):
            event_kind = "prearm"
        elif any(token in lowered for token in ("failsafe", "ekf", "compass", "mag", "battery", "gps", "fence")):
            event_kind = "fc_safety_status"
        recorder.event(event_kind, severity=severity, text=text)
        return
    if kind == "EKF_STATUS_REPORT":
        recorder.event("ekf_status", flags=msg.flags, velocity_variance=msg.velocity_variance,
                       pos_horiz_variance=msg.pos_horiz_variance,
                       pos_vert_variance=msg.pos_vert_variance,
                       compass_variance=msg.compass_variance)
        return
    if kind == "FENCE_STATUS" and msg.breach_status:
        recorder.event("fence_breach", status=msg.breach_status, count=msg.breach_count)


def close_master(master: Any) -> None:
    try:
        logfile = getattr(master, "logfile", None)
        if logfile is not None:
            logfile.flush()
            logfile.close()
    except Exception:
        pass
    try:
        master.close()
    except Exception:
        pass


def run_connection(master: Any, recorder: FlightRecorder) -> None:
    recorder.connection_number += 1
    tlog_path = recorder.run_dir / f"mavlink-{recorder.connection_number:03d}.tlog"
    master.setup_logfile(str(tlog_path), mode="wb")
    heartbeat = master.wait_heartbeat(timeout=12)
    if heartbeat is None:
        raise TimeoutError("No FC heartbeat within 12 seconds")
    if master.target_component in (0, mavutil.mavlink.MAV_COMP_ID_ALL):
        master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    recorder.event(
        "fc_connected", port=master.address, system_id=master.target_system,
        component_id=master.target_component, tlog=tlog_path.name,
    )
    request_streams(master, recorder)
    request_parameter_snapshot(master, recorder)
    recorder.event("recorder_ready", instruction="Manual/PosHold flight may begin; recorder is passive.")

    last_param_flush = time.monotonic()
    while not STOP_REQUESTED:
        now = time.monotonic()
        if now - recorder.last_heartbeat_sent >= HEARTBEAT_PERIOD_S:
            send_gcs_heartbeat(master)
            recorder.last_heartbeat_sent = now
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is not None:
            handle_message(master, recorder, msg)
        if now - last_param_flush >= 10.0:
            complete = recorder.parameter_total is not None and len(recorder.parameters) >= recorder.parameter_total
            recorder.flush_parameters(complete)
            last_param_flush = now


def signal_handler(signum: int, _frame: Any) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def main() -> int:
    os.umask(0o077)
    try:
        lock = acquire_lock()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    recorder = FlightRecorder()
    logger = setup_process_logging(recorder.run_dir)
    recorder.logger = logger
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    logger.info("Passive flight recorder started: %s", recorder.run_dir)
    recorder.event("recorder_started", run_directory=str(recorder.run_dir))

    try:
        while not STOP_REQUESTED:
            connection = resolve_connection()
            if connection is None:
                recorder.event("fc_waiting", reason="serial device not found")
                time.sleep(RECONNECT_DELAY_S)
                continue
            master = None
            try:
                recorder.event("fc_connecting", port=connection)
                master = mavutil.mavlink_connection(
                    connection, baud=BAUD_RATE, dialect="ardupilotmega", autoreconnect=True
                )
                run_connection(master, recorder)
            except Exception as exc:
                if not STOP_REQUESTED:
                    recorder.event("fc_connection_error", error=str(exc))
                    logger.warning("FC connection ended: %s; retrying in %.0fs", exc, RECONNECT_DELAY_S)
                    time.sleep(RECONNECT_DELAY_S)
            finally:
                if master is not None:
                    close_master(master)
    finally:
        recorder.close()
        lock.close()
        logger.info("Passive flight recorder stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
