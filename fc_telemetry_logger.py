#!/usr/bin/env python3
"""
Verbose flight-controller telemetry logger.
===========================================

Records everything the flight controller says, into its own rotating log file,
while the raster mission daemon is running.

WHY THIS IS NOT flight_recorder.py
----------------------------------
``flight_recorder.py`` opens its OWN MAVLink connection, which is why its unit
declares ``Conflicts=ardupilot-mission.service``. Two readers on one serial link
consume each other's bytes and corrupt the stream. It therefore cannot run
alongside a mission.

This module opens nothing. It attaches a pymavlink message hook to the SAME
connection object the mission already uses, so it sees every message from inside
the existing single receive path. No extra reader, no extra serial traffic, and
nothing is requested from the flight controller that the mission did not already
request (unless FC_LOG_EXTRA_STREAMS=1 is set deliberately — see below).

NEVER BLOCK THE RECEIVE PATH
----------------------------
The hook runs on the telemetry listener thread. If it wrote to the SD card
directly, a slow flush would stall MAVLink reception during flight. So the hook
only puts a tuple on a bounded queue and returns; a separate daemon thread does
all formatting and disk I/O. If the queue ever fills, messages are DROPPED and
counted rather than allowed to back up into the control path. The drop count is
reported in the log, so a gap is always visible rather than silent.

OUTPUT
------
``fc_telemetry.log``  verbose, human readable, size-rotated (default 5 x 20 MB)
``fc_raw_<stamp>.tlog`` raw MAVLink, loadable in Mission Planner / MAVExplorer

Verbosity (FC_LOG_LEVEL):
  full     every message with every field, plus 1 Hz SUMMARY, plus events  (default)
  summary  1 Hz SUMMARY lines plus events only — quiet, still tells the story
  events   events only: STATUSTEXT, mode/arm changes, failsafes, EKF, fence, acks

Events are always marked ``***`` so they can be grepped out of a full log:
    grep '\\*\\*\\*' fc_telemetry.log
"""

import atexit
import json
import logging
import logging.handlers
import os
import queue
import threading
import time

from pymavlink import mavutil

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

FC_LOG_FILE = os.getenv("FC_LOG_FILE", os.path.join(PROJECT_DIR, "fc_telemetry.log"))
FC_LOG_LEVEL = os.getenv("FC_LOG_LEVEL", "full").strip().lower()
FC_LOG_MAX_BYTES = int(os.getenv("FC_LOG_MAX_BYTES", str(20 * 1024 * 1024)))
FC_LOG_BACKUPS = int(os.getenv("FC_LOG_BACKUPS", "5"))
FC_LOG_QUEUE_MAX = int(os.getenv("FC_LOG_QUEUE_MAX", "20000"))
FC_LOG_SUMMARY_HZ = float(os.getenv("FC_LOG_SUMMARY_HZ", "1.0"))
FC_LOG_TLOG = os.getenv("FC_LOG_TLOG", "1") == "1"
FC_LOG_TLOG_DIR = os.getenv("FC_LOG_TLOG_DIR", os.path.join(PROJECT_DIR, "flight_logs"))

# Extra telemetry streams. DEFAULT OFF on purpose: the known-good baseline runs
# with the stream set the mission already requests, and adding traffic to a
# 115200 baud link shared with the control messages is a deliberate decision, not
# a side effect of turning logging on.
FC_LOG_EXTRA_STREAMS = os.getenv("FC_LOG_EXTRA_STREAMS", "0") == "1"
_EXTRA_STREAMS = [
    ("ATTITUDE", 10),
    ("VFR_HUD", 4),
    ("VIBRATION", 1),
    ("SERVO_OUTPUT_RAW", 2),
    ("NAV_CONTROLLER_OUTPUT", 2),
    ("SCALED_PRESSURE", 1),
]

# Messages whose arrival is itself an event worth calling out.
_EVENT_TYPES = {
    "STATUSTEXT", "COMMAND_ACK", "FENCE_STATUS", "EKF_STATUS_REPORT",
    "HIGH_LATENCY2", "MISSION_ACK", "PARAM_VALUE",
}

_QUIET_TYPES = {"ATTITUDE", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT",
                "RC_CHANNELS", "SERVO_OUTPUT_RAW", "SCALED_PRESSURE", "RAW_IMU"}


def _make_logger():
    log = logging.getLogger("fc_telemetry")
    log.setLevel(logging.INFO)
    log.propagate = False                     # keep out of the mission log
    if not log.handlers:
        handler = logging.handlers.RotatingFileHandler(
            FC_LOG_FILE, maxBytes=FC_LOG_MAX_BYTES, backupCount=FC_LOG_BACKUPS)
        handler.setFormatter(logging.Formatter(
            "%(asctime)s.%(msecs)03d | %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
        log.addHandler(handler)
    return log


class _State:
    """Latest value of each thing worth putting on the 1 Hz SUMMARY line."""

    def __init__(self):
        self.mode = "?"
        self.armed = None
        self.alt_rel = float("nan")
        self.lat = float("nan")
        self.lon = float("nan")
        self.north = float("nan")
        self.east = float("nan")
        self.down = float("nan")
        self.vx = self.vy = self.vz = float("nan")
        self.volt = float("nan")
        self.curr = float("nan")
        self.batt_pct = None
        self.fix = None
        self.sats = None
        self.eph = None
        self.ch8 = None
        self.rssi = None
        self.ekf_flags = None
        self.ekf_vel = self.ekf_pos = self.ekf_cmp = float("nan")
        self.fence_breach = None
        self.sys_status_errors = None


class FCTelemetryLogger:
    def __init__(self):
        self.log = _make_logger()
        self.queue = queue.Queue(maxsize=FC_LOG_QUEUE_MAX)
        self.state = _State()
        self.dropped = 0
        self.seen = 0
        self.counts = {}
        self._last_summary = 0.0
        self._last_drop_report = 0.0
        self._thread = None
        self._stop = threading.Event()
        self._tlog = None
        self._attached_to = None

    # -- receive path: must stay trivial ------------------------------------
    def hook(self, connection, msg):
        try:
            # Wall clock on purpose: the .tlog container stores Unix-epoch
            # microseconds per frame. Everything that measures an AGE or a
            # TIMEOUT uses time.monotonic() instead - see _write() below.
            self.queue.put_nowait((time.time(), msg))
        except queue.Full:
            self.dropped += 1
        except Exception:
            pass                                   # never break the receive path

    # -- writer thread -------------------------------------------------------
    def _run(self):
        while not (self._stop.is_set() and self.queue.empty()):
            try:
                stamp, msg = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._write(stamp, msg)
            except Exception as exc:
                try:
                    self.log.info(f"*** LOGGER ERROR while writing: {exc!r}")
                except Exception:
                    pass

    def _write(self, stamp, msg):
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            self.log.info("*** BAD_DATA on the link (corrupt or partial frame)")
            return

        self.seen += 1
        self.counts[mtype] = self.counts.get(mtype, 0) + 1

        if self._tlog is not None:
            try:
                buf = msg.get_msgbuf()
                self._tlog.write(int(stamp * 1.0e6).to_bytes(8, "big") + bytes(buf))
            except Exception:
                pass

        event = self._track(msg, mtype)

        if FC_LOG_LEVEL == "full":
            src = f"{msg.get_srcSystem()}.{msg.get_srcComponent()}"
            self.log.info(f"{mtype:<24} src={src:<6} {self._fields(msg)}")
        if event:
            self.log.info(f"*** {event}")

        now = time.monotonic()
        if FC_LOG_SUMMARY_HZ > 0 and (now - self._last_summary) >= (1.0 / FC_LOG_SUMMARY_HZ):
            self._last_summary = now
            self.log.info(self._summary())
        if self.dropped and (now - self._last_drop_report) >= 10.0:
            self._last_drop_report = now
            self.log.info(f"*** QUEUE OVERFLOW: {self.dropped} messages dropped so far "
                          "(disk slower than the link; log has gaps)")

    @staticmethod
    def _fields(msg):
        try:
            d = msg.to_dict()
            d.pop("mavpackettype", None)
            parts = []
            for key, value in d.items():
                if isinstance(value, float):
                    parts.append(f"{key}={value:.6g}")
                elif isinstance(value, (bytes, bytearray)):
                    parts.append(f"{key}={value.decode('utf-8', 'ignore').strip()!r}")
                else:
                    parts.append(f"{key}={value}")
            return " ".join(parts)
        except Exception as exc:
            return f"<unformattable: {exc!r}>"

    def _track(self, msg, mtype):
        """Updates the summary state; returns an event string when one happened."""
        st = self.state
        if mtype == "HEARTBEAT":
            if msg.get_srcComponent() not in (0, 1):
                return None
            try:
                mode = mavutil.mode_string_v10(msg)
            except Exception:
                mode = str(msg.custom_mode)
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            events = []
            if mode != st.mode:
                events.append(f"FLIGHT MODE: {st.mode} -> {mode}")
                st.mode = mode
            if armed != st.armed:
                events.append("ARMED" if armed else
                              ("DISARMED" if st.armed is not None else "state: DISARMED"))
                st.armed = armed
            return "; ".join(events) if events else None

        if mtype == "STATUSTEXT":
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode("utf-8", "ignore")
            sev = getattr(msg, "severity", 6)
            names = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
                     4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
            return f"AP {names.get(sev, sev)}: {text.strip()}"

        if mtype == "GLOBAL_POSITION_INT":
            st.lat, st.lon = msg.lat / 1e7, msg.lon / 1e7
            st.alt_rel = msg.relative_alt / 1000.0
            return None
        if mtype == "LOCAL_POSITION_NED":
            st.north, st.east, st.down = msg.x, msg.y, msg.z
            st.vx, st.vy, st.vz = msg.vx, msg.vy, msg.vz
            return None
        if mtype == "SYS_STATUS":
            st.volt = msg.voltage_battery / 1000.0
            st.curr = msg.current_battery / 100.0
            st.batt_pct = msg.battery_remaining
            errors = (msg.errors_count1 or msg.errors_count2
                      or msg.errors_count3 or msg.errors_count4)
            if errors and errors != st.sys_status_errors:
                st.sys_status_errors = errors
                return (f"SYS_STATUS error counts: {msg.errors_count1}/{msg.errors_count2}/"
                        f"{msg.errors_count3}/{msg.errors_count4}")
            return None
        if mtype == "BATTERY_STATUS":
            try:
                st.volt = msg.voltages[0] / 1000.0
            except Exception:
                pass
            if msg.battery_remaining not in (-1, None):
                st.batt_pct = msg.battery_remaining
            return None
        if mtype == "GPS_RAW_INT":
            prev = st.fix
            st.fix, st.sats = msg.fix_type, msg.satellites_visible
            st.eph = msg.eph
            if prev is not None and msg.fix_type != prev:
                return f"GPS FIX: {prev} -> {msg.fix_type} (sats={msg.satellites_visible})"
            return None
        if mtype == "RC_CHANNELS":
            st.ch8 = msg.chan8_raw
            st.rssi = msg.rssi
            return None
        if mtype == "EKF_STATUS_REPORT":
            prev = st.ekf_flags
            st.ekf_flags = msg.flags
            st.ekf_vel = msg.velocity_variance
            st.ekf_pos = msg.pos_horiz_variance
            st.ekf_cmp = msg.compass_variance
            if prev is not None and msg.flags != prev:
                return f"EKF FLAGS: 0x{prev:04x} -> 0x{msg.flags:04x}"
            return None
        if mtype == "FENCE_STATUS":
            breached = msg.breach_status != 0
            if breached != st.fence_breach:
                st.fence_breach = breached
                return (f"FENCE BREACH (status={msg.breach_status}, count={msg.breach_count})"
                        if breached else "FENCE breach cleared")
            return None
        if mtype == "COMMAND_ACK":
            return f"COMMAND_ACK cmd={msg.command} result={msg.result}"
        return None

    def _summary(self):
        s = self.state
        armed = "ARMED" if s.armed else ("disarmed" if s.armed is not None else "?")
        return (f"SUMMARY mode={s.mode} {armed} alt={s.alt_rel:.2f}m "
                f"ned=({s.north:.2f},{s.east:.2f},{s.down:.2f}) "
                f"gps=({s.lat:.7f},{s.lon:.7f}) fix={s.fix} sats={s.sats} eph={s.eph} "
                f"batt={s.volt:.2f}V/{s.curr:.1f}A/{s.batt_pct}% "
                f"ch8={s.ch8} rssi={s.rssi} "
                f"ekf=0x{(s.ekf_flags or 0):04x} v={s.ekf_vel:.2f} p={s.ekf_pos:.2f} "
                f"c={s.ekf_cmp:.2f} rx={self.seen} drop={self.dropped}")

    # -- lifecycle -----------------------------------------------------------
    def start(self, master, mission_logger=None):
        if self._attached_to is master:
            return
        self._attached_to = master

        if FC_LOG_TLOG:
            try:
                os.makedirs(FC_LOG_TLOG_DIR, exist_ok=True)
                stamp = time.strftime("%Y%m%d-%H%M%S")
                path = os.path.join(FC_LOG_TLOG_DIR, f"fc_raw_{stamp}.tlog")
                self._tlog = open(path, "wb")
                self.log.info(f"*** RAW TLOG: {path}")
            except Exception as exc:
                self._tlog = None
                self.log.info(f"*** could not open tlog: {exc!r}")

        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True,
                                            name="fc-telemetry-writer")
            self._thread.start()

        if self.hook not in master.message_hooks:
            master.message_hooks.append(self.hook)

        self.log.info("=" * 74)
        self.log.info(f"*** FC TELEMETRY LOG OPENED  level={FC_LOG_LEVEL} "
                      f"summary={FC_LOG_SUMMARY_HZ}Hz extra_streams={FC_LOG_EXTRA_STREAMS}")
        self.log.info("=" * 74)

        if FC_LOG_EXTRA_STREAMS:
            self._request_extra(master, mission_logger)

        if mission_logger:
            mission_logger.info(
                f"[FC LOG] Verbose flight-controller log attached to the existing MAVLink "
                f"receive path (no additional reader) -> {FC_LOG_FILE} "
                f"[level={FC_LOG_LEVEL}]")

    def _request_extra(self, master, mission_logger):
        for name, hz in _EXTRA_STREAMS:
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
            if msg_id is None:
                continue
            try:
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
                self.log.info(f"*** requested extra stream {name} at {hz}Hz")
            except Exception as exc:
                self.log.info(f"*** extra stream {name} failed: {exc!r}")
        if mission_logger:
            mission_logger.warning(
                "[FC LOG] FC_LOG_EXTRA_STREAMS=1: extra telemetry streams requested. "
                "This adds traffic to the 115200 baud link shared with control messages.")

    def close(self):
        try:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=5.0)
            top = sorted(self.counts.items(), key=lambda kv: -kv[1])[:12]
            self.log.info(f"*** FC TELEMETRY LOG CLOSING: {self.seen} messages, "
                          f"{self.dropped} dropped")
            self.log.info("*** message counts: "
                          + ", ".join(f"{k}={v}" for k, v in top))
            for handler in self.log.handlers:
                handler.flush()
        except Exception:
            pass
        if self._tlog is not None:
            try:
                self._tlog.flush()
                self._tlog.close()
            except Exception:
                pass
            self._tlog = None


RECORDER = FCTelemetryLogger()
atexit.register(RECORDER.close)


def attach(master, mission_logger=None):
    """Attaches the verbose logger to an already-open MAVLink connection."""
    RECORDER.start(master, mission_logger)
