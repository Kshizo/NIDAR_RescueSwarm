#!/usr/bin/env python3
"""
ArduPilot Continuous Autonomous Drone Mission Daemon & Video Recording
========================================================================
Target Hardware:
- Flight Controller: MicoAir743v2 / Matek / Pixhawk (ArduPilot / ArduCopter Firmware)
- Companion Computer: Raspberry Pi 5 (Ubuntu 24.04 ARM64)
- Radio / Receiver: RadioMaster TX15 / ExpressLRS (ELRS)
- Camera: Microsoft LifeCam HD-3000 (/dev/video0)
- Protocol: pymavlink (MAVLink dialect: ardupilotmega over serial)

RC Channel 8 (Switch SC on TX15) is the explicit control selector:
  CH8 ≈ 1000 (< 1250): MANUAL — Pilot has full control. Companion stops all
                         autonomous velocity commands immediately.
  CH8 ≈ 1500 (1250–1750): GUIDED / AUTONOMOUS — Companion may run the
                         autonomous mission (subject to all safety checks).
  CH8 ≈ 2000 (> 1750): LAND — Companion commands ArduPilot LAND immediately.

Autonomous Mission Sequence (CH8 ≈ 1500, on ground):
  1. Enter GUIDED mode.
  2. Arm if necessary.
  3. Takeoff to 1.2 m target altitude.
  4. Verify altitude is within ±0.15 m of target and stable.
  5. Capture the local-NED origin, fly a 3 m orbit around it, and return to it.
  6. Stop horizontal movement.
  7. Execute ultra-slow multi-phase soft landing for concrete/fragile landing gear.
  8. Confirm touchdown and disarm before declaring success.

Safety:
  - Geofence OFF by default (CONFIGURE_FENCE=0): containment is the pilot's job,
    via manual takeover on the transmitter. Set CONFIGURE_FENCE=1 to restore it.
  - If the fence is re-enabled, FENCE_ACTION = 2 (Always Land) — no RTL in small
    areas — and the companion monitors FENCE_STATUS, stopping all autonomous
    horizontal velocity commands and commanding LAND on a breach.
  - Battery failsafes enabled (LAND on low/critical battery).
  - EKF health, telemetry freshness, and heartbeat checks during autonomous flight.
  - Single MAVLink receive path (telemetry listener thread only).
  - set_flight_mode verifies the mode was actually entered.
  - Unexpected exceptions during mission trigger safe-state transition.

Continuous Architecture:
  - Runs persistently in background. Never exits after a mission, error, or
    takeover. Automatically reconnects when FC is power-cycled.
  - Ultra-lightweight: Non-blocking telemetry caching (<1% CPU).
  - Zero-CPU video recording via direct MJPEG stream copy.
"""

import fcntl
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from logging.handlers import RotatingFileHandler
import logging

from pymavlink import mavutil
import video_recorder

# ---------------------------------------------------------------------------
# Configuration Parameters
# ---------------------------------------------------------------------------
# Empty means "auto-discover the flight controller" (see resolve_connection_string).
# Hardcoding /dev/ttyACM0 was a real failure mode: the CDC-ACM device number is
# not stable, so power-cycling the flight controller (a battery swap, for example)
# moves it to /dev/ttyACM1 and the daemon then waits forever on a port that no
# longer exists.
CONNECTION_STRING = os.getenv("ARDUPILOT_CONNECTION", "")
BAUD_RATE = int(os.getenv("ARDUPILOT_BAUD", "115200"))
TAKEOFF_ALTITUDE_M = float(os.getenv("TAKEOFF_ALTITUDE_M", "1.2"))          # Target altitude (m)
# Takeoff climb rate and vertical acceleration for GUIDED/AUTO climbs.
# ArduPilot defaults (WPNAV_SPEED_UP=250 cm/s, WPNAV_ACCEL_Z=100 cm/s/s) spool the
# motors hard to break ground, which is the largest current spike of the whole
# flight and therefore the largest voltage sag. Climbing gently trades a couple of
# seconds for a noticeably shallower sag. Both are clamped to ArduPilot's own
# accepted ranges (SPEED_UP >= 10 cm/s, ACCEL_Z >= 50 cm/s/s).
TAKEOFF_CLIMB_SPEED_MPS = float(os.getenv("TAKEOFF_CLIMB_SPEED_MPS", "0.5"))
TAKEOFF_CLIMB_ACCEL_MPS2 = float(os.getenv("TAKEOFF_CLIMB_ACCEL_MPS2", "0.5"))
DESCENT_SPEED_MPS = float(os.getenv("DESCENT_SPEED_MPS", "0.15"))           # Ultra-slow descent (m/s)
TOUCHDOWN_SPEED_MPS = float(os.getenv("TOUCHDOWN_SPEED_MPS", "0.08"))       # Final cushion touchdown (m/s)
CONCRETE_SLOWDOWN_ALT_M = float(os.getenv("CONCRETE_SLOWDOWN_ALT_M", "0.40"))  # Transition altitude (m)
FORWARD_SPEED_MPS = float(os.getenv("FORWARD_SPEED_MPS", "0.3"))            # Forward speed (m/s)
FORWARD_DURATION_S = float(os.getenv("FORWARD_DURATION_S", "5.0"))          # Forward duration (s)
# Horizontal-geofence orbit test. These companion-computer checks leave the
# existing ArduPilot fence parameters below as an independent backup layer.
HORIZONTAL_GEOFENCE_RADIUS_M = float(os.getenv("HORIZONTAL_GEOFENCE_RADIUS_M", "4.0"))
ORBIT_RADIUS_M = float(os.getenv("ORBIT_RADIUS_M", "3.0"))
ORBIT_LOOPS = float(os.getenv("ORBIT_LOOPS", "1.0"))
ORBIT_SPEED_MPS = float(os.getenv("ORBIT_SPEED_MPS", "0.3"))
HORIZONTAL_GEOFENCE_ABORT_MARGIN_M = float(os.getenv("HORIZONTAL_GEOFENCE_ABORT_MARGIN_M", "0.15"))
LOCAL_POSITION_MAX_AGE_S = float(os.getenv("LOCAL_POSITION_MAX_AGE_S", "1.0"))
LOCAL_POSITION_TOLERANCE_M = float(os.getenv("LOCAL_POSITION_TOLERANCE_M", "0.15"))
STREAM_HZ = 10.0                                                           # Setpoint streaming frequency
LOG_FILE = os.getenv("LOG_FILE", "/home/aahswarm/ardupilot_testing/ardupilot_mission.log")

# Single-instance lock. Two daemons sharing /dev/ttyACM0 steal bytes from each
# other, which corrupts MAVLink parsing (mode reads as UNKNOWN, heartbeats go
# missing mid-climb). The lock makes that impossible.
LOCK_FILE = os.getenv("ARDUPILOT_LOCK_FILE", "/tmp/ardupilot_mission.lock")

# Mission trigger. CH8 is the normal trigger, but the daemon must also work with
# no RC receiver attached: dropping this file in place starts one mission.
TRIGGER_FILE = os.getenv("MISSION_TRIGGER_FILE", "/home/aahswarm/ardupilot_testing/START_MISSION")
# Start one mission automatically as soon as the aircraft is ready, without RC
# and without a trigger file. Off by default — this arms the aircraft unattended.
AUTOSTART_WITHOUT_RC = os.getenv("AUTOSTART_WITHOUT_RC", "0") == "1"
# Start one mission without a file trigger or CH8 transition once the FC is
# connected and the pilot has deliberately left a live receiver in GUIDED.
# This preserves the manual-takeover path: CH8=MANUAL still aborts immediately.
# It is deliberately different from AUTOSTART_WITHOUT_RC, which has no live
# manual override and remains disabled for real flights.
AUTOSTART_ON_READY = os.getenv("AUTOSTART_ON_READY", "0") == "1"
# Trigger a mission when the aircraft is actually put into GUIDED flight mode
# (from a transmitter mode switch or a GCS), instead of on a CH8 band change.
# Off by default so the base horizontal-geofence daemon keeps its CH8 trigger.
# When enabled the CH8 launch condition is disabled, but every other CH8 role
# is untouched: CH8=MANUAL/LAND still aborts and hands the aircraft back.
TRIGGER_ON_GUIDED_MODE = os.getenv("TRIGGER_ON_GUIDED_MODE", "0") == "1"
# Minimum seconds between mission attempts (stops the abort/retry storm).
MISSION_COOLDOWN_S = float(os.getenv("MISSION_COOLDOWN_S", "20.0"))

# Bench mode: disables ArduPilot pre-arm checks so a bench rig with no battery
# monitor / no RC can still be armed. NEVER enable this for a real flight.
BENCH_MODE = os.getenv("BENCH_MODE", "0") == "1"

# Geofence configuration. Disabled by default: the pilot keeps the aircraft
# inside the test area by hand, taking manual control on the transmitter if it
# drifts. With CONFIGURE_FENCE=0 the daemon actively writes FENCE_ENABLE=0 so a
# fence left enabled on the flight controller from an earlier run is cleared —
# otherwise "PreArm: Fence requires position" keeps blocking the arm.
# Set CONFIGURE_FENCE=1 to put the fence back; FENCE_MARGIN must then stay below
# FENCE_RADIUS or the aircraft is "breaching" the moment it is powered up.
CONFIGURE_FENCE = os.getenv("CONFIGURE_FENCE", "0") == "1"
FENCE_RADIUS_M = float(os.getenv("FENCE_RADIUS_M", "15.0"))
FENCE_ALT_MAX_M = float(os.getenv("FENCE_ALT_MAX_M", "5.0"))

# Takeoff is considered to have failed if the aircraft has not left the ground
# by this many seconds after the takeoff command (thrust / power problem).
# Raised from 8s because TAKEOFF_CLIMB_* now deliberately slows the climb:
# a gentle spool-up takes longer to break ground, and 8s left too little margin.
NO_CLIMB_TIMEOUT_S = float(os.getenv("NO_CLIMB_TIMEOUT_S", "12.0"))
# AGL above which the aircraft counts as having actually left the ground.
LIFTOFF_ALT_M = 0.15

# CH8 thresholds with deadband/hysteresis
CH8_MANUAL_UPPER = 1250     # Below this → MANUAL
CH8_LAND_LOWER = 1750       # Above this → LAND
# Between CH8_MANUAL_UPPER and CH8_LAND_LOWER → GUIDED/AUTO

# Telemetry freshness: max age before considering telemetry stale
# Heartbeat is streamed at 2 Hz; allow several missed frames before aborting.
TELEMETRY_MAX_AGE_S = 5.0

# Altitude acceptance band for takeoff: target ± this value
ALTITUDE_ACCEPTANCE_BAND_M = 0.15
# Number of consecutive altitude-stable readings required
ALTITUDE_STABLE_COUNT = 5

# MAVLink Setpoint Bitmask: Ignore Position & Accel, Enable Velocity (vx, vy, vz) + Yaw Rate
SETPOINT_VELOCITY_YAW_RATE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)

logger = logging.getLogger("ardupilot_mission")

# Holds the single-instance file lock for the lifetime of the process.
_instance_lock = None

# Stable udev path for the flight controller. The by-id symlink encodes the
# board's serial number, so it survives re-enumeration; plain /dev/ttyACM* does not.
SERIAL_BY_ID_DIR = "/dev/serial/by-id"


def resolve_connection_string() -> str | None:
    """
    Returns the connection string to use, re-resolved on every connect attempt.

    Order of preference:
      1. An explicit ARDUPILOT_CONNECTION (including udp:/tcp: for testing).
      2. A /dev/serial/by-id/ entry that looks like an ArduPilot board.
      3. Any /dev/serial/by-id/ entry.
      4. The lowest-numbered /dev/ttyACM*.
    """
    if CONNECTION_STRING:
        return CONNECTION_STRING

    try:
        entries = sorted(os.listdir(SERIAL_BY_ID_DIR))
    except OSError:
        entries = []

    for name in entries:
        if "ArduPilot" in name or "MicoAir" in name:
            return os.path.join(SERIAL_BY_ID_DIR, name)
    if entries:
        return os.path.join(SERIAL_BY_ID_DIR, entries[0])

    try:
        acm = sorted(n for n in os.listdir("/dev") if n.startswith("ttyACM"))
    except OSError:
        acm = []
    if acm:
        return os.path.join("/dev", acm[0])

    return None


# ---------------------------------------------------------------------------
# CH8 classification helper
# ---------------------------------------------------------------------------
def classify_ch8(raw: int, rc_present: bool = True) -> str:
    """
    Classify CH8 PWM into MANUAL, GUIDED, LAND — or NO_SIGNAL.

    A raw value of 0 (or an RC_CHANNELS frame reporting chancount == 0) means the
    flight controller sees no receiver at all.  That is NOT a switch position and
    must never be mapped onto one: mapping it to GUIDED starts unwanted missions,
    mapping it to MANUAL aborts missions that were legitimately started without RC.
    """
    if not rc_present or raw <= 0:
        return "NO_SIGNAL"
    if raw < CH8_MANUAL_UPPER:
        return "MANUAL"
    elif raw > CH8_LAND_LOWER:
        return "LAND"
    else:
        return "GUIDED"


# ---------------------------------------------------------------------------
# Telemetry State
# ---------------------------------------------------------------------------
@dataclass
class TelemetryState:
    """Live telemetry cache updated asynchronously by background listener."""
    is_connected: bool = False
    is_armed: bool = False
    flight_mode: str = "UNKNOWN"
    relative_altitude_m: float = 0.0
    heading_deg: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    battery_percent: float = -1.0
    battery_voltage_v: float = 0.0
    gps_fix_type: int = 0
    satellites_visible: int = 0
    ekf_ok: bool = False
    in_air: bool = False
    last_heartbeat: float = 0.0
    last_msg_time: float = 0.0
    chan8_raw: int = 0           # 0 = no RC data
    chan8_state: str = "NO_SIGNAL"  # NO_SIGNAL until a real RC frame arrives
    rc_present: bool = False     # True once RC_CHANNELS reports chancount > 0
    mission_uses_rc: bool = False  # True if the running mission was started via CH8
    last_rc_time: float = 0.0
    fence_breached: bool = False # Set True when FENCE_STATUS reports a breach
    local_north_m: float = 0.0
    local_east_m: float = 0.0
    last_local_position_time: float = 0.0

    # --- Altitude ---------------------------------------------------------
    # relative_altitude_m is height above the *takeoff point* (AGL), derived from
    # ONE source with a ground baseline subtracted.  Previously three different
    # messages (GLOBAL_POSITION_INT, LOCAL_POSITION_NED, ALTITUDE) each wrote this
    # field directly; they disagree by metres, so the value oscillated and the
    # climb monitor could never confirm a takeoff.
    alt_raw_by_source: dict = field(default_factory=dict)
    alt_baseline_by_source: dict = field(default_factory=dict)
    alt_source: str = "NONE"
    baseline_frozen: bool = False
    baseline_valid: bool = False   # True once a real on-ground reference exists

    # --- Command feedback -------------------------------------------------
    last_command_ack: tuple | None = None
    prearm_messages: deque = field(default_factory=lambda: deque(maxlen=24))


# Altitude sources in priority order.  Only the highest-priority source that is
# actually arriving is used; lower-priority ones are cached but never override it.
ALT_SOURCE_PRIORITY = ("GLOBAL_POSITION_INT", "LOCAL_POSITION_NED")


def update_altitude(state: TelemetryState, source: str, raw_value: float):
    """
    Feeds one altitude reading into the state and recomputes AGL.

    ArduPilot's GLOBAL_POSITION_INT.relative_alt is relative to *home*, and
    LOCAL_POSITION_NED.z is relative to the *EKF origin*.  Both carry an offset of
    several metres once home/origin have been set at a different barometric
    reading, which is why a stationary aircraft on the ground was reporting -3.6 m
    on one message and +0.1 m on the next.  We therefore pick a single source and
    subtract a ground baseline, so relative_altitude_m is always height above the
    spot the aircraft is sitting on.
    """
    state.alt_raw_by_source[source] = raw_value

    # While disarmed on the ground, keep re-zeroing the baseline; it is frozen at
    # arming so the climb is measured against the takeoff point.  The aircraft
    # must be known-disarmed for this: if the daemon connects to an aircraft that
    # is already flying, zeroing here would make an airborne aircraft report 0 m
    # AGL and look like it was on the ground.
    if not state.baseline_frozen and state.last_heartbeat > 0 and not state.is_armed:
        state.alt_baseline_by_source[source] = raw_value
        state.baseline_valid = True

    for candidate in ALT_SOURCE_PRIORITY:
        if candidate in state.alt_raw_by_source:
            state.alt_source = candidate
            break

    primary = state.alt_source
    if primary in state.alt_raw_by_source:
        if state.baseline_valid:
            baseline = state.alt_baseline_by_source.get(primary, 0.0)
        else:
            # No ground reference was ever captured (daemon started mid-flight):
            # fall back to the raw height-above-home the autopilot reports.
            baseline = 0.0
        state.relative_altitude_m = state.alt_raw_by_source[primary] - baseline


def freeze_altitude_baseline(state: TelemetryState):
    """Latches the current ground reading as the zero point for the coming flight."""
    for source, raw in state.alt_raw_by_source.items():
        state.alt_baseline_by_source[source] = raw
    state.baseline_frozen = True
    logger.info(f"Altitude baseline latched at takeoff point "
                f"(source={state.alt_source}, raw={state.alt_raw_by_source.get(state.alt_source, 0.0):+.2f}m). "
                f"All altitudes below are AGL.")


def release_altitude_baseline(state: TelemetryState):
    """Lets the ground baseline track again once the aircraft is back on the ground."""
    state.baseline_frozen = False


def is_telemetry_fresh(state: TelemetryState) -> bool:
    """Returns True if telemetry data is recent enough to trust."""
    if state.last_heartbeat <= 0:
        return False
    return (time.monotonic() - state.last_heartbeat) < TELEMETRY_MAX_AGE_S


# ---------------------------------------------------------------------------
# Logging Setup
# ---------------------------------------------------------------------------
def setup_logging():
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = RotatingFileHandler(LOG_FILE, maxBytes=5_000_000, backupCount=5)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)


# ---------------------------------------------------------------------------
# Ground Detection
# ---------------------------------------------------------------------------
def is_drone_on_ground(state: TelemetryState) -> bool:
    """Determines whether the drone is currently on the ground."""
    if not state.is_armed and state.relative_altitude_m < 0.35:
        return True
    if not state.in_air and state.relative_altitude_m < 0.35:
        return True
    if state.relative_altitude_m < 0.20:
        return True
    return False


# ---------------------------------------------------------------------------
# Autonomous-flight safety gate
# ---------------------------------------------------------------------------
def is_safe_for_autonomous(state: TelemetryState) -> bool:
    """
    Returns True only if all conditions for continued autonomous flight are met:
    - CH8 is in GUIDED position
    - ArduPilot flight mode is GUIDED
    - Telemetry is fresh
    - EKF is healthy
    """
    if state.rc_present and state.chan8_state != "GUIDED":
        return False
    if state.flight_mode != "GUIDED":
        return False
    if not is_telemetry_fresh(state):
        return False
    return True


def check_autonomous_abort(state: TelemetryState, context: str) -> str | None:
    """
    Checks whether autonomous flight should be aborted.
    Returns a reason string if abort is needed, None if safe to continue.
    """
    # NO_SIGNAL is only an abort condition if we were relying on RC in the first
    # place.  A mission deliberately started without a receiver must not abort
    # itself on the very first telemetry frame.
    if state.chan8_state == "MANUAL":
        return f"[CH8 MANUAL] Pilot takeover detected during {context}."
    if state.chan8_state == "LAND":
        return f"[CH8 LAND] Land command detected during {context}."
    if state.chan8_state == "NO_SIGNAL" and state.mission_uses_rc:
        return f"[RC LOST] RC link lost during {context}."
    if state.fence_breached:
        return f"[GEOFENCE] Breach detected during {context}."
    if state.flight_mode not in ("GUIDED", "TAKEOFF"):
        return f"[MODE CHANGE] Flight mode changed to {state.flight_mode} during {context}."
    if not is_telemetry_fresh(state):
        return f"[TELEMETRY STALE] No fresh heartbeat for >{TELEMETRY_MAX_AGE_S:.1f}s during {context}."
    if not state.ekf_ok:
        return f"[EKF UNHEALTHY] Navigation health invalid during {context}."
    return None


# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------
def get_mode_name(master, heartbeat_msg) -> str:
    """Extracts human-readable flight mode name from heartbeat."""
    mode_mapping = master.mode_mapping() or {}
    reverse_mapping = {val: name for name, val in mode_mapping.items()}
    return reverse_mapping.get(heartbeat_msg.custom_mode, f"UNKNOWN({heartbeat_msg.custom_mode})")


def set_flight_mode(master, mode_name: str, state: TelemetryState, timeout_sec: float = 6.0) -> bool:
    """
    Commands ArduPilot to change flight mode and verifies the mode was actually entered.
    Uses the shared TelemetryState (updated by the telemetry listener) for confirmation
    rather than calling recv_match on the shared connection.
    """
    mode_mapping = master.mode_mapping() or {}
    if mode_name not in mode_mapping:
        logger.warning(f"Flight mode '{mode_name}' not recognized in mode mapping: {list(mode_mapping.keys())}")
        return False
    mode_id = mode_mapping[mode_name]
    logger.info(f"Setting flight mode to {mode_name} (ID: {mode_id})...")

    # Re-send periodically: a single set_mode can be lost on a busy serial link,
    # and ArduPilot silently ignores mode changes it cannot accept yet.
    start = time.monotonic()
    last_send = 0.0
    while time.monotonic() - start < timeout_sec:
        if time.monotonic() - last_send >= 1.0:
            master.set_mode(mode_id)
            last_send = time.monotonic()
        if state.flight_mode == mode_name:
            logger.info(f"Flight mode confirmed: {mode_name}")
            return True
        time.sleep(0.1)

    logger.warning(f"Flight mode change to {mode_name} not confirmed within {timeout_sec:.1f}s (current: {state.flight_mode})")
    return False


def request_message_interval(master, message_id: int, frequency_hz: float):
    """Requests ArduPilot to stream a specific MAVLink message at the requested frequency."""
    interval_us = int(1_000_000 / frequency_hz) if frequency_hz > 0 else -1
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )


def configure_ardupilot_message_streams(master):
    """Configures high-rate telemetry streams for precision control and monitoring."""
    logger.info("Requesting telemetry message streams from ArduPilot...")
    streams = [
        (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_FENCE_STATUS, 2),
    ]
    for msg_id, hz in streams:
        request_message_interval(master, msg_id, hz)


def configure_ardupilot_landing_parameters(master):
    """
    Configures ArduPilot parameters for a soft touchdown, battery failsafes and
    the geofence.

    FENCE_ACTION values for ArduPilot Copter (AC_FENCE):
      0 = Report Only
      1 = RTL or Land
      2 = Always Land  <-- Used here: safest for small obstacle areas
      3 = SmartRTL or RTL or Land
      4 = Brake or Land

    Two things this deliberately does NOT do any more:
      * It no longer forces ARMING_CHECK = 0.  Blanket-disabling the pre-arm
        checks is what let the aircraft arm with no RC link, no battery-voltage
        reading and an uncalibrated compass — it armed and then could not fly.
        The checks are restored to ArduPilot's default unless BENCH_MODE is set.
      * It no longer writes a 3 m fence radius with a 2 m fence margin.  The
        margin must stay below the radius or the aircraft counts as breaching
        the fence while it is still sitting on the ground.
    """
    params = [
        ("LAND_SPEED", 20.0),       # cm/s (default is 50 cm/s)
        ("LAND_ALT_LOW", 150.0),    # cm (slowdown begins at 1.5m)
        # Gentle climb: lowers the peak current draw (and so the voltage sag)
        # during the spool-up that breaks ground. See TAKEOFF_CLIMB_* above.
        ("WPNAV_SPEED_UP", max(10.0, TAKEOFF_CLIMB_SPEED_MPS * 100.0)),
        ("WPNAV_ACCEL_Z", max(50.0, TAKEOFF_CLIMB_ACCEL_MPS2 * 100.0)),
        ("BATT_FS_LOW_ACT", 1.0),   # 1: Land on low battery
        ("BATT_FS_CRT_ACT", 1.0),   # 1: Land on critical battery
        # RC loss lands in place instead of RTL. The default (1 = always RTL)
        # climbs to RTL_ALT — 15 m — which is absurd for a 1.2 m hover test and
        # is the opposite of handing control back to the pilot.
        ("FS_THR_ENABLE", 3.0),     # 3: Land on RC loss
        # Companion-link failsafe. If this daemon stops sending its 1 Hz
        # heartbeat (see gcs_heartbeat_loop) the aircraft lands rather than
        # holding in GUIDED forever. Safe to enable because FS_OPTIONS bit 16
        # ("continue in pilot-controlled modes on GCS failsafe") is set, so this
        # cannot fire while the pilot is flying manually on the transmitter.
        ("FS_GCS_ENABLE", 5.0),     # 5: Land when companion heartbeats stop
    ]

    if CONFIGURE_FENCE:
        # FENCE_MARGIN must be comfortably inside FENCE_RADIUS.
        fence_margin = max(1.0, min(2.0, FENCE_RADIUS_M / 3.0))
        params += [
            ("FENCE_ENABLE", 1.0),
            ("FENCE_TYPE", 3.0),            # 3: Max Alt & Circle
            ("FENCE_ALT_MAX", FENCE_ALT_MAX_M),
            ("FENCE_RADIUS", FENCE_RADIUS_M),
            ("FENCE_MARGIN", fence_margin),
            ("FENCE_ACTION", 2.0),          # 2: Always Land on breach
        ]
    else:
        # Explicitly turn the fence off rather than just skipping it, so a fence
        # stored on the board from an earlier run cannot block the arm.
        params.append(("FENCE_ENABLE", 0.0))
        logger.warning("Geofence DISABLED (CONFIGURE_FENCE=0) — writing FENCE_ENABLE=0. "
                       "There is no automatic containment: the pilot is responsible for "
                       "keeping the aircraft inside the test area, taking manual control "
                       "on the transmitter if it drifts.")

    if BENCH_MODE:
        logger.warning("=" * 70)
        logger.warning("BENCH_MODE ENABLED — disabling ArduPilot pre-arm checks (ARMING_CHECK=0).")
        logger.warning("The aircraft may arm with no RC, no battery monitor and no position.")
        logger.warning("DO NOT fly in this mode. Unset BENCH_MODE before any real flight.")
        logger.warning("=" * 70)
        params.append(("ARMING_CHECK", 0.0))
        params.append(("BRD_SAFETYENABLE", 0.0))
    else:
        # 1 = run all pre-arm checks (ArduPilot default).
        params.append(("ARMING_CHECK", 1.0))

    for param_name, param_val in params:
        try:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                param_name.encode("utf-8"),
                float(param_val),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            logger.info(f"Configured ArduPilot parameter: {param_name} = {param_val}")
            time.sleep(0.05)
        except Exception as e:
            logger.debug(f"Could not set parameter {param_name}: {e}")


def run_prearm_checks(master):
    """Asks ArduPilot to re-run its pre-arm checks so failures are reported now."""
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
    except Exception as e:
        logger.debug(f"Could not request pre-arm checks: {e}")


def describe_ack_result(result: int) -> str:
    """Human-readable MAV_RESULT name."""
    entry = mavutil.mavlink.enums["MAV_RESULT"].get(result)
    return entry.name if entry else f"MAV_RESULT({result})"


def send_body_velocity(master, vx: float, vy: float, vz: float, yaw_rate_dps: float = 0.0):
    """
    Sends a body-frame velocity and yaw rate setpoint to ArduPilot in GUIDED mode.
    vx: forward velocity (+ forward, - backward) in m/s
    vy: lateral velocity (+ right, - left) in m/s
    vz: vertical velocity (+ down, - up) in m/s
    yaw_rate_dps: yaw rotation rate in deg/s (+ clockwise)
    """
    yaw_rate_rad = math.radians(yaw_rate_dps)
    master.mav.set_position_target_local_ned_send(
        0,                                                  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,          # Body-fixed forward/right/down frame
        SETPOINT_VELOCITY_YAW_RATE_MASK,                    # Typemask (velocities + yaw rate)
        0.0, 0.0, 0.0,                                      # Positions (ignored)
        float(vx), float(vy), float(vz),                    # Velocities (m/s)
        0.0, 0.0, 0.0,                                      # Accelerations (ignored)
        0.0,                                                # Yaw (ignored)
        float(yaw_rate_rad)                                 # Yaw rate (rad/s)
    )



def send_local_ned_velocity(master, vx: float, vy: float, vz: float = 0.0):
    """Sends a local-NED velocity setpoint while leaving altitude control unchanged."""
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        SETPOINT_VELOCITY_YAW_RATE_MASK,
        0.0, 0.0, 0.0,
        float(vx), float(vy), float(vz),
        0.0, 0.0, 0.0,
        0.0,
        0.0,
    )
def arm_motors(master, state: TelemetryState, timeout_sec: float = 10.0) -> bool:
    """
    Sends the arm command and waits for confirmation via the shared TelemetryState.

    On failure it reports *why*: ArduPilot answers a rejected arm request with a
    COMMAND_ACK result and emits "PreArm: ..." status texts.  The previous version
    logged a bare "Arming confirmation timed out", which hid the real causes
    (no RC receiver, battery below arming voltage, compass, fence breach).
    """
    state.prearm_messages.clear()
    state.last_command_ack = None

    logger.info("Sending arm command to ArduPilot motors...")
    master.arducopter_arm()

    start_time = time.monotonic()
    rejected = None
    while time.monotonic() - start_time < timeout_sec:
        if state.is_armed:
            logger.info("ArduPilot confirmed ARMED.")
            return True
        ack = state.last_command_ack
        if ack and ack[0] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack[1] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                # Accepted; keep waiting for the heartbeat to show the armed flag.
                pass
            elif ack[1] != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                rejected = ack[1]
                break
        time.sleep(0.1)

    if rejected is not None:
        logger.error(f"Arm command REJECTED by ArduPilot: {describe_ack_result(rejected)}")
    else:
        logger.error(f"Arming not confirmed within {timeout_sec:.0f}s.")

    # Give ArduPilot a moment to publish the pre-arm reasons.
    run_prearm_checks(master)
    time.sleep(1.5)

    reasons = []
    for text in state.prearm_messages:
        if text not in reasons:
            reasons.append(text)
    if reasons:
        logger.error("ArduPilot refused to arm for the following reason(s):")
        for text in reasons:
            logger.error(f"    - {text}")
    else:
        logger.error("ArduPilot reported no pre-arm text. Check the flight controller "
                     "log / GCS for the arming failure reason.")

    return state.is_armed


def disarm_motors(master, state: TelemetryState | None = None,
                  force: bool = False, timeout_sec: float = 5.0) -> bool:
    """
    Sends the disarm command and confirms it took effect.

    pymavlink's mavutil.arducopter_disarm() takes no arguments in this version,
    so the previous `arducopter_disarm(force=force)` call raised TypeError on
    every disarm path.  Sending MAV_CMD_COMPONENT_ARM_DISARM directly also lets
    us pass ArduPilot's force-disarm magic value when it is genuinely needed.
    """
    logger.info(f"Sending {'FORCE ' if force else ''}disarm command to ArduPilot...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,                      # param1: 0 = disarm
        21196 if force else 0,  # param2: ArduPilot force-disarm magic number
        0, 0, 0, 0, 0,
    )

    if state is None:
        return True

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if not state.is_armed:
            logger.info("ArduPilot confirmed DISARMED.")
            return True
        time.sleep(0.1)

    logger.warning(f"Disarm not confirmed within {timeout_sec:.0f}s (still armed).")
    return False


def send_takeoff_command(master, state: TelemetryState, altitude_m: float,
                        ack_timeout_s: float = 3.0) -> bool:
    """
    Sends MAV_CMD_NAV_TAKEOFF and verifies ArduPilot accepted it.

    Without this check a rejected takeoff looked identical to a takeoff that was
    accepted but produced no thrust — both simply failed to climb.
    """
    logger.info(f"Sending MAV_CMD_NAV_TAKEOFF for target altitude {altitude_m:.1f}m...")
    state.last_command_ack = None
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        float(altitude_m)
    )

    deadline = time.monotonic() + ack_timeout_s
    while time.monotonic() < deadline:
        ack = state.last_command_ack
        if ack and ack[0] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if ack[1] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("Takeoff command ACCEPTED by ArduPilot.")
                return True
            logger.error(f"Takeoff command REJECTED by ArduPilot: {describe_ack_result(ack[1])}")
            for text in list(state.prearm_messages)[-5:]:
                logger.error(f"    - {text}")
            return False
        time.sleep(0.05)

    logger.warning(f"No COMMAND_ACK for takeoff within {ack_timeout_s:.1f}s — "
                   f"proceeding to monitor the climb anyway.")
    return True


# ---------------------------------------------------------------------------
# Telemetry Listener (single MAVLink receive path)
# ---------------------------------------------------------------------------
def telemetry_listener_loop(master, state: TelemetryState, stop_event: threading.Event):
    """
    Background telemetry thread that continuously parses MAVLink messages
    and maintains the live TelemetryState cache.  This is the ONLY thread
    that calls recv_match on the MAVLink connection.
    """
    last_ch8_state = None

    while not stop_event.is_set():
        try:
            msg = master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue

            now = time.monotonic()
            state.last_msg_time = now
            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                # Only the autopilot's own heartbeat describes the aircraft. Other
                # components on the link (gimbals, telemetry radios, a GCS) also
                # emit HEARTBEAT, and taking mode/armed state from those flips
                # flight_mode to UNKNOWN and armed to False at random.
                if msg.get_srcSystem() != master.target_system:
                    continue
                if msg.get_srcComponent() not in (0, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                    continue

                state.is_connected = True
                state.last_heartbeat = now
                state.flight_mode = get_mode_name(master, msg)
                was_armed = state.is_armed
                state.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                # Freeze the altitude zero-point at arming, release it on disarm.
                if state.is_armed and not was_armed:
                    if state.baseline_valid:
                        freeze_altitude_baseline(state)
                    else:
                        logger.warning("Armed before a ground altitude reference was captured "
                                       "(daemon may have connected mid-flight). "
                                       "Reporting height above home instead of AGL.")
                        state.baseline_frozen = True
                elif was_armed and not state.is_armed:
                    release_altitude_baseline(state)

                # Determine in-air status
                if state.is_armed and (state.relative_altitude_m > 0.35 or state.in_air):
                    state.in_air = True
                elif not state.is_armed and state.relative_altitude_m < 0.25:
                    state.in_air = False

            elif msg_type == "GLOBAL_POSITION_INT":
                update_altitude(state, "GLOBAL_POSITION_INT", msg.relative_alt / 1000.0)
                state.heading_deg = msg.hdg / 100.0
                state.vx = msg.vx / 100.0
                state.vy = msg.vy / 100.0
                state.vz = msg.vz / 100.0

            elif msg_type == "LOCAL_POSITION_NED":
                # ArduPilot sends NaN here whenever the EKF has no horizontal
                # solution (CONST_POS_MODE, seen on the bench as ned=(nan,nan,nan)).
                # A NaN is not a position, so it must not refresh the timestamp:
                # dropping the frame lets local_position_is_fresh() go stale on its
                # own, which every caller already handles. Storing it instead would
                # poison the containment maths, where `nan < tolerance` is False and
                # `outer.contains(nan, nan)` reads as a geofence breach.
                if math.isfinite(msg.x) and math.isfinite(msg.y):
                    state.local_north_m = msg.x
                    state.local_east_m = msg.y
                    state.last_local_position_time = now
                if math.isfinite(msg.z):
                    # In NED, z is negative upwards. Fallback source only.
                    update_altitude(state, "LOCAL_POSITION_NED", -msg.z)

            elif msg_type == "SYS_STATUS":
                state.battery_voltage_v = msg.voltage_battery / 1000.0
                if msg.battery_remaining != -1:
                    state.battery_percent = float(msg.battery_remaining)

            elif msg_type == "BATTERY_STATUS":
                if msg.battery_remaining != -1:
                    state.battery_percent = float(msg.battery_remaining)
                if len(msg.voltages) > 0 and msg.voltages[0] != 65535:
                    state.battery_voltage_v = msg.voltages[0] / 1000.0

            elif msg_type == "GPS_RAW_INT":
                state.gps_fix_type = msg.fix_type
                state.satellites_visible = msg.satellites_visible

            elif msg_type == "EKF_STATUS_REPORT":
                # EKF is healthy when flags indicate velocity and position variance are good
                state.ekf_ok = bool(msg.flags & (1 << 4))

            elif msg_type == "RC_CHANNELS":
                # chancount == 0 means the flight controller has no receiver data
                # at all; the channel values are then meaningless zeros.
                rc_present = msg.chancount > 0 and msg.chan8_raw > 0
                state.rc_present = rc_present
                state.chan8_raw = msg.chan8_raw
                if rc_present:
                    state.last_rc_time = now
                new_ch8_state = classify_ch8(msg.chan8_raw, rc_present)
                if new_ch8_state != last_ch8_state:
                    if new_ch8_state == "NO_SIGNAL":
                        logger.warning(f"[CH8] No RC receiver data (chancount={msg.chancount}, "
                                       f"raw={msg.chan8_raw}). CH8 triggering is unavailable.")
                    else:
                        logger.info(f"[CH8] Switch state changed: {last_ch8_state} -> "
                                    f"{new_ch8_state} (raw={msg.chan8_raw})")
                    last_ch8_state = new_ch8_state
                state.chan8_state = new_ch8_state

            elif msg_type == "FENCE_STATUS":
                # FENCE_STATUS.breach_status: 0 = no breach, non-zero = breach active
                was_breached = state.fence_breached
                state.fence_breached = (msg.breach_status != 0)
                if state.fence_breached and not was_breached:
                    logger.warning(f"[GEOFENCE] Breach detected! breach_status={msg.breach_status}, "
                                   f"breach_count={msg.breach_count}")
                elif not state.fence_breached and was_breached:
                    logger.info("[GEOFENCE] Breach cleared.")

            elif msg_type == "COMMAND_ACK":
                state.last_command_ack = (msg.command, msg.result, now)

            elif msg_type == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else msg.text.decode('utf-8', errors='ignore')
                text = text.strip()
                logger.info(f"[AP STATUSTEXT] {text}")
                lowered = text.lower()
                if lowered.startswith("prearm") or lowered.startswith("arm:") or "arming" in lowered:
                    state.prearm_messages.append(text)

        except Exception as e:
            logger.debug(f"Telemetry listener error: {e}")
            time.sleep(0.05)


# ---------------------------------------------------------------------------
# Telemetry Logging
# ---------------------------------------------------------------------------
def log_telemetry_health(state: TelemetryState):
    """Logs current telemetry, GPS, and battery status."""
    gps_status = f"Fix={state.gps_fix_type} (Sats: {state.satellites_visible})" if state.gps_fix_type >= 3 else "NO FIX"
    rc_status = f"CH8={state.chan8_raw} ({state.chan8_state})" if state.rc_present else "RC=NONE"
    logger.info(
        f"ArduPilot Health: Mode={state.flight_mode}, Armed={state.is_armed}, "
        f"AGL={state.relative_altitude_m:.2f}m (src={state.alt_source}), GPS={gps_status}, "
        f"EKF={'OK' if state.ekf_ok else 'FAIL'}, "
        f"Batt={state.battery_percent:.0f}% ({state.battery_voltage_v:.2f}V), "
        f"{rc_status}"
    )


# ---------------------------------------------------------------------------
# Takeoff climb monitor
# ---------------------------------------------------------------------------
def wait_for_climb(master, state: TelemetryState, target_altitude_m: float, timeout_sec: float = 30.0) -> bool:
    """
    Monitors the climb to the target altitude.

    Altitudes here are AGL relative to the takeoff point (see update_altitude),
    so a home/EKF-origin offset can no longer make a climbing aircraft look like
    it is several metres underground.

    Requires altitude within ±ALTITUDE_ACCEPTANCE_BAND_M of target, stable for
    ALTITUDE_STABLE_COUNT consecutive readings.  Fails early and loudly if the
    aircraft never leaves the ground at all, which is a thrust/power problem
    rather than a climb that simply needs more time.
    """
    logger.info(f"Monitoring takeoff climb to target {target_altitude_m:.1f}m AGL "
                f"(acceptance band: ±{ALTITUDE_ACCEPTANCE_BAND_M:.2f}m)...")
    lower_bound = target_altitude_m - ALTITUDE_ACCEPTANCE_BAND_M
    start_time = time.monotonic()
    last_log = 0.0
    stable_count = 0
    lifted_off = False
    max_alt_seen = state.relative_altitude_m

    while time.monotonic() - start_time < timeout_sec:
        # Safety checks
        abort_reason = check_autonomous_abort(state, "climb")
        if abort_reason:
            logger.warning(abort_reason)
            if state.chan8_state == "LAND" or state.fence_breached:
                set_flight_mode(master, "LAND", state)
            return False

        current_alt = state.relative_altitude_m
        max_alt_seen = max(max_alt_seen, current_alt)

        if not lifted_off and current_alt >= LIFTOFF_ALT_M:
            lifted_off = True
            logger.info(f"Liftoff detected at {current_alt:.2f}m AGL.")

        # The aircraft is armed but still sitting on the ground: no point waiting
        # out the full climb timeout, and the reason is worth reporting.
        if not lifted_off and (time.monotonic() - start_time) > NO_CLIMB_TIMEOUT_S:
            logger.error(
                f"NO LIFTOFF: still at {current_alt:.2f}m AGL "
                f"{NO_CLIMB_TIMEOUT_S:.0f}s after the takeoff command "
                f"(max seen {max_alt_seen:.2f}m)."
            )
            logger.error("The takeoff command was accepted but the aircraft produced no climb.")
            logger.error(f"  Battery voltage reported by the flight controller: {state.battery_voltage_v:.2f}V")
            if state.battery_voltage_v < 1.0:
                logger.error("  -> No flight battery detected. The motors have no power source; "
                             "USB power runs the flight controller only.")
            for text in list(state.prearm_messages)[-5:]:
                logger.error(f"  - {text}")
            return False

        if current_alt >= lower_bound:
            stable_count += 1
            if stable_count >= ALTITUDE_STABLE_COUNT:
                logger.info(f"Target altitude reached and stable: {current_alt:.2f}m AGL "
                            f"(target: {target_altitude_m:.1f}m)")
                return True
        else:
            stable_count = 0

        if time.monotonic() - last_log >= 2.0:
            logger.info(f"Climbing... {current_alt:.2f}m AGL / target {target_altitude_m:.1f}m "
                        f"(mode={state.flight_mode}, armed={state.is_armed})")
            last_log = time.monotonic()

        time.sleep(0.2)

    logger.warning(f"Takeoff climb confirmation timed out after {timeout_sec:.1f}s "
                   f"(alt: {state.relative_altitude_m:.2f}m AGL, max seen {max_alt_seen:.2f}m).")
    return False


# ---------------------------------------------------------------------------
# Velocity streaming with safety checks
# ---------------------------------------------------------------------------
def send_body_velocity_with_override_check(
    master,
    state: TelemetryState,
    vx: float,
    vy: float,
    vz: float,
    yaw_rate_dps: float,
    duration_s: float,
) -> bool:
    """
    Streams body-frame velocity setpoints to ArduPilot GUIDED mode at STREAM_HZ.
    Continuously monitors CH8, flight mode, telemetry freshness, and EKF.
    Returns True if completed successfully, False if interrupted.
    """
    interval = 1.0 / STREAM_HZ
    end_time = time.monotonic() + duration_s

    while time.monotonic() < end_time:
        # Safety gate
        abort_reason = check_autonomous_abort(state, "velocity streaming")
        if abort_reason:
            logger.warning(abort_reason)
            if state.chan8_state == "LAND" or state.fence_breached:
                set_flight_mode(master, "LAND", state)
            return False

        try:
            send_body_velocity(master, vx, vy, vz, yaw_rate_dps)
        except Exception as e:
            logger.error(f"Error sending body velocity setpoint: {e}")
            return False

        time.sleep(interval)

    return True


# ---------------------------------------------------------------------------
# Local-NED horizontal geofence orbit
# ---------------------------------------------------------------------------
def local_position_is_fresh(state: TelemetryState) -> bool:
    # time.monotonic(), never time.time().  On 2026-09-15 this check used the
    # wall clock, systemd-timesyncd stepped it forward 82s mid-hover on its first
    # NTP contact, and every stored stamp instantly looked 82s old.  The raster
    # leg refused to start on "stale" telemetry that was in fact 100ms old by the
    # flight controller's own boot clock, and the aircraft safety-landed.
    # The Pi 5 has no battery-backed RTC, so that step happens on every boot.
    #
    # The isfinite pair is belt and braces: the telemetry listener already drops
    # NaN frames rather than storing them, so a stored value should always be
    # real. Every position-dependent check in both modules funnels through this
    # one gate, which makes it the cheapest place to be certain.
    return (
        state.last_local_position_time > 0
        and math.isfinite(state.local_north_m)
        and math.isfinite(state.local_east_m)
        and time.monotonic() - state.last_local_position_time <= LOCAL_POSITION_MAX_AGE_S
    )


def horizontal_distance_from_origin(state: TelemetryState, x0: float, y0: float) -> float:
    return math.hypot(state.local_north_m - x0, state.local_east_m - y0)


def run_horizontal_geofence_breach_land(master, state: TelemetryState, distance_m: float):
    """Stops the companion orbit and uses the existing in-air safety landing."""
    logger.warning(
        f"[HORIZONTAL GEOFENCE] Distance {distance_m:.2f}m from fixed origin; "
        f"abort threshold is {HORIZONTAL_GEOFENCE_RADIUS_M - HORIZONTAL_GEOFENCE_ABORT_MARGIN_M:.2f}m."
    )
    try:
        send_local_ned_velocity(master, 0.0, 0.0, 0.0)
    except Exception as e:
        logger.error(f"Could not send horizontal stop setpoint: {e}")
    run_in_air_safety_land(master, state)


def horizontal_orbit_safety_check(master, state: TelemetryState, x0: float, y0: float, context: str) -> bool:
    abort_reason = check_autonomous_abort(state, context)
    if abort_reason:
        logger.warning(abort_reason)
        if state.chan8_state == "LAND" or state.fence_breached:
            set_flight_mode(master, "LAND", state)
        return False
    if not local_position_is_fresh(state):
        logger.warning(f"[LOCAL POSITION STALE] No fresh LOCAL_POSITION_NED during {context}.")
        return False
    distance_m = horizontal_distance_from_origin(state, x0, y0)
    if distance_m >= HORIZONTAL_GEOFENCE_RADIUS_M - HORIZONTAL_GEOFENCE_ABORT_MARGIN_M:
        run_horizontal_geofence_breach_land(master, state, distance_m)
        return False
    return True


def move_to_local_ned_point(master, state: TelemetryState, x0: float, y0: float,
                            target_x: float, target_y: float, context: str) -> bool:
    """Conservatively tracks one local-NED point without commanding vertical motion."""
    interval = 1.0 / STREAM_HZ
    timeout_s = max(15.0, 3.0 * math.hypot(target_x - x0, target_y - y0) / ORBIT_SPEED_MPS)
    end_time = time.monotonic() + timeout_s

    while time.monotonic() < end_time:
        if not horizontal_orbit_safety_check(master, state, x0, y0, context):
            return False
        error_x = target_x - state.local_north_m
        error_y = target_y - state.local_east_m
        distance_m = math.hypot(error_x, error_y)
        if distance_m <= LOCAL_POSITION_TOLERANCE_M:
            send_local_ned_velocity(master, 0.0, 0.0, 0.0)
            return True
        speed_mps = min(ORBIT_SPEED_MPS, max(0.05, 0.8 * distance_m))
        send_local_ned_velocity(master, speed_mps * error_x / distance_m,
                                speed_mps * error_y / distance_m, 0.0)
        time.sleep(interval)

    logger.warning(f"Timed out while {context}.")
    return False


def fly_horizontal_geofence_orbit(master, state: TelemetryState) -> bool:
    """Flies one origin-fixed local-NED orbit and returns to that same origin."""
    if not local_position_is_fresh(state):
        logger.warning("Cannot capture horizontal orbit origin: LOCAL_POSITION_NED is unavailable or stale.")
        return False

    # This origin is captured once after takeoff stabilization and is never changed.
    x0 = state.local_north_m
    y0 = state.local_east_m
    logger.info(f"[HORIZONTAL GEOFENCE] Fixed origin captured: N={x0:.2f}m, E={y0:.2f}m.")

    first_x = x0 + ORBIT_RADIUS_M
    first_y = y0
    logger.info(f"Moving from origin to the {ORBIT_RADIUS_M:.1f}m orbit radius.")
    if not move_to_local_ned_point(master, state, x0, y0, first_x, first_y, "moving to orbit radius"):
        return False

    orbit_duration_s = (2.0 * math.pi * ORBIT_RADIUS_M * ORBIT_LOOPS) / ORBIT_SPEED_MPS
    logger.info(f"Flying {ORBIT_LOOPS:.1f} local-NED orbit(s) at radius {ORBIT_RADIUS_M:.1f}m "
                f"around fixed origin for {orbit_duration_s:.1f}s.")
    interval = 1.0 / STREAM_HZ
    orbit_start = time.monotonic()
    while True:
        elapsed_s = time.monotonic() - orbit_start
        if elapsed_s >= orbit_duration_s:
            break
        if not horizontal_orbit_safety_check(master, state, x0, y0, "horizontal orbit"):
            return False
        theta = 2.0 * math.pi * ORBIT_LOOPS * elapsed_s / orbit_duration_s
        # The commanded circle is always centered on the immutable (x0, y0).
        target_x = x0 + ORBIT_RADIUS_M * math.cos(theta)
        target_y = y0 + ORBIT_RADIUS_M * math.sin(theta)
        error_x = target_x - state.local_north_m
        error_y = target_y - state.local_east_m
        error_m = math.hypot(error_x, error_y)
        if error_m > 0.0:
            speed_mps = min(ORBIT_SPEED_MPS, max(0.05, 0.8 * error_m))
            send_local_ned_velocity(master, speed_mps * error_x / error_m,
                                    speed_mps * error_y / error_m, 0.0)
        else:
            send_local_ned_velocity(master, 0.0, 0.0, 0.0)
        time.sleep(interval)

    # Complete the circle at theta=2*pi*ORBIT_LOOPS, then return to the fixed origin.
    if not move_to_local_ned_point(master, state, x0, y0, first_x, first_y, "completing horizontal orbit"):
        return False
    logger.info("Returning to the fixed horizontal origin.")
    return move_to_local_ned_point(master, state, x0, y0, x0, y0, "returning to horizontal origin")

# ---------------------------------------------------------------------------
# Controlled Soft Landing
# ---------------------------------------------------------------------------
def execute_controlled_soft_landing(master, state: TelemetryState, max_timeout_s: float = 60.0) -> bool:
    """
    Executes an ultra-gentle, multi-phase controlled landing tailored for hard
    concrete surfaces / fragile landing gear:

    Phase 1: Zero-velocity hover (2.0s) to eliminate horizontal drift.
    Phase 2: Ultra-slow GUIDED descent at DESCENT_SPEED_MPS (0.15 m/s).
    Phase 3: Touchdown cushion crawl at TOUCHDOWN_SPEED_MPS (0.08 m/s)
             when altitude <= CONCRETE_SLOWDOWN_ALT_M.
    Phase 4: Touchdown detection, switch to LAND mode for ground latching
             and disarm confirmation.

    Returns True only if touchdown AND disarm are confirmed.
    Returns False if interrupted by pilot takeover, telemetry loss, or timeout
    without confirmed touchdown.
    """
    logger.info("=========================================================")
    logger.info("=== STARTING ULTRA-SLOW CONTROLLED CONCRETE LANDING ===")
    logger.info("=========================================================")
    interval = 1.0 / STREAM_HZ
    start_time = time.monotonic()
    last_log_time = 0.0

    # Phase 1: Stabilize hover before descent
    logger.info("Phase 1: Stabilizing hover (2.0s) to eliminate horizontal drift...")
    for _ in range(int(2.0 * STREAM_HZ)):
        abort_reason = check_autonomous_abort(state, "landing hover")
        if abort_reason:
            logger.warning(abort_reason)
            if state.chan8_state == "LAND" or state.fence_breached:
                set_flight_mode(master, "LAND", state)
            return False
        send_body_velocity(master, 0.0, 0.0, 0.0, 0.0)
        time.sleep(interval)

    # Phase 2 & 3: Controlled slow descent down to touchdown
    logger.info(
        f"Phase 2 & 3: Beginning ultra-slow descent ({DESCENT_SPEED_MPS:.2f} m/s) "
        f"with cushion crawl at {CONCRETE_SLOWDOWN_ALT_M:.2f}m ({TOUCHDOWN_SPEED_MPS:.2f} m/s)..."
    )

    ground_counter = 0

    while time.monotonic() - start_time < max_timeout_s:
        # Check CH8 override
        abort_reason = check_autonomous_abort(state, "landing descent")
        if abort_reason:
            logger.warning(abort_reason)
            if state.chan8_state == "LAND" or state.fence_breached:
                set_flight_mode(master, "LAND", state)
            return False

        # Determine descent speed based on current altitude
        current_alt = state.relative_altitude_m
        if current_alt <= CONCRETE_SLOWDOWN_ALT_M:
            current_vz = TOUCHDOWN_SPEED_MPS
            phase_str = "Cushion Crawl (Concrete Touchdown)"
        else:
            current_vz = DESCENT_SPEED_MPS
            phase_str = "Slow Descent"

        # Check if drone reached the ground
        if is_drone_on_ground(state) or not state.in_air or (current_alt <= 0.12 and current_alt >= -0.5):
            ground_counter += 1
            if ground_counter >= 5:  # Confirmed on ground for 0.5s
                logger.info("Touchdown detected on concrete surface!")
                break
        else:
            ground_counter = 0

        # Send downward velocity setpoint (+vz is down in NED)
        send_body_velocity(master, 0.0, 0.0, current_vz, 0.0)

        if time.monotonic() - last_log_time >= 2.0:
            logger.info(
                f"[LANDING] {phase_str}: Alt={current_alt:.2f}m, "
                f"DescentRate={current_vz*100:.1f} cm/s"
            )
            last_log_time = time.monotonic()

        time.sleep(interval)

    # Phase 4: Command ArduPilot LAND mode for ground latching and motor shutdown
    logger.info("Phase 4: Switching to LAND mode for final touchdown & disarm...")
    set_flight_mode(master, "LAND", state)

    # Wait for motors to disarm — require actual confirmation
    logger.info("Waiting for touchdown confirmation and motor disarm...")
    disarm_start = time.monotonic()
    last_land_log = 0.0
    landing_confirmed = False
    while time.monotonic() - disarm_start < 20.0:
        if not state.is_armed and is_drone_on_ground(state) and not state.in_air:
            logger.info("Touchdown confirmed on concrete and aircraft disarmed.")
            landing_confirmed = True
            break
        if time.monotonic() - last_land_log >= 3.0:
            logger.info(f"Landing confirmation in progress: Alt={state.relative_altitude_m:.2f}m, Armed={state.is_armed}")
            last_land_log = time.monotonic()
        time.sleep(0.5)

    # Only disarm if confirmed on ground — never disarm while airborne
    if state.is_armed and is_drone_on_ground(state) and not state.in_air:
        logger.info("Aircraft is on ground; sending disarm command to prevent prop wash on concrete...")
        disarm_motors(master, state)
        time.sleep(1.0)
        if not state.is_armed:
            landing_confirmed = True

    if not landing_confirmed:
        logger.error("Landing NOT confirmed — aircraft may still be armed or airborne! "
                     "NOT switching to LOITER. Manual intervention may be needed.")
        return False

    # Only switch to LOITER if landing is fully confirmed
    set_flight_mode(master, "LOITER", state)
    return True


# ---------------------------------------------------------------------------
# In-Air Safety Land
# ---------------------------------------------------------------------------
def run_in_air_safety_land(master, state: TelemetryState):
    """
    Triggered when autonomous control is engaged while drone is ALREADY IN THE AIR,
    or when CH8 is switched to LAND position.
    Executes immediate safety LAND per concrete landing safety requirement.
    """
    logger.warning("==========================================================")
    logger.warning("=== EXECUTING SAFETY LAND ===")
    logger.warning("==========================================================")

    # Ensure video is recording
    if not video_recorder.is_recording():
        try:
            video_recorder.start_recording()
        except Exception as e:
            logger.warning(f"Could not start video: {e}")

    # Command LAND mode
    logger.info("Issuing ArduPilot LAND command...")
    set_flight_mode(master, "LAND", state)

    # Monitor descent until landed on ground and disarmed
    start_time = time.monotonic()
    last_log = 0.0
    landing_confirmed = False
    while time.monotonic() - start_time < 60.0:
        if not state.is_armed and is_drone_on_ground(state) and not state.in_air:
            logger.info("Touchdown confirmed on ground and aircraft disarmed.")
            landing_confirmed = True
            break
        if time.monotonic() - last_log >= 3.0:
            logger.info(
                f"Landing descent in progress: altitude={state.relative_altitude_m:.2f}m, "
                f"mode={state.flight_mode}, armed={state.is_armed}"
            )
            last_log = time.monotonic()
        time.sleep(0.5)

    # Only disarm if confirmed on ground
    if state.is_armed and is_drone_on_ground(state) and not state.in_air:
        disarm_motors(master, state)
        time.sleep(1.0)
        if not state.is_armed:
            landing_confirmed = True

    if landing_confirmed:
        set_flight_mode(master, "LOITER", state)
    else:
        logger.error("Safety landing NOT fully confirmed — NOT switching to LOITER. "
                     "Manual intervention may be needed.")

    # Finalize video
    try:
        saved_file = video_recorder.stop_recording()
        if saved_file:
            logger.info(f"Safety landing flight video saved at: {saved_file}")
    except Exception as e:
        logger.error(f"Error finalizing video: {e}")

    logger.info("Safety landing completed. Returning to STANDBY mode.")


# ---------------------------------------------------------------------------
# Geofence Breach Landing
# ---------------------------------------------------------------------------
def run_geofence_breach_land(master, state: TelemetryState):
    """
    Called when FENCE_STATUS reports a geofence breach.
    Immediately stops all companion autonomous horizontal velocity commands,
    commands ArduPilot LAND, and monitors until confirmed touchdown + disarm.

    - Does NOT attempt RTL or any horizontal movement.
    - Does NOT disarm while airborne.
    - Does NOT switch to LOITER unless touchdown AND disarm are confirmed.
    - Respects CH8=MANUAL (pilot takeover) — stops companion commands and yields.
    """
    logger.warning("==========================================================")
    logger.warning("=== [GEOFENCE] BREACH — EXECUTING SAFE LAND ===")
    logger.warning("==========================================================")
    logger.warning("[GEOFENCE] Autonomous motion stopped.")

    # Ensure video is recording
    if not video_recorder.is_recording():
        try:
            video_recorder.start_recording()
        except Exception as e:
            logger.warning(f"Could not start video: {e}")

    # Command LAND — no horizontal movement, just descend in place
    logger.warning("[GEOFENCE] Commanding soft LAND.")
    set_flight_mode(master, "LAND", state)

    # Monitor descent until landed on ground and disarmed
    start_time = time.monotonic()
    last_log = 0.0
    landing_confirmed = False
    while time.monotonic() - start_time < 60.0:
        # CH8 = MANUAL means pilot wants control — stop companion intervention
        if state.chan8_state == "MANUAL":
            logger.info("[GEOFENCE] CH8=MANUAL during breach landing. Yielding to pilot.")
            # Do NOT continue monitoring — pilot owns the aircraft
            break

        if not state.is_armed and is_drone_on_ground(state) and not state.in_air:
            logger.info("[GEOFENCE] Touchdown confirmed.")
            logger.info("[GEOFENCE] Disarm confirmed.")
            landing_confirmed = True
            break

        if time.monotonic() - last_log >= 3.0:
            logger.info(
                f"[GEOFENCE] Landing in progress: altitude={state.relative_altitude_m:.2f}m, "
                f"mode={state.flight_mode}, armed={state.is_armed}, CH8={state.chan8_state}"
            )
            last_log = time.monotonic()
        time.sleep(0.5)

    # Only disarm if confirmed on ground — never disarm while airborne
    if state.is_armed and is_drone_on_ground(state) and not state.in_air:
        logger.info("[GEOFENCE] Aircraft on ground; sending disarm command...")
        disarm_motors(master, state)
        time.sleep(1.0)
        if not state.is_armed:
            logger.info("[GEOFENCE] Disarm confirmed.")
            landing_confirmed = True

    if landing_confirmed:
        set_flight_mode(master, "LOITER", state)
        logger.info("[GEOFENCE] Breach landing completed successfully. Returning to STANDBY.")
    else:
        logger.error("[GEOFENCE] Landing NOT fully confirmed — NOT switching to LOITER. "
                     "Aircraft remains in safest available landing state. Manual intervention may be needed.")

    # Finalize video
    try:
        saved_file = video_recorder.stop_recording()
        if saved_file:
            logger.info(f"[GEOFENCE] Flight video saved at: {saved_file}")
    except Exception as e:
        logger.error(f"Error finalizing video: {e}")

    # Clear breach flag only after confirmed landing (or pilot takeover)
    # The flag will be naturally cleared by FENCE_STATUS telemetry when no longer breached
    logger.info("[GEOFENCE] Breach response complete.")


# ---------------------------------------------------------------------------
# Manual Flight Monitor
# ---------------------------------------------------------------------------
def handle_manual_flight(master, state: TelemetryState):
    """
    Monitors pilot manual flight.
    - Records video in background without interfering with manual stick control.
    - Does NOT send any velocity commands — pilot has full control.
    - When pilot lands and disarms, finalizes video and returns to STANDBY.
    """
    logger.info(f"[MANUAL FLIGHT] Monitoring pilot manual flight (Mode: {state.flight_mode})...")
    if not video_recorder.is_recording():
        try:
            video_recorder.start_recording()
        except Exception as e:
            logger.warning(f"Could not start video: {e}")

    last_log = time.monotonic()
    while state.is_armed or state.in_air:
        # If pilot switches CH8 to LAND while in the air -> Safety Land
        if state.chan8_state == "LAND" and not is_drone_on_ground(state):
            logger.warning("[MANUAL FLIGHT] CH8 switched to LAND in air -> Executing Safety LAND!")
            run_in_air_safety_land(master, state)
            return

        if time.monotonic() - last_log >= 5.0:
            logger.info(
                f"[MANUAL FLIGHT] Flying: Mode={state.flight_mode}, "
                f"Alt={state.relative_altitude_m:.2f}m, Batt={state.battery_percent:.0f}%, "
                f"CH8={state.chan8_state}"
            )
            last_log = time.monotonic()

        time.sleep(0.5)

    logger.info("[MANUAL FLIGHT] Aircraft landed and disarmed. Finalizing video...")

    set_flight_mode(master, "LOITER", state)

    try:
        saved_file = video_recorder.stop_recording()
        if saved_file:
            logger.info(f"Manual flight video saved at: {saved_file}")
    except Exception as e:
        logger.error(f"Error finalizing video: {e}")

    logger.info("Returning to STANDBY mode.")


# ---------------------------------------------------------------------------
# Autonomous Ground Mission
# ---------------------------------------------------------------------------
def run_ground_autonomous_mission(master, state: TelemetryState):
    """
    Executes the autonomous flight mission when triggered on the ground with CH8 ≈ 1500:
      1. Pre-flight health inspection
      2. Start video recording
      3. Enter GUIDED mode (verified)
      4. Arm motors (if not armed)
      5. Takeoff to 1.2 m target altitude
      6. Verify altitude within ±0.15 m and stable
      7. Capture the fixed local-NED origin after stabilization
      8. Move to a 3 m radius, complete one origin-fixed 360° orbit, and return
      9. Stop horizontal movement
     10. Execute ultra-slow multi-phase soft landing for concrete
     11. Confirm touchdown & disarm
     12. Finalize video recording
     13. Return to STANDBY
    """
    logger.info("========================================================")
    logger.info("=== STARTING AUTONOMOUS GROUND FLIGHT SEQUENCE ===")
    logger.info("========================================================")
    log_telemetry_health(state)

    # Remember whether this mission is under RC supervision, so that losing the
    # RC link aborts an RC-triggered mission but does not abort a mission that
    # was deliberately started with no receiver attached.
    state.mission_uses_rc = state.rc_present

    # Pre-flight safety check: only enforce the CH8 gate when there is an RC link.
    if state.rc_present and state.chan8_state != "GUIDED":
        logger.warning(f"CH8 is not in GUIDED position ({state.chan8_state}). Aborting.")
        return

    if not state.rc_present:
        logger.warning("No RC receiver detected — running without RC supervision. "
                       "There is no stick/switch override available for this flight.")

    # 1. Start Video Recording
    logger.info("Starting onboard video recording...")
    try:
        video_recorder.start_recording()
    except Exception as e:
        logger.warning(f"Could not start video: {e}")

    mission_success = False
    try:
        # 2. Switch to GUIDED Mode if not already in GUIDED
        if state.flight_mode != "GUIDED":
            mode_ok = set_flight_mode(master, "GUIDED", state)
            if not mode_ok:
                logger.error("Failed to enter GUIDED mode. Aborting.")
                return

        # Re-check CH8 after mode change
        if state.mission_uses_rc and state.chan8_state != "GUIDED":
            logger.warning(f"CH8 changed to {state.chan8_state} during mode switch. Aborting.")
            return

        # 3. Arm Aircraft (if not already armed)
        if not state.is_armed:
            armed_ok = arm_motors(master, state, timeout_sec=8.0)
            if not armed_ok and not state.is_armed:
                logger.error("Arming failed. Aborting autonomous mission.")
                return
            time.sleep(1.0)

        # Re-check CH8 after arming
        if state.mission_uses_rc and state.chan8_state != "GUIDED":
            logger.warning(f"CH8 changed to {state.chan8_state} after arming. Aborting.")
            if state.chan8_state == "LAND":
                set_flight_mode(master, "LAND", state)
            return

        # 4. Command Takeoff — verify ArduPilot accepted the command
        if not send_takeoff_command(master, state, TAKEOFF_ALTITUDE_M):
            logger.error("Takeoff command was rejected. Disarming and aborting.")
            if state.is_armed and is_drone_on_ground(state):
                disarm_motors(master, state)
            return

        # 5. Wait for Climb — requires tight altitude acceptance and stability
        climb_ok = wait_for_climb(master, state, TAKEOFF_ALTITUDE_M, timeout_sec=30.0)
        if not climb_ok:
            logger.error("Takeoff climb not confirmed. Aborting horizontal orbit.")
            # Never left the ground: shut the motors down rather than leaving
            # an armed aircraft sitting there with spinning props.
            if state.is_armed and is_drone_on_ground(state):
                logger.info("Aircraft never left the ground. Disarming.")
                disarm_motors(master, state)
                time.sleep(1.0)
                return
            # Transition to safest state
            if state.is_armed and not is_drone_on_ground(state):
                if state.fence_breached:
                    run_geofence_breach_land(master, state)
                else:
                    logger.info("Aircraft is airborne after failed takeoff confirmation. Commanding LAND.")
                    set_flight_mode(master, "LAND", state)
                    run_in_air_safety_land(master, state)
            return

        # Final safety check before horizontal orbit
        abort_reason = check_autonomous_abort(state, "pre-orbit")
        if abort_reason:
            logger.warning(abort_reason)
            if state.chan8_state == "LAND":
                set_flight_mode(master, "LAND", state)
            if state.is_armed and not is_drone_on_ground(state):
                if state.fence_breached:
                    run_geofence_breach_land(master, state)
                else:
                    run_in_air_safety_land(master, state)
            return

        logger.info("Takeoff stabilized. Hovering for 2.0 seconds...")
        hover_ok = send_body_velocity_with_override_check(
            master, state,
            vx=0.0, vy=0.0, vz=0.0, yaw_rate_dps=0.0,
            duration_s=2.0
        )
        if not hover_ok:
            logger.warning("Hover interrupted. Yielding control.")
            if state.is_armed and not is_drone_on_ground(state) and state.chan8_state != "MANUAL":
                if state.fence_breached:
                    run_geofence_breach_land(master, state)
                else:
                    run_in_air_safety_land(master, state)
            return

        # 6. Capture the fixed origin, orbit it once, and return to it.
        orbit_ok = fly_horizontal_geofence_orbit(master, state)
        if not orbit_ok:
            logger.warning("Horizontal geofence orbit interrupted. Yielding control.")
            if state.is_armed and not is_drone_on_ground(state) and state.chan8_state != "MANUAL":
                if state.fence_breached:
                    run_geofence_breach_land(master, state)
                else:
                    run_in_air_safety_land(master, state)
            return

        # 7. Stop horizontal movement — hover before landing
        logger.info("Stopping horizontal movement. Hovering for 3.0s before landing...")
        hover_ok = send_body_velocity_with_override_check(
            master, state,
            vx=0.0, vy=0.0, vz=0.0, yaw_rate_dps=0.0,
            duration_s=3.0
        )
        if not hover_ok:
            logger.warning("Pre-landing hover interrupted. Yielding control.")
            if state.is_armed and not is_drone_on_ground(state) and state.chan8_state != "MANUAL":
                if state.fence_breached:
                    run_geofence_breach_land(master, state)
                else:
                    run_in_air_safety_land(master, state)
            return

        # 8. Execute Ultra-Slow Controlled Soft Landing on Concrete
        landing_success = execute_controlled_soft_landing(master, state)
        if not landing_success:
            logger.warning("Controlled landing did not complete successfully.")
            if state.is_armed and not is_drone_on_ground(state) and state.chan8_state != "MANUAL":
                if state.fence_breached:
                    run_geofence_breach_land(master, state)
                else:
                    run_in_air_safety_land(master, state)
            return

        mission_success = True
        logger.info("=== Autonomous Mission Completed Successfully! ===")

    except Exception as e:
        logger.exception(f"Mission encountered an unexpected error: {e}")
        # CRITICAL: Do not just log and return — transition to safest state
        logger.warning("Unexpected exception during mission. Attempting safe-state transition...")
        try:
            if state.is_armed and not is_drone_on_ground(state):
                logger.info("Aircraft is airborne during exception. Commanding LAND.")
                set_flight_mode(master, "LAND", state)
                # Monitor landing
                land_start = time.monotonic()
                while time.monotonic() - land_start < 30.0:
                    if not state.is_armed and is_drone_on_ground(state):
                        logger.info("Aircraft landed after exception-triggered LAND.")
                        break
                    time.sleep(0.5)
        except Exception as e2:
            logger.error(f"Error during exception safe-state transition: {e2}")

    finally:
        # Finalize Video Recording
        try:
            saved_file = video_recorder.stop_recording()
            if saved_file:
                logger.info(f"Flight video saved successfully at: {saved_file}")
        except Exception as e:
            logger.error(f"Error finalizing video: {e}")

        if mission_success:
            logger.info("Mission cycle finished successfully. Returning to STANDBY mode.")
        else:
            logger.info("Mission cycle finished (not fully successful). Returning to STANDBY mode.")


# ---------------------------------------------------------------------------
# Supervisor Loop
# ---------------------------------------------------------------------------
def consume_trigger_file() -> bool:
    """
    Returns True (once) if the mission trigger file is present, removing it so a
    single trigger starts exactly one mission.
    """
    try:
        if os.path.exists(TRIGGER_FILE):
            os.remove(TRIGGER_FILE)
            return True
    except OSError as e:
        logger.warning(f"Could not consume trigger file {TRIGGER_FILE}: {e}")
    return False


def supervisor_loop(master, state: TelemetryState, stop_event: threading.Event):
    """
    Continuous supervisor loop that monitors CH8 control authority, flight mode,
    and arming triggers. Transitions between STANDBY, AUTO MISSION, SAFETY LAND,
    and MANUAL FLIGHT.

    Mission triggers, in order of precedence:
      1. CH8 moved into the GUIDED band (only when a receiver is actually present,
         and only while TRIGGER_ON_GUIDED_MODE is off)
      1b. TRIGGER_ON_GUIDED_MODE=1 instead: the aircraft being put into actual
         GUIDED flight mode, with CH8 in GUIDED when a receiver is present
      2. The trigger file being created (works with no receiver)
      3. AUTOSTART_ON_READY=1, with a live RC link already in GUIDED
      4. AUTOSTART_WITHOUT_RC=1, which fires once per connection without RC

    A failed mission is followed by a cooldown.  Previously the GUIDED branch
    re-entered the mission on every loop iteration while the aircraft sat
    disarmed on the ground, which restarted the whole sequence — and a fresh
    video recording — every few seconds.
    """
    logger.info("Continuous flight supervisor active. Ready for flight triggers.")
    if AUTOSTART_ON_READY:
        logger.info("[AUTOSTART] Enabled: one mission will start after the FC is ready, "
                    "the receiver is live, and CH8 is already GUIDED. "
                    "No trigger file or CH8 transition is required.")
    if TRIGGER_ON_GUIDED_MODE:
        logger.info("[TRIGGER] Mission starts on an observed transition into GUIDED "
                    "flight mode (CH8 must also be GUIDED when a receiver is present). "
                    "The CH8 launch condition is disabled; CH8=MANUAL/LAND still aborts.")
    if not state.rc_present:
        logger.warning("No RC receiver data. CH8 triggering is unavailable; "
                       f"create {TRIGGER_FILE} to start a mission"
                       + (" (AUTOSTART_WITHOUT_RC is enabled)." if AUTOSTART_WITHOUT_RC else "."))

    last_standby_log = 0.0
    last_ch8_state = None
    last_mission_end = 0.0
    autostart_fired = False

    # The CH8 trigger is armed only after the switch has been SEEN in a
    # non-GUIDED position. Without this, finding the switch already in the GUIDED
    # band counts as a transition (None -> GUIDED) and launches a flight the
    # moment the daemon connects: at boot, or on any USB re-enumeration mid-session.
    # The pilot must move the switch out of GUIDED and back in to start a mission.
    ch8_trigger_armed = False

    # The GUIDED-mode trigger is armed the same way: only after the aircraft has
    # been SEEN in a mode other than GUIDED. Without this, a daemon restart or a
    # USB re-enumeration while the FC happens to sit in GUIDED would count as a
    # transition (None -> GUIDED) and arm the aircraft unattended.
    guided_trigger_armed = False
    last_flight_mode = None

    while not stop_event.is_set():
        now = time.monotonic()

        # 1. Connection / Heartbeat Watchdog
        if state.last_heartbeat > 0 and (now - state.last_heartbeat > 10.0):
            logger.warning("Heartbeat timeout (>10s without telemetry). Flight controller disconnected.")
            return

        ch8 = state.chan8_state
        ch8_changed = (ch8 != last_ch8_state)
        is_ground = is_drone_on_ground(state)
        cooling_down = (now - last_mission_end) < MISSION_COOLDOWN_S

        if ch8_changed:
            logger.info(f"[SUPERVISOR] CH8 state: {last_ch8_state} -> {ch8} "
                        f"(Mode={state.flight_mode}, Armed={state.is_armed}, "
                        f"InAir={state.in_air}, AGL={state.relative_altitude_m:.2f}m)")
            if not TRIGGER_ON_GUIDED_MODE and ch8 == "GUIDED" and not ch8_trigger_armed:
                logger.warning("[SUPERVISOR] CH8 is in the GUIDED band, but the switch was "
                               "already there when the link came up. Ignoring it. "
                               "Move CH8 to MANUAL and back to GUIDED to start a mission.")
            last_ch8_state = ch8

        # Seeing the switch away from GUIDED arms the trigger for the next move into it.
        if ch8 in ("MANUAL", "LAND"):
            if not ch8_trigger_armed and not TRIGGER_ON_GUIDED_MODE:
                logger.info("[SUPERVISOR] CH8 trigger armed — moving CH8 to GUIDED will now "
                            "start an autonomous mission.")
            ch8_trigger_armed = True

        # 2. Geofence Breach Detection
        if state.fence_breached and state.is_armed and not is_ground:
            if ch8 != "MANUAL":
                logger.warning("[SUPERVISOR] Geofence breach detected while airborne -> Executing Geofence Breach Land...")
                run_geofence_breach_land(master, state)
                last_ch8_state = state.chan8_state
                last_mission_end = time.monotonic()
                continue

        # ---------------------------------------------------------------
        # CH8 = LAND: Immediate LAND command
        # ---------------------------------------------------------------
        if ch8 == "LAND":
            if state.is_armed and not is_ground:
                logger.warning("[SUPERVISOR] CH8=LAND while airborne -> Executing Safety Land...")
                run_in_air_safety_land(master, state)
                last_ch8_state = state.chan8_state
                last_mission_end = time.monotonic()
                continue
            if ch8_changed:
                logger.info("[SUPERVISOR] CH8=LAND but aircraft is on ground. No action needed.")

        # ---------------------------------------------------------------
        # CH8 = MANUAL, or no RC at all: the companion does not own the aircraft
        # ---------------------------------------------------------------
        elif ch8 in ("MANUAL", "NO_SIGNAL"):
            if state.is_armed and state.in_air:
                # Someone else is flying it — monitor only, send no commands.
                handle_manual_flight(master, state)
                last_ch8_state = state.chan8_state
                last_mission_end = time.monotonic()
                continue

        # ---------------------------------------------------------------
        # CH8 = GUIDED: an RC-supervised mission may start
        # ---------------------------------------------------------------
        elif ch8 == "GUIDED":
            if state.is_armed and state.in_air:
                # Switched to GUIDED while already flying -> Safety Land
                logger.warning("[SUPERVISOR] CH8=GUIDED while IN AIR -> Executing Safety Land...")
                run_in_air_safety_land(master, state)
                last_ch8_state = state.chan8_state
                last_mission_end = time.monotonic()
                continue
            if (not TRIGGER_ON_GUIDED_MODE and ch8_changed and ch8_trigger_armed
                    and is_ground and not cooling_down):
                ch8_trigger_armed = False
                logger.info("[SUPERVISOR] CH8=GUIDED on GROUND -> Starting Autonomous Ground Mission...")
                run_ground_autonomous_mission(master, state)
                last_ch8_state = state.chan8_state
                last_mission_end = time.monotonic()
                continue

        # ---------------------------------------------------------------
        # Flight-mode trigger: the pilot puts the aircraft into actual GUIDED
        # ---------------------------------------------------------------
        if TRIGGER_ON_GUIDED_MODE:
            mode = state.flight_mode
            mode_changed = (mode != last_flight_mode)

            if mode_changed:
                logger.info(f"[SUPERVISOR] Flight mode: {last_flight_mode} -> {mode} "
                            f"(CH8={ch8}, Armed={state.is_armed}, "
                            f"InAir={state.in_air}, AGL={state.relative_altitude_m:.2f}m)")
                if mode == "GUIDED" and not guided_trigger_armed:
                    logger.warning("[SUPERVISOR] Aircraft is in GUIDED, but the mode was never "
                                   "seen leaving GUIDED first. Ignoring it. Select another mode "
                                   "and return to GUIDED to start a mission.")
                last_flight_mode = mode

            # Seeing a real non-GUIDED mode arms the trigger for the next entry
            # into GUIDED. UNKNOWN is a telemetry gap, not a mode change, and
            # must not arm anything.
            if mode not in ("GUIDED", "UNKNOWN"):
                if not guided_trigger_armed:
                    logger.info("[SUPERVISOR] GUIDED-mode trigger armed — putting the aircraft "
                                "into GUIDED will now start an autonomous mission.")
                guided_trigger_armed = True

            if (mode_changed and mode == "GUIDED" and guided_trigger_armed
                    and is_ground and not state.is_armed and not cooling_down):
                # When a receiver is present CH8 must still hand control to the
                # companion, so the manual-takeover path stays live for the flight.
                if state.rc_present and ch8 != "GUIDED":
                    logger.warning(f"[SUPERVISOR] GUIDED mode selected but CH8={ch8}. Not "
                                   "starting a mission. Move CH8 to GUIDED, then re-select "
                                   "GUIDED mode.")
                else:
                    guided_trigger_armed = False
                    logger.info("[SUPERVISOR] GUIDED mode selected on GROUND -> "
                                "Starting Autonomous Ground Mission...")
                    run_ground_autonomous_mission(master, state)
                    last_ch8_state = state.chan8_state
                    # Re-sync after the blocking mission so the mode it left
                    # behind cannot read as a fresh transition next iteration.
                    last_flight_mode = state.flight_mode
                    guided_trigger_armed = False
                    last_mission_end = time.monotonic()
                    continue

        # ---------------------------------------------------------------
        # Non-RC triggers
        # ---------------------------------------------------------------
        if is_ground and not state.is_armed and not cooling_down:
            trigger_reason = None
            if consume_trigger_file():
                trigger_reason = f"trigger file {TRIGGER_FILE}"
            elif (AUTOSTART_ON_READY and not autostart_fired and state.rc_present
                  and state.chan8_state == "GUIDED"):
                autostart_fired = True
                trigger_reason = "AUTOSTART_ON_READY (RC live, CH8=GUIDED)"
            elif AUTOSTART_WITHOUT_RC and not autostart_fired and not state.rc_present:
                autostart_fired = True
                trigger_reason = "AUTOSTART_WITHOUT_RC"

            if trigger_reason:
                logger.info(f"[SUPERVISOR] Mission triggered by {trigger_reason} -> "
                            f"Starting Autonomous Ground Mission...")
                run_ground_autonomous_mission(master, state)
                last_ch8_state = state.chan8_state
                last_mission_end = time.monotonic()
                continue

        # Periodic standby status log (every 15s)
        if now - last_standby_log >= 15.0:
            if TRIGGER_ON_GUIDED_MODE:
                if not guided_trigger_armed:
                    wait_for = "the aircraft to leave GUIDED, then return (trigger not armed)"
                elif state.rc_present and ch8 != "GUIDED":
                    wait_for = "CH8=GUIDED, then GUIDED flight mode"
                else:
                    wait_for = "the aircraft to be put into GUIDED flight mode"
            elif AUTOSTART_ON_READY and not autostart_fired:
                wait_for = "automatic start: RC live with CH8=GUIDED"
            elif not state.rc_present:
                wait_for = f"trigger file ({TRIGGER_FILE})"
            elif ch8_trigger_armed:
                wait_for = "CH8=GUIDED"
            else:
                wait_for = "CH8 to leave GUIDED, then return (trigger not armed)"
            logger.info(
                f"[STANDBY] Waiting for {wait_for}... "
                f"(CH8={ch8}, Mode={state.flight_mode}, Armed={state.is_armed}, "
                f"Batt={state.battery_percent:.0f}%, AGL={state.relative_altitude_m:.2f}m)"
            )
            last_standby_log = now

        time.sleep(0.2)


# ---------------------------------------------------------------------------
# GCS Heartbeat Sender
# ---------------------------------------------------------------------------
def gcs_heartbeat_loop(master, stop_event: threading.Event):
    """
    Sends periodic GCS heartbeats to ArduPilot at ~1 Hz.
    ArduPilot requires heartbeats from companion computers to:
    - Accept commands (arm, mode change, velocity setpoints)
    - Prevent GCS failsafe triggering
    """
    logger.info("Starting GCS heartbeat sender (1 Hz)...")
    while not stop_event.is_set():
        try:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,   # type
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,         # autopilot
                0,                                             # base_mode
                0,                                             # custom_mode
                0,                                             # system_status
            )
        except Exception as e:
            logger.debug(f"GCS heartbeat send error: {e}")
        stop_event.wait(1.0)


# ---------------------------------------------------------------------------
# Main Daemon
# ---------------------------------------------------------------------------
def main_daemon():
    """Top-level continuous daemon loop with connection recovery and graceful teardown."""
    setup_logging()
    logger.info("==========================================================")
    logger.info("=== ARDUPILOT CONTINUOUS AUTONOMOUS MISSION DAEMON ===")
    logger.info("==========================================================")
    logger.info(
        f"Mission Parameters: Port={CONNECTION_STRING or 'auto-discover'}, Baud={BAUD_RATE}, "
        f"TakeoffAlt={TAKEOFF_ALTITUDE_M}m, "
        f"DescentSpeed={DESCENT_SPEED_MPS}m/s, "
        f"TouchdownSpeed={TOUCHDOWN_SPEED_MPS}m/s, "
        f"Orbit={ORBIT_RADIUS_M}m x {ORBIT_LOOPS} at {ORBIT_SPEED_MPS}m/s, "
        f"HorizontalFence={HORIZONTAL_GEOFENCE_RADIUS_M}m, "
        f"Geofence={f'{FENCE_RADIUS_M}m circle, {FENCE_ALT_MAX_M}m max alt' if CONFIGURE_FENCE else 'DISABLED'}"
    )

    while True:
        try:
            connection = resolve_connection_string()
            is_network = connection is not None and (
                connection.startswith("udp") or connection.startswith("tcp")
            )
            if connection is None or (not is_network and not os.path.exists(connection)):
                logger.info("Waiting for ArduPilot flight controller... "
                            "(no serial device found, retrying in 3s)")
                time.sleep(3.0)
                continue

            logger.info(f"Connecting to ArduPilot flight controller at {connection} (baud: {BAUD_RATE})...")
            master = mavutil.mavlink_connection(
                connection,
                baud=BAUD_RATE,
                dialect="ardupilotmega",
                autoreconnect=True,
            )

            logger.info("Waiting for ArduPilot heartbeat...")
            heartbeat = master.wait_heartbeat(timeout=8.0)
            if not heartbeat:
                logger.info("Waiting for ArduPilot flight controller on serial link... (retrying in 3s)")
                master.close()
                time.sleep(3.0)
                continue

            # pymavlink can leave target_component at 0. Commands still reach the
            # autopilot, but addressing it explicitly avoids any ambiguity when
            # other components (gimbal, radio) share the link.
            if master.target_component in (0, mavutil.mavlink.MAV_COMP_ID_ALL):
                master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

            logger.info(
                f"Connected to ArduPilot! (System ID: {master.target_system}, "
                f"Component ID: {master.target_component})"
            )

            # Start background telemetry thread
            state = TelemetryState()
            stop_event = threading.Event()
            telemetry_thread = threading.Thread(
                target=telemetry_listener_loop,
                args=(master, state, stop_event),
                daemon=True,
            )
            telemetry_thread.start()

            # Start GCS heartbeat thread (ArduPilot requires periodic heartbeats)
            heartbeat_thread = threading.Thread(
                target=gcs_heartbeat_loop,
                args=(master, stop_event),
                daemon=True,
            )
            heartbeat_thread.start()

            try:
                # Request message streams and configure soft landing parameters
                configure_ardupilot_message_streams(master)
                configure_ardupilot_landing_parameters(master)
                # Let the telemetry cache fill before deciding anything: the old
                # 1.5s wait meant the first mission decision was made on partly
                # empty state (mode UNKNOWN, no RC frame seen yet).
                time.sleep(3.0)
                log_telemetry_health(state)
                if not state.rc_present:
                    logger.warning("Flight controller reports NO RC receiver data "
                                   "(RC_CHANNELS chancount=0).")
                if state.battery_voltage_v < 1.0:
                    logger.warning(f"Flight controller reports battery voltage "
                                   f"{state.battery_voltage_v:.2f}V — no flight battery detected. "
                                   f"The motors cannot run on USB power alone.")
                logger.info("System initialized and READY in STANDBY mode.")

                # Run continuous flight supervisor loop
                supervisor_loop(master, state, stop_event)

            finally:
                # Clean up telemetry thread and connection
                stop_event.set()
                telemetry_thread.join(timeout=2.0)
                heartbeat_thread.join(timeout=2.0)
                if video_recorder.is_recording():
                    video_recorder.stop_recording()
                master.close()

        except KeyboardInterrupt:
            logger.info("Daemon received interrupt. Exiting...")
            break
        except Exception as e:
            logger.exception(f"Supervisor loop encountered error: {e}. Reconnecting in 3 seconds...")
            time.sleep(3.0)


def handle_exit_signal(sig, frame):
    """Signal handler for graceful shutdown on SIGINT / SIGTERM."""
    logger.info(f"Received exit signal {sig}. Finalizing video and exiting...")
    video_recorder.stop_recording()
    sys.exit(0)


def acquire_single_instance_lock():
    """
    Takes an exclusive lock so only one daemon can own the serial link.

    Two daemons reading /dev/ttyACM0 at once each consume part of the other's
    bytes.  The result is silently corrupted telemetry — flight mode parsed as
    UNKNOWN, heartbeats vanishing mid-climb, and SerialException reconnect loops.
    Returns the open file object, which must stay referenced for the lock to hold.
    """
    lock_handle = open(LOCK_FILE, "w")
    try:
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print(f"Another ardupilot_mission.py instance already holds {LOCK_FILE}. Exiting.",
              file=sys.stderr)
        sys.exit(1)
    lock_handle.write(f"{os.getpid()}\n")
    lock_handle.flush()
    return lock_handle


def main():
    signal.signal(signal.SIGINT, handle_exit_signal)
    signal.signal(signal.SIGTERM, handle_exit_signal)

    # Held for the lifetime of the process.
    global _instance_lock
    _instance_lock = acquire_single_instance_lock()

    try:
        main_daemon()
    except KeyboardInterrupt:
        logger.info("Daemon terminated by user.")
        video_recorder.stop_recording()
        sys.exit(0)
    except Exception:
        logger.exception("Daemon exited with unhandled exception.")
        video_recorder.stop_recording()
        sys.exit(1)


if __name__ == "__main__":
    main()
