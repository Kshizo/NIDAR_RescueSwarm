#!/usr/bin/env python3
"""
ArduPilot Pre-Flight Readiness Check
====================================
Read-only. Sends no arm, mode or movement command — it asks the flight
controller to re-run its own pre-arm checks and reports what it says.

Run this after fixing the RC link or calibrating the compass to confirm the
aircraft actually considers itself flyable, without arming anything.

    python preflight_check.py

Exit status is 0 when every check passes, 1 otherwise, so it can gate a script.
"""

import fcntl
import os
import sys
import threading
import time

from pymavlink import mavutil

SERIAL_BY_ID_DIR = "/dev/serial/by-id"
V4L_BY_ID_DIR = "/dev/v4l/by-id"
LOCK_FILE = os.getenv("ARDUPILOT_LOCK_FILE", "/tmp/ardupilot_mission.lock")
BAUD_RATE = int(os.getenv("ARDUPILOT_BAUD", "115200"))
SETTLE_SECONDS = float(os.getenv("PREFLIGHT_SETTLE_S", "12"))

# EKF_STATUS_REPORT flag bits.
EKF_FLAGS = [
    (1, "ATTITUDE"), (2, "VEL_HORIZ"), (4, "VEL_VERT"), (8, "POS_HORIZ_REL"),
    (16, "POS_HORIZ_ABS"), (32, "POS_VERT_ABS"), (64, "POS_VERT_AGL"),
    (128, "CONST_POS_MODE"), (256, "PRED_POS_HORIZ_REL"), (512, "PRED_POS_HORIZ_ABS"),
]

OK, WARN, FAIL = "OK", "WARN", "FAIL"
_results = []


def record(name, status, detail):
    _results.append((name, status, detail))


def find_flight_controller():
    try:
        for name in sorted(os.listdir(SERIAL_BY_ID_DIR)):
            if "ArduPilot" in name or "MicoAir" in name:
                return os.path.join(SERIAL_BY_ID_DIR, name)
    except OSError:
        pass
    return None


def check_daemon_not_running():
    """
    Only one process may read the serial link. Two readers each consume part of
    the other's bytes and corrupt the MAVLink stream, so refuse rather than
    produce a misleading result.
    """
    try:
        handle = open(LOCK_FILE, "w")
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return handle
    except OSError:
        print("The mission daemon is running and owns the serial link.\n"
              "Stop it first, then re-run this check:\n"
              "    systemctl stop ardupilot-mission.service\n"
              "    python preflight_check.py\n"
              "    systemctl start ardupilot-mission.service", file=sys.stderr)
        return None


def start_gcs_heartbeat(master):
    """
    The aircraft runs with FS_GCS_ENABLE=5, so it expects a ~1 Hz heartbeat from
    the companion computer. Without one the flight controller reports
    "PreArm: GCS failsafe on" and this check would show a blocker it created
    itself. The mission daemon sends the same heartbeat in flight, so sending it
    here also makes the check reflect real flight conditions.

    Returns an Event; set it to stop the sender.
    """
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            try:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            stop.wait(1.0)

    threading.Thread(target=loop, daemon=True).start()
    return stop


def request_streams(master):
    wanted = [
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 5),
        (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 2),
    ]
    for msg_id, hz in wanted:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1_000_000 / hz), 0, 0, 0, 0, 0)
        time.sleep(0.02)


def collect(master, seconds):
    """Gathers telemetry and the flight controller's own pre-arm verdict."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, 0)

    snap = {"prearm": [], "armed": False}
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "HEARTBEAT" and msg.get_srcComponent() in (0, 1):
            snap["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            snap["mode"] = mavutil.mode_string_v10(msg)
        elif kind == "SYS_STATUS":
            snap["voltage"] = msg.voltage_battery / 1000.0
            snap["battery_pct"] = msg.battery_remaining
        elif kind == "GPS_RAW_INT":
            snap["fix"] = msg.fix_type
            snap["sats"] = msg.satellites_visible
            snap["eph"] = msg.eph / 100.0
        elif kind == "RC_CHANNELS":
            snap["chancount"] = msg.chancount
            snap["ch8"] = msg.chan8_raw
            snap["rssi"] = msg.rssi
        elif kind == "EKF_STATUS_REPORT":
            snap["ekf"] = msg.flags
        elif kind == "HOME_POSITION":
            snap["home"] = (msg.latitude / 1e7, msg.longitude / 1e7)
        elif kind == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            text = text.strip()
            if text.lower().startswith(("prearm", "arm:")) and text not in snap["prearm"]:
                snap["prearm"].append(text)
    return snap


def evaluate(snap, arm_voltage):
    volts = snap.get("voltage")
    if volts is None:
        record("Battery", FAIL, "no battery telemetry")
    elif volts < 1.0:
        record("Battery", FAIL, f"{volts:.2f} V — no flight battery detected")
    elif volts < arm_voltage:
        record("Battery", FAIL, f"{volts:.2f} V is below the {arm_voltage:.1f} V arming threshold")
    else:
        record("Battery", OK, f"{volts:.2f} V ({snap.get('battery_pct', '?')}%)")

    fix = snap.get("fix", 0)
    if fix >= 3:
        record("GPS", OK, f"3D fix, {snap.get('sats','?')} sats, eph {snap.get('eph',0):.2f} m")
    else:
        record("GPS", FAIL, f"fix type {fix} — no 3D fix")

    ekf = snap.get("ekf")
    if ekf is None:
        record("EKF position", FAIL, "no EKF status received")
    else:
        names = [n for bit, n in EKF_FLAGS if ekf & bit]
        if ekf & 16 and not ekf & 128:
            record("EKF position", OK, f"horizontal position valid (flags={ekf})")
        else:
            record("EKF position", FAIL,
                   f"no horizontal position (flags={ekf}: {', '.join(names) or 'none'})")

    if snap.get("home"):
        lat, lon = snap["home"]
        record("Home point", OK, f"set at {lat:.6f}, {lon:.6f}")
    else:
        record("Home point", FAIL, "not set — GUIDED cannot hold position")

    # A missing receiver is a WARN, not a FAIL: the transmitter is switched on at
    # the field, just before flight, so it is normally absent during a bench
    # check. Nothing is bypassed by this — the flight controller enforces it
    # itself ("PreArm: RC not found" below), and that stays a blocker.
    chancount = snap.get("chancount")
    if chancount is None:
        record("RC receiver", WARN, "no RC_CHANNELS messages at all — is the transmitter on?")
    elif chancount == 0 or snap.get("ch8", 0) == 0:
        record("RC receiver", WARN,
               f"no receiver data (chancount={chancount}, rssi={snap.get('rssi','?')}) "
               f"— switch the transmitter on; no manual override until you do")
    else:
        record("RC receiver", OK,
               f"{chancount} channels, CH8={snap.get('ch8')}, rssi={snap.get('rssi','?')}")

    compass_msgs = [t for t in snap["prearm"] if "mag field" in t.lower() or "compass" in t.lower()]
    if compass_msgs:
        record("Compass", FAIL, compass_msgs[0])
    else:
        record("Compass", OK, "no compass complaint from the flight controller")

    try:
        cameras = [n for n in os.listdir(V4L_BY_ID_DIR) if n.endswith("index0")]
    except OSError:
        cameras = []
    if cameras:
        record("Camera", OK, cameras[0])
    else:
        record("Camera", WARN, "no USB camera detected — flights will record no video")

    blockers = snap["prearm"]
    if blockers:
        record("Pre-arm", FAIL, f"flight controller reports {len(blockers)} blocker(s)")
    else:
        record("Pre-arm", OK, "flight controller reports no blockers")
    return blockers


def read_arm_voltage(master, default=14.8):
    master.mav.param_request_read_send(
        master.target_system, master.target_component, b"BATT_ARM_VOLT", -1)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
        if msg is None:
            continue
        name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        if name.strip("\x00") == "BATT_ARM_VOLT":
            return msg.param_value
    return default


def main():
    lock = check_daemon_not_running()
    if lock is None:
        return 2

    port = os.getenv("ARDUPILOT_CONNECTION") or find_flight_controller()
    if port is None:
        print("No flight controller found under /dev/serial/by-id/. Is it plugged in?",
              file=sys.stderr)
        return 2

    print("ArduPilot pre-flight check — read-only, nothing will be armed.")
    print(f"Link: {port}")
    master = mavutil.mavlink_connection(port, baud=BAUD_RATE, dialect="ardupilotmega")
    if not master.wait_heartbeat(timeout=10):
        print("No heartbeat from the flight controller.", file=sys.stderr)
        return 2
    if master.target_component in (0, mavutil.mavlink.MAV_COMP_ID_ALL):
        master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

    # Start this before anything else asks the board a question, so the GCS
    # failsafe has cleared by the time the pre-arm checks are run.
    heartbeat_stop = start_gcs_heartbeat(master)

    arm_voltage = read_arm_voltage(master)
    request_streams(master)
    time.sleep(2.0)
    print(f"Listening for {SETTLE_SECONDS:.0f}s...\n")
    snap = collect(master, SETTLE_SECONDS)
    blockers = evaluate(snap, arm_voltage)
    heartbeat_stop.set()
    master.close()

    width = max(len(name) for name, _, _ in _results)
    for name, status, detail in _results:
        dots = "." * (width + 3 - len(name))
        print(f"  {name} {dots} {status:<4}  {detail}")

    if blockers:
        print("\n  Flight controller's own words:")
        for text in blockers:
            print(f"    - {text}")

    failures = [n for n, s, _ in _results if s == FAIL]
    warnings = [n for n, s, _ in _results if s == WARN]
    print()
    if failures:
        print(f"VERDICT: NOT READY TO FLY — {len(failures)} blocking check(s): {', '.join(failures)}")
        return 1
    if warnings:
        print(f"VERDICT: READY TO FLY, with warnings: {', '.join(warnings)}")
        return 0
    print("VERDICT: READY TO FLY — all checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
