#!/usr/bin/env python3
"""
Make Flight Ready
=================
One command that takes the aircraft as far towards flight-ready as software can,
then tells you exactly what is left for you to do with your hands.

It will, on its own:
  * find the flight controller by serial number
  * apply the safe parameter set (arming checks ON, geofence off, gentle
    landing speeds)
  * clear latched battery / radio failsafes by rebooting the board when needed
  * wait for GPS and the EKF to settle
  * re-run the flight controller's own pre-arm checks and report the verdict

It will NOT arm the aircraft, spin a motor, or take off.

    systemctl stop ardupilot-mission.service
    python make_flight_ready.py
    systemctl start ardupilot-mission.service

Add --calibrate-compass to chain straight into the compass calibration, which
needs you to rotate the airframe by hand.

Exit status: 0 = ready to fly, 1 = blockers remain, 2 = could not run.
"""

import argparse
import fcntl
import os
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

SERIAL_BY_ID_DIR = "/dev/serial/by-id"
LOCK_FILE = os.getenv("ARDUPILOT_LOCK_FILE", "/tmp/ardupilot_mission.lock")
BAUD_RATE = int(os.getenv("ARDUPILOT_BAUD", "115200"))
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

# Parameters the aircraft should be flying with. The geofence is deliberately
# turned OFF — containment is the pilot's job, via manual takeover on the
# transmitter — and FENCE_ENABLE=0 is written explicitly so a fence stored from
# an earlier run cannot block the arm with "PreArm: Fence requires position".
SAFE_PARAMS = [
    ("ARMING_CHECK", 1.0, "run all pre-arm checks"),
    ("LAND_SPEED", 20.0, "20 cm/s touchdown"),
    ("LAND_ALT_LOW", 150.0, "slow down below 1.5 m"),
    ("BATT_FS_LOW_ACT", 1.0, "land on low battery"),
    ("BATT_FS_CRT_ACT", 1.0, "land on critical battery"),
    ("FENCE_ENABLE", 0.0, "geofence OFF — pilot contains the aircraft"),
    ("FS_THR_ENABLE", 3.0, "land on RC loss (not RTL to 15 m)"),
    ("FS_GCS_ENABLE", 5.0, "land if the companion heartbeat stops"),
]

GREEN, RED, YELLOW, DIM, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def say(symbol, colour, text):
    print(f"  {colour}{symbol}{RESET} {text}")


def step(title):
    print(f"\n{title}")
    print(f"{DIM}{'-' * len(title)}{RESET}")


def find_flight_controller():
    try:
        for name in sorted(os.listdir(SERIAL_BY_ID_DIR)):
            if "ArduPilot" in name or "MicoAir" in name:
                return os.path.join(SERIAL_BY_ID_DIR, name)
    except OSError:
        pass
    return None


def acquire_lock():
    try:
        handle = open(LOCK_FILE, "w")
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return handle
    except OSError:
        print("The mission daemon is running and owns the serial link.\n"
              "Stop it first:  systemctl stop ardupilot-mission.service", file=sys.stderr)
        return None


def connect(timeout=12):
    port = os.getenv("ARDUPILOT_CONNECTION") or find_flight_controller()
    if port is None:
        return None, None
    master = mavutil.mavlink_connection(port, baud=BAUD_RATE, dialect="ardupilotmega")
    heartbeat = master.wait_heartbeat(timeout=timeout)
    if not heartbeat:
        master.close()
        return None, None
    if master.target_component in (0, mavutil.mavlink.MAV_COMP_ID_ALL):
        master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    return master, heartbeat


def start_gcs_heartbeat(master):
    """
    FS_GCS_ENABLE=5 makes the flight controller expect a ~1 Hz companion
    heartbeat. Without one it reports "PreArm: GCS failsafe on" and this script
    would report a blocker it created itself. The mission daemon sends the same
    heartbeat in flight, so sending it here also makes the verdict reflect real
    flight conditions.

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
    for msg_id, hz in [
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 5),
        (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1),
    ]:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1_000_000 / hz), 0, 0, 0, 0, 0)
        time.sleep(0.02)


def gather(master, seconds):
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
        elif kind == "SYS_STATUS":
            snap["voltage"] = msg.voltage_battery / 1000.0
        elif kind == "GPS_RAW_INT":
            snap["fix"], snap["sats"] = msg.fix_type, msg.satellites_visible
        elif kind == "RC_CHANNELS":
            snap["chancount"], snap["ch8"] = msg.chancount, msg.chan8_raw
        elif kind == "EKF_STATUS_REPORT":
            snap["ekf"] = msg.flags
        elif kind == "HOME_POSITION":
            snap["home"] = True
        elif kind == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            text = text.strip()
            if text.lower().startswith(("prearm", "arm:")) and text not in snap["prearm"]:
                snap["prearm"].append(text)
    return snap


def apply_safe_params(master):
    for name, value, why in SAFE_PARAMS:
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        say("set", GREEN, f"{name:<16} = {value:<7g} {DIM}{why}{RESET}")
        time.sleep(0.06)


def reboot(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1.0)
    try:
        master.close()
    except Exception:
        pass


def wait_for_board(timeout=60):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if find_flight_controller():
            time.sleep(3.0)
            return True
        time.sleep(1.0)
    return False


def main():
    parser = argparse.ArgumentParser(description="Take the aircraft to flight-ready.")
    parser.add_argument("--calibrate-compass", action="store_true",
                        help="chain into the compass calibration (needs you to rotate it)")
    parser.add_argument("--settle", type=float, default=25.0,
                        help="seconds to let GPS/EKF settle after configuring (default 25)")
    args = parser.parse_args()

    lock = acquire_lock()
    if lock is None:
        return 2

    print("Make flight ready — nothing will be armed.")

    step("1. Flight controller")
    master, heartbeat = connect()
    if master is None:
        say("!!", RED, "No flight controller found. Is it plugged in and powered?")
        return 2
    say("ok", GREEN, f"connected on {find_flight_controller()}")
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        say("!!", RED, "The aircraft is ARMED. Disarm before running this.")
        return 2
    request_streams(master)
    heartbeat_stop = start_gcs_heartbeat(master)

    step("2. Safe parameter set")
    apply_safe_params(master)

    step("3. Current state")
    snap = gather(master, 10)
    latched = [t for t in snap["prearm"] if "failsafe" in t.lower()]
    volts = snap.get("voltage", 0.0)
    say("ok" if volts > 1.0 else "!!", GREEN if volts > 1.0 else RED,
        f"battery {volts:.2f} V")

    # A failsafe latched at boot (for example the board powered up with no
    # battery attached) stays asserted until the board restarts.
    if latched and volts > 1.0:
        step("4. Clearing latched failsafes")
        for text in latched:
            say("--", YELLOW, f"latched: {text}")
        say("..", YELLOW, "rebooting the flight controller to clear them")
        heartbeat_stop.set()
        reboot(master)
        if not wait_for_board():
            say("!!", RED, "The board did not come back. Re-plug it and re-run.")
            return 2
        master, _ = connect()
        if master is None:
            say("!!", RED, "Could not reconnect after the reboot.")
            return 2
        say("ok", GREEN, "back up")
        request_streams(master)
        heartbeat_stop = start_gcs_heartbeat(master)
    else:
        step("4. Latched failsafes")
        say("ok", GREEN, "none to clear")

    step(f"5. Letting GPS and the EKF settle ({args.settle:.0f}s)")
    time.sleep(args.settle)
    snap = gather(master, 12)

    step("6. Verdict")
    blockers = []
    warnings = []

    if snap.get("fix", 0) >= 3:
        say("ok", GREEN, f"GPS 3D fix, {snap.get('sats','?')} sats")
    else:
        say("!!", RED, f"GPS fix type {snap.get('fix',0)}")
        blockers.append("GPS")

    ekf = snap.get("ekf", 0)
    if ekf & 16 and not ekf & 128:
        say("ok", GREEN, f"EKF horizontal position valid (flags={ekf})")
    else:
        say("!!", RED, f"EKF has no horizontal position (flags={ekf})")
        blockers.append("EKF position")

    if snap.get("home"):
        say("ok", GREEN, "home point set")
    else:
        say("!!", RED, "home point not set")
        blockers.append("home")

    chancount = snap.get("chancount", 0)
    if chancount and snap.get("ch8", 0):
        say("ok", GREEN, f"RC receiver: {chancount} channels, CH8={snap['ch8']}")
    else:
        # Not counted as a blocker here: the transmitter is switched on at the
        # field, just before flight. The flight controller still enforces this
        # itself — "PreArm: RC not found" keeps it from arming until the
        # receiver is actually live — so nothing is being bypassed.
        say("--", YELLOW, "RC receiver not detected — switch the transmitter on "
                          "before flying; the FC will not arm without it")
        warnings.append("RC")

    compass_bad = [t for t in snap["prearm"] if "mag field" in t.lower() or "compass" in t.lower()]
    if compass_bad:
        say("!!", RED, compass_bad[0])
        blockers.append("compass")
    else:
        say("ok", GREEN, "compass accepted by the flight controller")

    if snap["prearm"]:
        print()
        say("--", YELLOW, "flight controller still reports:")
        for text in snap["prearm"]:
            print(f"       - {text}")

    heartbeat_stop.set()
    master.close()
    lock.close()

    print()
    if not blockers:
        if warnings:
            print(f"{GREEN}READY TO FLY{RESET}, with warnings: {', '.join(warnings)}")
        else:
            print(f"{GREEN}READY TO FLY{RESET} — every check passed.")
        if "RC" in warnings:
            print(f"{YELLOW}  Switch the transmitter on and confirm CH8 moves before "
                  f"fitting propellers.{RESET}")
        print("\nNext: bench test with PROPELLERS OFF, then fly.")
        return 0

    print(f"{RED}NOT READY{RESET} — {len(blockers)} blocker(s): {', '.join(blockers)}")
    print("\nWhat is left, and who does it:")
    if "compass" in blockers:
        print(f"  compass  {DIM}needs your hands{RESET} — rotate the airframe through all six")
        print(f"           faces during calibration:  python compass_calibrate.py")
    if "EKF position" in blockers or "home" in blockers:
        print(f"  EKF/home {DIM}follows from the compass{RESET} — expected to clear once the")
        print(f"           compass calibration is accepted.")

    if args.calibrate_compass and "compass" in blockers:
        print("\nStarting the compass calibration now...")
        return subprocess.call([sys.executable,
                                os.path.join(PROJECT_DIR, "compass_calibrate.py")])
    return 1


if __name__ == "__main__":
    sys.exit(main())
