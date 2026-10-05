#!/usr/bin/env python3
"""
Onboard Compass Calibration
===========================
Drives ArduPilot's own compass calibration over MAVLink, so no ground station
(QGroundControl / Mission Planner) is needed on the Pi.

This is the same routine those apps trigger — MAV_CMD_DO_START_MAG_CAL — with
live progress and the fitness result printed here. It arms nothing and does not
touch the motors.

You still have to move the aircraft: hold it and rotate it slowly through every
face (nose down, nose up, left side down, right side down, upright, inverted),
turning a full circle in each. Take about a minute.

    systemctl stop ardupilot-mission.service
    python compass_calibrate.py
    systemctl start ardupilot-mission.service

Accepting the result writes the new offsets to the flight controller and reboots
it so they take effect. Exit status is 0 on a successful, accepted calibration.
"""

import argparse
import fcntl
import os
import sys
import time

from pymavlink import mavutil

SERIAL_BY_ID_DIR = "/dev/serial/by-id"
LOCK_FILE = os.getenv("ARDUPILOT_LOCK_FILE", "/tmp/ardupilot_mission.lock")
BAUD_RATE = int(os.getenv("ARDUPILOT_BAUD", "115200"))
TIMEOUT_S = float(os.getenv("MAGCAL_TIMEOUT_S", "300"))

# MAG_CAL_STATUS values from the MAVLink common dialect.
CAL_STATUS = {
    0: "NOT_STARTED", 1: "WAITING_TO_START", 2: "RUNNING_STEP_ONE",
    3: "RUNNING_STEP_TWO", 4: "SUCCESS", 5: "FAILED", 6: "BAD_ORIENTATION",
    7: "BAD_RADIUS",
}


def find_flight_controller():
    try:
        for name in sorted(os.listdir(SERIAL_BY_ID_DIR)):
            if "ArduPilot" in name or "MicoAir" in name:
                return os.path.join(SERIAL_BY_ID_DIR, name)
    except OSError:
        pass
    return None


def acquire_lock():
    """Only one process may own the serial link."""
    try:
        handle = open(LOCK_FILE, "w")
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return handle
    except OSError:
        print("The mission daemon is running and owns the serial link.\n"
              "Stop it first:  systemctl stop ardupilot-mission.service", file=sys.stderr)
        return None


def send(master, command, *params):
    values = list(params) + [0] * (7 - len(params))
    master.mav.command_long_send(master.target_system, master.target_component,
                                 command, 0, *values)


def main():
    parser = argparse.ArgumentParser(description="Calibrate the onboard compass.")
    parser.add_argument("-y", "--yes", action="store_true",
                        help="accept a successful calibration without prompting "
                             "(for running unattended while you rotate the airframe)")
    args = parser.parse_args()

    lock = acquire_lock()
    if lock is None:
        return 2

    port = os.getenv("ARDUPILOT_CONNECTION") or find_flight_controller()
    if port is None:
        print("No flight controller found under /dev/serial/by-id/.", file=sys.stderr)
        return 2

    master = mavutil.mavlink_connection(port, baud=BAUD_RATE, dialect="ardupilotmega")
    heartbeat = master.wait_heartbeat(timeout=10)
    if not heartbeat:
        print("No heartbeat from the flight controller.", file=sys.stderr)
        return 2
    if master.target_component in (0, mavutil.mavlink.MAV_COMP_ID_ALL):
        master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("The aircraft is ARMED. Disarm before calibrating.", file=sys.stderr)
        return 2

    print("Compass calibration — nothing will be armed.")
    print(f"Link: {port}\n")
    print("Rotate the aircraft slowly through all six faces, a full turn on each:")
    print("  nose down, nose up, left side down, right side down, upright, inverted.")
    print("Start moving it now. Ctrl-C to cancel.\n")

    # Cancel anything already running, then start fresh on all compasses.
    send(master, mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, 0)
    time.sleep(0.5)
    # params: mask(0=all), retry, autosave, delay, autoreboot
    send(master, mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL, 0, 1, 0, 0, 0)

    reports = {}
    last_line = ""
    deadline = time.time() + TIMEOUT_S
    try:
        while time.time() < deadline:
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            kind = msg.get_type()

            if kind == "MAG_CAL_PROGRESS":
                bar_len = 28
                filled = int(bar_len * msg.completion_pct / 100)
                line = (f"  compass {msg.compass_id}  "
                        f"[{'#' * filled}{'.' * (bar_len - filled)}] "
                        f"{msg.completion_pct:3d}%  "
                        f"{CAL_STATUS.get(msg.cal_status, msg.cal_status)}")
                if line != last_line:
                    print(f"\r{line}", end="", flush=True)
                    last_line = line

            elif kind == "MAG_CAL_REPORT":
                print()
                status = CAL_STATUS.get(msg.cal_status, msg.cal_status)
                reports[msg.compass_id] = msg
                print(f"  compass {msg.compass_id}: {status}  "
                      f"fitness={msg.fitness:.2f} "
                      f"offsets=({msg.ofs_x:+.1f}, {msg.ofs_y:+.1f}, {msg.ofs_z:+.1f})")

            elif kind == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
                text = text.strip()
                if any(k in text.lower() for k in ("mag", "compass", "calib")):
                    print(f"\n  [FC] {text}")
                    last_line = ""

            if reports and all(r.cal_status in (4, 5, 6, 7) for r in reports.values()):
                time.sleep(1.0)
                break
    except KeyboardInterrupt:
        print("\nCancelled. Reverting to the previous calibration.")
        send(master, mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, 0)
        master.close()
        return 130

    if not reports:
        print("\nNo calibration report received — the flight controller never started. "
              "Check that a compass is enabled (COMPASS_USE=1).", file=sys.stderr)
        send(master, mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, 0)
        master.close()
        return 1

    good = [c for c, r in reports.items() if r.cal_status == 4]
    bad = [c for c, r in reports.items() if r.cal_status != 4]
    print()
    for compass_id in bad:
        print(f"Compass {compass_id} FAILED: "
              f"{CAL_STATUS.get(reports[compass_id].cal_status)}")
    if not good:
        print("No compass calibrated successfully. Nothing was saved.")
        send(master, mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, 0)
        master.close()
        return 1

    # Fitness is the RMS residual in milligauss; lower is better.
    worst = max(reports[c].fitness for c in good)
    print(f"Calibrated {len(good)} compass(es). Worst fitness {worst:.2f} mGauss "
          f"({'good' if worst < 8 else 'marginal — consider recalibrating'}).")

    if args.yes:
        print("\n--yes: accepting automatically.")
    else:
        answer = input("\nAccept and save this calibration? The board will reboot. [y/N] ").strip().lower()
        if answer not in ("y", "yes"):
            print("Discarded. The previous calibration is still in place.")
            send(master, mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, 0)
            master.close()
            return 1

    send(master, mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL, 0)
    time.sleep(1.0)
    print("Accepted. Rebooting the flight controller...")
    send(master, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
    master.close()

    print("\nWait ~30s for it to come back, then confirm with:")
    print("    python preflight_check.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
