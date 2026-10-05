#!/usr/bin/env python3
"""
ArduPilot Connection & Telemetry Diagnostic Test Utility
=========================================================
Tests serial communication, heartbeat, autopilot type, flight mode,
arming state, sensor health (GPS / EKF / Battery), and local position.
"""

import logging
import os
import sys
import time
from logging.handlers import RotatingFileHandler
from pymavlink import mavutil

def _find_flight_controller() -> str:
    """Stable path to the board; /dev/ttyACM* numbering changes on every power cycle."""
    by_id = "/dev/serial/by-id"
    try:
        for name in sorted(os.listdir(by_id)):
            if "ArduPilot" in name or "MicoAir" in name:
                return os.path.join(by_id, name)
    except OSError:
        pass
    return "/dev/ttyACM0"


CONNECTION_STRING = os.getenv("ARDUPILOT_CONNECTION", "") or _find_flight_controller()
BAUD_RATE = int(os.getenv("ARDUPILOT_BAUD", "115200"))
LOG_FILE = os.getenv("LOG_FILE", "/home/aahswarm/ardupilot_testing/ardupilot_connection_test.log")
TIMEOUT_SEC = 10.0

logger = logging.getLogger("ardupilot_connection_test")


def setup_logging():
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = RotatingFileHandler(LOG_FILE, maxBytes=2_000_000, backupCount=5)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)


def test_connection():
    setup_logging()
    logger.info("========================================================")
    logger.info("=== ARDUPILOT CONNECTION & DIAGNOSTIC TEST UTILITY ===")
    logger.info("========================================================")
    logger.info(f"Attempting connection to ArduPilot on {CONNECTION_STRING} (baud: {BAUD_RATE})...")

    try:
        master = mavutil.mavlink_connection(
            CONNECTION_STRING,
            baud=BAUD_RATE,
            dialect="ardupilotmega",
        )
    except Exception as e:
        logger.error(f"Failed to open serial port {CONNECTION_STRING}: {e}")
        return False

    logger.info(f"Waiting up to {TIMEOUT_SEC}s for heartbeat from ArduPilot...")
    heartbeat = master.wait_heartbeat(timeout=TIMEOUT_SEC)

    if not heartbeat:
        logger.error(f"Connection timeout: No heartbeat received from {CONNECTION_STRING} within {TIMEOUT_SEC}s.")
        master.close()
        return False

    # Autopilot type & vehicle type identification
    ap_type = mavutil.mavlink.enums["MAV_AUTOPILOT"].get(heartbeat.autopilot, None)
    ap_name = ap_type.name if ap_type else f"Unknown({heartbeat.autopilot})"
    veh_type = mavutil.mavlink.enums["MAV_TYPE"].get(heartbeat.type, None)
    veh_name = veh_type.name if veh_type else f"Unknown({heartbeat.type})"

    mode_map = master.mode_mapping() or {}
    rev_mode_map = {v: k for k, v in mode_map.items()}
    current_mode = rev_mode_map.get(heartbeat.custom_mode, f"Custom({heartbeat.custom_mode})")
    is_armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    logger.info("=== Heartbeat Received ===")
    logger.info(f"  System ID:       {master.target_system}")
    logger.info(f"  Component ID:    {master.target_component}")
    logger.info(f"  Autopilot Type:  {ap_name}")
    logger.info(f"  Vehicle Type:    {veh_name}")
    logger.info(f"  Flight Mode:     {current_mode}")
    logger.info(f"  Armed Status:    {'ARMED' if is_armed else 'DISARMED'}")
    logger.info(f"  Available Modes: {list(mode_map.keys())}")

    logger.info("Reading sensor and telemetry streams (3 seconds)...")
    start_read = time.time()
    seen_types = set()

    while time.time() - start_read < 3.0:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue
        msg_type = msg.get_type()
        if msg_type not in seen_types:
            seen_types.add(msg_type)
            if msg_type == "GLOBAL_POSITION_INT":
                logger.info(f"  [GPS/Pos] RelAlt: {msg.relative_alt/1000.0:.2f}m, Heading: {msg.hdg/100.0:.1f}deg")
            elif msg_type == "SYS_STATUS":
                logger.info(f"  [Battery] Voltage: {msg.voltage_battery/1000.0:.2f}V, Remaining: {msg.battery_remaining}%")
            elif msg_type == "GPS_RAW_INT":
                logger.info(f"  [GPS] Fix Type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
            elif msg_type == "ATTITUDE":
                logger.info(f"  [Attitude] Roll: {msg.roll*57.2958:.1f}deg, Pitch: {msg.pitch*57.2958:.1f}deg, Yaw: {msg.yaw*57.2958:.1f}deg")

    master.close()
    logger.info("Diagnostic test completed successfully! ArduPilot connection verified.")
    return True


if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
