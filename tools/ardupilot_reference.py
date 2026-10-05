from logging.handlers import RotatingFileHandler
from pymavlink import mavutil
import logging
import sys
import time

CONNECTION_STRING = "/dev/ttyACM0"
BAUD = 115200
TAKEOFF_ALTITUDE_M = 3
HOVER_SECONDS = 10
MODE_WAIT_TIMEOUT = None
LOG_FILE = "/home/aahswarm/autonomousDrone/guided_auto_mission.log"

logger = logging.getLogger("guided_auto_mission")


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


def mode_name_from_heartbeat(master, heartbeat_msg):
    mode_mapping = master.mode_mapping() or {}
    reverse_mapping = {value: key for key, value in mode_mapping.items()}
    return reverse_mapping.get(
        heartbeat_msg.custom_mode, f"UNKNOWN({heartbeat_msg.custom_mode})"
    )


def wait_for_command_ack(master, expected_command, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if not msg:
            continue
        if msg.command == expected_command:
            logger.info(
                "COMMAND_ACK received for command=%s result=%s",
                msg.command,
                msg.result,
            )
            return msg
        logger.info(
            "Ignoring COMMAND_ACK for command=%s while waiting for command=%s",
            msg.command,
            expected_command,
        )
    logger.warning("Timed out waiting for COMMAND_ACK for command=%s", expected_command)
    return None


def send_takeoff_command(master, altitude_m):
    logger.info("Sending MAV_CMD_NAV_TAKEOFF to %.1f m", altitude_m)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude_m,
    )


def request_message_interval(master, message_name, frequency_hz):
    message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
    if message_id is None:
        logger.warning("Could not find MAVLink message id for %s", message_name)
        return

    interval_us = int(1_000_000 / frequency_hz)
    logger.info(
        "Requesting message stream %s at %s Hz (message id %s)",
        message_name,
        frequency_hz,
        message_id,
    )
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def wait_for_guided_mode_transition(master, timeout=None):
    logger.info("Waiting for a transmitter-driven mode change into GUIDED")
    start_time = time.time()
    last_mode = None
    seen_non_guided_mode = False

    while True:
        if timeout is not None and time.time() - start_time > timeout:
            raise TimeoutError("Timed out waiting for GUIDED mode")

        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not heartbeat:
            continue

        current_mode = mode_name_from_heartbeat(master, heartbeat)
        if current_mode != last_mode:
            logger.info("Current FC mode: %s", current_mode)
            last_mode = current_mode

        if current_mode != "GUIDED":
            seen_non_guided_mode = True
            continue

        if seen_non_guided_mode:
            logger.info("GUIDED mode transition detected, mission will start")
            return


def wait_for_relative_altitude(master, minimum_altitude_m, timeout=20):
    logger.info(
        "Waiting to reach at least %.1f m relative altitude", minimum_altitude_m
    )
    deadline = time.time() + timeout

    while time.time() < deadline:
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "ALTITUDE"],
            blocking=True,
            timeout=1,
        )
        if not msg:
            continue

        if msg.get_type() == "GLOBAL_POSITION_INT":
            relative_altitude_m = msg.relative_alt / 1000.0
        else:
            relative_altitude_m = msg.altitude_relative

        logger.info("Relative altitude: %.2f m", relative_altitude_m)
        if relative_altitude_m >= minimum_altitude_m:
            logger.info("Altitude threshold reached")
            return

    logger.warning("Altitude confirmation timed out, continuing mission")


def hover(seconds):
    logger.info("Hovering for %s seconds", seconds)
    for remaining in range(seconds, 0, -1):
        logger.info("Hover time remaining: %s s", remaining)
        time.sleep(1)


def main():
    setup_logging()
    logger.info("Mission watcher starting")
    logger.info(
        "Mission config: connection=%s baud=%s takeoff_altitude_m=%s hover_seconds=%s",
        CONNECTION_STRING,
        BAUD,
        TAKEOFF_ALTITUDE_M,
        HOVER_SECONDS,
    )

    logger.info("Connecting to flight controller")
    master = mavutil.mavlink_connection(
        CONNECTION_STRING,
        baud=BAUD,
        dialect="ardupilotmega",
    )

    logger.info("Waiting for heartbeat")
    master.wait_heartbeat(timeout=30)
    logger.info(
        "Connected to system=%s component=%s",
        master.target_system,
        master.target_component,
    )

    request_message_interval(master, "GLOBAL_POSITION_INT", 2)
    request_message_interval(master, "ALTITUDE", 2)

    wait_for_guided_mode_transition(master, timeout=MODE_WAIT_TIMEOUT)

    logger.info("Arming motors")
    master.arducopter_arm()
    master.motors_armed_wait()
    logger.info("Drone is armed")

    time.sleep(2)

    send_takeoff_command(master, TAKEOFF_ALTITUDE_M)
    wait_for_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    wait_for_relative_altitude(master, minimum_altitude_m=1.5)
    hover(HOVER_SECONDS)

    logger.info("Switching to LAND mode")
    land_mode = master.mode_mapping().get("LAND")
    if land_mode is None:
        raise RuntimeError("LAND mode is not available on this autopilot")

    master.set_mode(land_mode)
    logger.info("LAND command sent, waiting for motors to disarm")
    master.motors_disarmed_wait()
    logger.info("Mission complete: drone landed and disarmed")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.warning("Mission script interrupted by user")
        sys.exit(1)
    except Exception:
        logger.exception("Mission failed with an exception")
        sys.exit(1)
