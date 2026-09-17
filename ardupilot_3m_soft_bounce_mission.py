#!/usr/bin/env python3
"""
Three-metre horizontal geofence soft-bounce mission.

This is a mission variant of ``ardupilot_horizontal_geofence_mission.py`` and
deliberately reuses that known-good daemon's connection handling, telemetry,
CH8/manual takeover, takeoff, recording, failsafes, and soft landing.

Flight sequence:
  1. Take off and stabilize using the base mission.
  2. Capture the current local-NED position as the immutable origin.
  3. Command travel toward a point beyond the 3 m circular boundary.
  4. Ease off the outward velocity in a soft zone and reverse at 2.85 m,
     leaving 0.15 m for position/velocity error.
  5. Ramp the inward velocity up gently, return to the captured origin, stop,
     and let the base mission perform its ultra-slow soft landing.

The 3 m fence in this file is companion-controlled.  It is intentionally not
an ArduPilot FENCE_RADIUS breach: ArduPilot's breach actions can Brake/Land or
Land, but cannot perform this deterministic bounce, return, and soft landing.
The normal pilot takeover and all base-mission abort paths remain active.

Only run this script in a clear area after confirming the direction in which
the vehicle is pointing.  The outbound leg follows the captured takeoff yaw.
"""

import math
import os
import time

import ardupilot_horizontal_geofence_mission as mission


# The requested circular horizontal boundary, centred on the post-takeoff
# local-NED origin.  Defaults are intentionally conservative for a small test.
SOFT_GEOFENCE_RADIUS_M = float(os.getenv("SOFT_GEOFENCE_RADIUS_M", "3.0"))
INTENDED_TARGET_RADIUS_M = float(os.getenv("INTENDED_TARGET_RADIUS_M", "3.5"))
BOUNCE_TURNAROUND_RADIUS_M = float(os.getenv("BOUNCE_TURNAROUND_RADIUS_M", "2.85"))
BOUNCE_DECEL_START_RADIUS_M = float(os.getenv("BOUNCE_DECEL_START_RADIUS_M", "2.20"))
BOUNDARY_OVERSHOOT_ABORT_M = float(os.getenv("BOUNDARY_OVERSHOOT_ABORT_M", "0.20"))

OUTBOUND_SPEED_MPS = float(os.getenv("OUTBOUND_SPEED_MPS", "0.30"))
MIN_EDGE_SPEED_MPS = float(os.getenv("MIN_EDGE_SPEED_MPS", "0.04"))
RETURN_SPEED_MPS = float(os.getenv("RETURN_SPEED_MPS", "0.25"))
BOUNCE_REVERSE_RAMP_S = float(os.getenv("BOUNCE_REVERSE_RAMP_S", "2.0"))
BOUNCE_PAUSE_S = float(os.getenv("BOUNCE_PAUSE_S", "0.5"))
EDGE_POSITION_TOLERANCE_M = float(os.getenv("EDGE_POSITION_TOLERANCE_M", "0.03"))
MISSION_LEG_TIMEOUT_S = float(os.getenv("MISSION_LEG_TIMEOUT_S", "35.0"))

mission.HORIZONTAL_GEOFENCE_RADIUS_M = SOFT_GEOFENCE_RADIUS_M

# Start this mission when the aircraft is actually put into GUIDED flight mode
# rather than on a CH8 band change. CH8 keeps every other role it has: it must
# be in GUIDED for the mission to start when a receiver is present, and moving
# it to MANUAL or LAND still aborts and hands the aircraft back to the pilot.
mission.TRIGGER_ON_GUIDED_MODE = True


def _smoothstep(value: float) -> float:
    """Return a smooth 0..1 transition with zero slope at both ends."""
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _validate_configuration() -> None:
    if SOFT_GEOFENCE_RADIUS_M <= 0.0:
        raise ValueError("SOFT_GEOFENCE_RADIUS_M must be positive")
    if INTENDED_TARGET_RADIUS_M <= SOFT_GEOFENCE_RADIUS_M:
        raise ValueError("INTENDED_TARGET_RADIUS_M must be outside the soft geofence")
    if not 0.0 < BOUNCE_DECEL_START_RADIUS_M < BOUNCE_TURNAROUND_RADIUS_M:
        raise ValueError("bounce deceleration must start before the turnaround radius")
    if BOUNCE_TURNAROUND_RADIUS_M >= SOFT_GEOFENCE_RADIUS_M:
        raise ValueError("BOUNCE_TURNAROUND_RADIUS_M must remain inside the geofence")
    if min(OUTBOUND_SPEED_MPS, MIN_EDGE_SPEED_MPS, RETURN_SPEED_MPS) <= 0.0:
        raise ValueError("horizontal speeds must be positive")
    if BOUNCE_REVERSE_RAMP_S <= 0.0 or MISSION_LEG_TIMEOUT_S <= 0.0:
        raise ValueError("ramp and timeout values must be positive")


def _distance_from_origin(state, origin_north: float, origin_east: float) -> float:
    return math.hypot(
        state.local_north_m - origin_north,
        state.local_east_m - origin_east,
    )


def _soft_fence_safety_check(
    master,
    state,
    origin_north: float,
    origin_east: float,
    context: str,
) -> bool:
    """Apply base safety gates plus a last-resort 3 m overshoot guard."""
    abort_reason = mission.check_autonomous_abort(state, context)
    if abort_reason:
        mission.logger.warning(abort_reason)
        if state.chan8_state == "LAND" or state.fence_breached:
            mission.set_flight_mode(master, "LAND", state)
        return False

    if not mission.local_position_is_fresh(state):
        mission.logger.warning(
            f"[LOCAL POSITION STALE] No fresh LOCAL_POSITION_NED during {context}."
        )
        return False

    distance_m = _distance_from_origin(state, origin_north, origin_east)
    emergency_radius_m = SOFT_GEOFENCE_RADIUS_M + BOUNDARY_OVERSHOOT_ABORT_M
    if distance_m >= emergency_radius_m:
        mission.logger.error(
            f"[SOFT GEOFENCE OVERSHOOT] Distance={distance_m:.2f}m, "
            f"emergency limit={emergency_radius_m:.2f}m. Stopping; the base "
            "mission will enter its in-air safety landing path."
        )
        try:
            mission.send_local_ned_velocity(master, 0.0, 0.0, 0.0)
        except Exception as exc:
            mission.logger.error(f"Could not send emergency horizontal stop: {exc}")
        return False

    return True


def _send_horizontal_velocity(master, north_mps: float, east_mps: float) -> bool:
    try:
        mission.send_local_ned_velocity(master, north_mps, east_mps, 0.0)
        return True
    except Exception as exc:
        mission.logger.error(f"Could not send horizontal velocity setpoint: {exc}")
        return False


def fly_3m_soft_bounce(master, state) -> bool:
    """Attempt an outbound crossing, softly reverse, and return to the origin."""
    if not mission.local_position_is_fresh(state):
        mission.logger.warning(
            "Cannot capture soft-geofence origin: LOCAL_POSITION_NED is unavailable or stale."
        )
        return False

    origin_north = state.local_north_m
    origin_east = state.local_east_m
    outbound_heading_deg = state.heading_deg
    outbound_heading_rad = math.radians(outbound_heading_deg)
    unit_north = math.cos(outbound_heading_rad)
    unit_east = math.sin(outbound_heading_rad)

    mission.logger.info("=========================================================")
    mission.logger.info("=== STARTING 3 M HORIZONTAL GEOFENCE SOFT-BOUNCE TEST ===")
    mission.logger.info("=========================================================")
    mission.logger.info(
        f"Fixed origin: N={origin_north:.2f}m, E={origin_east:.2f}m; "
        f"outbound heading={outbound_heading_deg:.1f}deg."
    )
    mission.logger.info(
        f"Commanding toward an intended point at {INTENDED_TARGET_RADIUS_M:.2f}m, "
        f"outside the {SOFT_GEOFENCE_RADIUS_M:.2f}m boundary. Soft braking begins "
        f"at {BOUNCE_DECEL_START_RADIUS_M:.2f}m and reverses at "
        f"{BOUNCE_TURNAROUND_RADIUS_M:.2f}m."
    )

    interval_s = 1.0 / mission.STREAM_HZ
    outbound_deadline = time.monotonic() + MISSION_LEG_TIMEOUT_S
    last_log_time = 0.0

    # Outbound attempt.  The setpoint intention lies beyond the fence, while the
    # controller progressively removes outward speed before the actual boundary.
    while time.monotonic() < outbound_deadline:
        if not _soft_fence_safety_check(
            master, state, origin_north, origin_east, "soft-geofence outbound leg"
        ):
            return False

        radius_m = _distance_from_origin(state, origin_north, origin_east)
        if radius_m >= BOUNCE_TURNAROUND_RADIUS_M - EDGE_POSITION_TOLERANCE_M:
            break

        if radius_m <= BOUNCE_DECEL_START_RADIUS_M:
            speed_mps = OUTBOUND_SPEED_MPS
            phase = "outbound"
        else:
            soft_zone_width_m = (
                BOUNCE_TURNAROUND_RADIUS_M - BOUNCE_DECEL_START_RADIUS_M
            )
            remaining_fraction = (
                BOUNCE_TURNAROUND_RADIUS_M - radius_m
            ) / soft_zone_width_m
            blend = _smoothstep(remaining_fraction)
            speed_mps = MIN_EDGE_SPEED_MPS + (
                OUTBOUND_SPEED_MPS - MIN_EDGE_SPEED_MPS
            ) * blend
            phase = "soft braking"

        if not _send_horizontal_velocity(
            master, speed_mps * unit_north, speed_mps * unit_east
        ):
            return False

        if time.monotonic() - last_log_time >= 1.0:
            mission.logger.info(
                f"[SOFT BOUNCE] {phase}: radius={radius_m:.2f}m, "
                f"outward speed={speed_mps:.2f}m/s"
            )
            last_log_time = time.monotonic()
        time.sleep(interval_s)
    else:
        mission.logger.warning("Timed out before reaching the soft-bounce turnaround point.")
        _send_horizontal_velocity(master, 0.0, 0.0)
        return False

    radius_m = _distance_from_origin(state, origin_north, origin_east)
    mission.logger.info(
        f"[SOFT BOUNCE] Boundary response at radius={radius_m:.2f}m: "
        "outward command stopped; beginning gentle reversal."
    )

    # Briefly settle at the edge, continuously checking takeover and telemetry.
    pause_end = time.monotonic() + BOUNCE_PAUSE_S
    while time.monotonic() < pause_end:
        if not _soft_fence_safety_check(
            master, state, origin_north, origin_east, "soft-geofence edge pause"
        ):
            return False
        if not _send_horizontal_velocity(master, 0.0, 0.0):
            return False
        time.sleep(interval_s)

    # Smoothly ramp from zero to the inward return speed so the reversal does
    # not create a sharp acceleration step.
    ramp_start = time.monotonic()
    while True:
        elapsed_s = time.monotonic() - ramp_start
        if elapsed_s >= BOUNCE_REVERSE_RAMP_S:
            break
        if not _soft_fence_safety_check(
            master, state, origin_north, origin_east, "soft-geofence reversal"
        ):
            return False
        ramp = _smoothstep(elapsed_s / BOUNCE_REVERSE_RAMP_S)
        speed_mps = RETURN_SPEED_MPS * ramp
        if not _send_horizontal_velocity(
            master, -speed_mps * unit_north, -speed_mps * unit_east
        ):
            return False
        time.sleep(interval_s)

    # Home on the immutable origin.  Proportional slowing gives a gentle stop
    # and also corrects any cross-track drift accumulated during the bounce.
    mission.logger.info("[SOFT BOUNCE] Reversal complete; returning to fixed origin.")
    return_deadline = time.monotonic() + MISSION_LEG_TIMEOUT_S
    last_log_time = 0.0
    while time.monotonic() < return_deadline:
        if not _soft_fence_safety_check(
            master, state, origin_north, origin_east, "soft-geofence return"
        ):
            return False

        error_north = origin_north - state.local_north_m
        error_east = origin_east - state.local_east_m
        distance_m = math.hypot(error_north, error_east)
        if distance_m <= mission.LOCAL_POSITION_TOLERANCE_M:
            if not _send_horizontal_velocity(master, 0.0, 0.0):
                return False
            mission.logger.info(
                f"[SOFT BOUNCE] Origin reached within {distance_m:.2f}m; "
                "horizontal motion stopped."
            )
            return True

        speed_mps = min(RETURN_SPEED_MPS, max(0.04, 0.65 * distance_m))
        if not _send_horizontal_velocity(
            master,
            speed_mps * error_north / distance_m,
            speed_mps * error_east / distance_m,
        ):
            return False

        if time.monotonic() - last_log_time >= 1.0:
            mission.logger.info(
                f"[SOFT BOUNCE] returning: distance to origin={distance_m:.2f}m, "
                f"speed={speed_mps:.2f}m/s"
            )
            last_log_time = time.monotonic()
        time.sleep(interval_s)

    mission.logger.warning("Timed out while returning to the soft-geofence origin.")
    _send_horizontal_velocity(master, 0.0, 0.0)
    return False


def main() -> None:
    _validate_configuration()

    # The base ground mission resolves this symbol at runtime, so replacing it
    # changes only the horizontal demonstration and leaves every proven safety,
    # takeoff, landing, trigger, and reconnect path intact.
    mission.fly_horizontal_geofence_orbit = fly_3m_soft_bounce
    mission.main()


if __name__ == "__main__":
    main()
