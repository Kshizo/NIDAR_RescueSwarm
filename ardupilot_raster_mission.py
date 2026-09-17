#!/usr/bin/env python3
"""
Polygon raster (lawnmower) coverage mission.
===========================================

A mission variant of ``ardupilot_horizontal_geofence_mission.py``, built the same
way ``ardupilot_3m_soft_bounce_mission.py`` is built: it replaces ONLY the
horizontal flight leg and then calls the base mission, so the proven connection
handling, telemetry, CH8/manual takeover, takeoff, altitude control, recording,
failsafes, abort paths and ultra-slow soft landing are reused untouched.

    base mission (connection / telemetry / takeoff / landing / safety)
        -> raster flight-leg override          (fly_raster_pattern, below)
            -> polygon raster controller       (raster_plan_preview.plan_raster)

Flight sequence:
  1. Validate the mission polygon and build the raster path BEFORE connecting.
     An invalid polygon exits here, on the ground, with no MAVLink traffic at all.
  2. The base mission takes off and stabilises at TAKEOFF_ALTITUDE_M.
  3. Capture a runtime anchor: GPS latitude/longitude AND local-NED north/east
     from a near-simultaneous telemetry snapshot.
  4. Convert every planned waypoint from GPS into local NED using that anchor.
  5. Refuse to move if the transit, any waypoint, or the return would leave the
     horizontal geofence.
  6. TRANSIT_TO_RASTER -> RASTER -> RETURN_TO_ORIGIN (see below).
  7. Let the base mission land, unchanged.

THREE PHASES, BECAUSE THE TAKEOFF POINT IS NOT KNOWN
----------------------------------------------------
The aircraft is placed somewhere near the area, not on it, so the post-takeoff
hover origin is usually OUTSIDE the mission polygon. Requiring otherwise would
abort every real flight. The horizontal leg is therefore three explicit phases,
and exactly one safety check is phase-scoped:

    TRANSIT_TO_RASTER   origin -> first waypoint. Polygon containment NOT required.
    RASTER              the passes. Polygon containment REQUIRED, tested against
                        the ACTUAL aircraft position, every control iteration.
    RETURN_TO_ORIGIN    last waypoint -> origin. Polygon containment NOT required.

The outer geofence, telemetry freshness, EKF health, altitude band, battery floor
and every base abort check (CH8 takeover, CH8 LAND, RC loss, mode change, native
fence) are active in ALL THREE phases.

TWO INDEPENDENT GPS POLYGONS
----------------------------
The INNER polygon is where the aircraft is meant to fly. The OUTER polygon is
where it is allowed to be at all. They are not the same thing and this file does
not merge them:

  * INNER (A-D, ~57 m^2) constrains the PLAN offline
    (raster_plan_preview.verify_containment) and the AIRCRAFT in flight during the
    RASTER phase only, with a RASTER_POLYGON_BREACH_MARGIN_M excursion allowance.
  * OUTER (G1-G4, ~204 m^2) constrains the AIRCRAFT in EVERY phase, and the
    enforced boundary sits OUTER_GEOFENCE_MARGIN_M inside the supplied GPS
    corners. On breach: zero horizontal velocity, then the base module's own
    ``run_in_air_safety_land``.

Both are fixed to the ground. The circular geofence used until now was centred on
the post-takeoff hover origin and so moved with the aircraft; it is gone.

Neither polygon is derived from the other, and neither replaces the base module's
checks: ``raster_safety_check`` runs the same base functions the base
``horizontal_orbit_safety_check`` runs, in the same order, and adds to them.

NOT AN ARDUPILOT FENCE
----------------------
Both polygons are companion-side Python. CONFIGURE_FENCE stays 0 and no FENCE_*
parameter is written, so neither boundary survives loss of this process. The
pilot on the transmitter and the flight controller's own failsafes remain the
last line of defence.

WHY THE ANCHOR IS NEEDED
------------------------
Google Maps coordinates are WGS-84. The flight controller navigates in
LOCAL_POSITION_NED, whose origin is the EKF origin — wherever the autopilot
happened to acquire position, which is NOT the mission area and NOT the takeoff
point. Assuming they coincide puts the whole pattern tens of metres off. One
simultaneous observation of both frames fixes the offset exactly.

Only run this in a clear area, with the pilot on the transmitter and CH8 ready to
take over. The pattern leaves the takeoff point, so an abort lands the aircraft
wherever it is at that moment, not at home.
"""

import fcntl
import math
import os
import sys
import time

import ardupilot_horizontal_geofence_mission as mission
import fc_telemetry_logger
import raster_plan_preview as planner

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

BAR = "=" * 74
RULE = "-" * 74

# ---------------------------------------------------------------------------
# Keep every file this mission writes inside THIS project directory.
# ---------------------------------------------------------------------------
# The base module and video_recorder default to /home/aahswarm/ardupilot_testing,
# which is the frozen reference tree. Repointing the module attributes here means
# the development mission never writes into it, and the base module itself needs
# no edit.
mission.LOG_FILE = os.getenv("LOG_FILE", os.path.join(PROJECT_DIR, "ardupilot_raster_mission.log"))
mission.TRIGGER_FILE = os.getenv("MISSION_TRIGGER_FILE",
                                 os.path.join(PROJECT_DIR, "START_RASTER_MISSION"))
mission.video_recorder.RECORDINGS_DIR = os.getenv(
    "RECORDINGS_DIR", os.path.join(PROJECT_DIR, "recordings") + os.sep)

# ---------------------------------------------------------------------------
# The two polygons, both four Google Maps corners in perimeter order.
# ---------------------------------------------------------------------------
DEFAULT_CORNERS = planner.DEFAULT_CORNERS                # inner raster A->B->C->D
DEFAULT_OUTER_CORNERS = planner.DEFAULT_OUTER_CORNERS    # outer geofence G1->G4


def _parse_corners(text):
    parts = [chunk.strip() for chunk in text.split(";") if chunk.strip()]
    corners = []
    for part in parts:
        lat_text, lon_text = part.split(",")
        corners.append((float(lat_text), float(lon_text)))
    return tuple(corners)


_corners_env = os.getenv("RASTER_CORNERS", "")
RASTER_CORNERS = _parse_corners(_corners_env) if _corners_env else DEFAULT_CORNERS

_outer_env = os.getenv("OUTER_GEOFENCE_CORNERS", "")
OUTER_CORNERS = _parse_corners(_outer_env) if _outer_env else DEFAULT_OUTER_CORNERS

# Small first-test values, matching raster_plan_preview.
RASTER_PASS_SPACING_M = float(os.getenv("RASTER_PASS_SPACING_M", "0.40"))
RASTER_EDGE_INSET_M = float(os.getenv("EDGE_INSET_M", os.getenv("RASTER_EDGE_INSET_M", "0.20")))
RASTER_SPEED_MPS = float(os.getenv("RASTER_SPEED_MPS", "0.20"))

# Position tolerances. Intermediate pass endpoints are deliberately looser than
# the base 0.15 m: holding 0.15 m at every turn makes the aircraft crawl while it
# settles, and the extra precision buys nothing mid-pattern. The final return to
# the captured origin uses the base tolerance.
#
# 0.20 m: the passes on this area are ~4 m long and 0.39 m apart, so a 0.30 m
# acceptance radius would be comparable to the pass spacing itself.
RASTER_WAYPOINT_TOLERANCE_M = float(os.getenv("RASTER_WAYPOINT_TOLERANCE_M", "0.20"))
RASTER_FINAL_TOLERANCE_M = float(os.getenv("RASTER_FINAL_TOLERANCE_M",
                                           str(mission.LOCAL_POSITION_TOLERANCE_M)))

# Anchor quality gates.
RASTER_GPS_MAX_AGE_S = float(os.getenv("RASTER_GPS_MAX_AGE_S", "1.0"))
RASTER_ANCHOR_MAX_SKEW_S = float(os.getenv("RASTER_ANCHOR_MAX_SKEW_S", "0.35"))
RASTER_MIN_GPS_FIX_TYPE = int(os.getenv("RASTER_MIN_GPS_FIX_TYPE", "3"))

# Whole-flight horizontal budget, covering transit + pattern + return. Checked
# between waypoints, and also checked against the PLANNED route before launch
# (see check_time_budget) so an over-long pattern is refused on the ground rather
# than cut off part-way down a pass.
RASTER_TOTAL_TIMEOUT_S = float(os.getenv("RASTER_TOTAL_TIMEOUT_S", "180.0"))
# Multiplier applied to the planned straight-line time when checking the budget.
# Real flight is slower than path_length/speed: the controller eases off near
# every waypoint, and each turn costs settling time.
RASTER_TIME_CONTINGENCY = float(os.getenv("RASTER_TIME_CONTINGENCY", "1.35"))

# --- boundary 2 of 2: the OUTER GPS SAFETY GEOFENCE ---------------------------
# How far INSIDE the supplied outer polygon the enforced boundary sits. The GPS
# coordinates themselves are never altered; this margin is applied to them, so the
# aircraft is stopped before it reaches the real edge rather than after.
#
# Nothing reduces this to make geometry fit. If the inner area or the transit does
# not fit inside the outer polygon minus this margin, the mission refuses and
# prints the arithmetic.
OUTER_GEOFENCE_MARGIN_M = float(os.getenv("OUTER_GEOFENCE_MARGIN_M", "0.50"))
# How finely route segments are sampled when checking them against the outer
# polygon before launch. 0.05 m matches the planner's own containment sampling.
ROUTE_SAMPLE_STEP_M = float(os.getenv("ROUTE_SAMPLE_STEP_M", "0.05"))

# NOTE ON THE RETIRED CIRCULAR FENCE
# ----------------------------------
# Until now the outer boundary was a circle of HORIZONTAL_GEOFENCE_RADIUS_M
# centred on the post-takeoff hover origin. It is gone. A circle centred on the
# aircraft's own start point moves with the aircraft, so it constrains distance
# travelled rather than where the aircraft may be — which is not what a
# containment boundary is for. The outer polygon above is fixed to the ground.
#
# The base module still defines HORIZONTAL_GEOFENCE_RADIUS_M and still uses it
# inside horizontal_orbit_safety_check and run_horizontal_geofence_breach_land.
# This mission calls neither, so that constant is inert here; it is left alone
# rather than edited, because the base module is not ours to change.

# ---------------------------------------------------------------------------
# Mission phases
# ---------------------------------------------------------------------------
# The physical takeoff point is not known in advance: the aircraft is placed
# somewhere near the area, not on it. So the flight is three explicit phases, and
# the mission-polygon containment check applies to exactly one of them.
#
#   TRANSIT_TO_RASTER  origin -> first raster waypoint. The aircraft is expected
#                      to be outside the polygon here; requiring containment
#                      would abort every flight that did not take off inside the
#                      area.
#   RASTER             the passes themselves. The aircraft is inside the polygon
#                      and must stay there.
#   RETURN_TO_ORIGIN   last waypoint -> origin, then the base soft landing. The
#                      aircraft leaves the polygon again on purpose.
#
# The outer geofence, telemetry freshness, EKF, altitude, battery and every base
# abort check apply in ALL three phases. Only the polygon check is phase-scoped.
PHASE_TRANSIT = "TRANSIT_TO_RASTER"
PHASE_RASTER = "RASTER"
PHASE_RETURN = "RETURN_TO_ORIGIN"
POLYGON_ENFORCED_PHASES = (PHASE_RASTER,)

# --- boundary 1 of 2: the mission polygon, checked against the ACTUAL aircraft --
# How far the aircraft may stray outside the mission polygon before the pattern is
# aborted. This is a containment check on the real position, not on the plan: the
# planner already proves the PATH is inside the polygon, which says nothing about
# where the aircraft actually ends up when wind, overshoot, or a slow controller
# push it off the line.
#
# Enforced during the RASTER phase only — see the phase notes below. During
# transit and return the aircraft is deliberately outside the polygon.
#
# The default has to clear normal tracking error without being slack. On this
# small area the path is inset 0.20 m from the edge and waypoints are accepted at
# 0.20 m, so an on-plan aircraft can already sit on the polygon edge at a turn.
# 0.50 m of excursion beyond the edge covers that plus ordinary overshoot. The
# outer geofence sits at least 1.51 m beyond the inner edge on this area, and its
# own 0.50 m inward margin takes 0.50 m of that, so a full 0.50 m inner excursion
# still leaves roughly 0.51 m before the enforced outer boundary. That separation
# is checked numerically at plan time, not assumed — see check_boundary_separation.
RASTER_POLYGON_BREACH_MARGIN_M = float(os.getenv("RASTER_POLYGON_BREACH_MARGIN_M", "0.50"))

# --- operating altitude for THIS mission -------------------------------------
# 2.0 m AGL instead of the base 1.2 m: the test field has low obstacles and 2 m
# gives clearance over them. Applied by setting the base module's own constant
# below, so the proven send_takeoff_command / wait_for_climb path performs the
# takeoff exactly as before and only its target changes.
RASTER_TAKEOFF_ALTITUDE_M = float(os.getenv("TAKEOFF_ALTITUDE_M", "2.0"))

# --- altitude band held during the HORIZONTAL phases -------------------------
# The base mission monitors altitude during the climb and during the landing, but
# nothing watches it across the horizontal leg — and that leg is minutes long.
#
# This band is NOT applied during the climb. It is only ever evaluated inside
# raster_safety_check, which runs from the first horizontal setpoint onward, i.e.
# after wait_for_climb has already confirmed the aircraft is at target altitude
# and stable. A climb through 0.4 m or 1.4 m therefore cannot trip it.
RASTER_ALT_MIN_M = float(os.getenv("RASTER_ALT_MIN_M", "1.50"))
RASTER_ALT_MAX_M = float(os.getenv("RASTER_ALT_MAX_M", "2.50"))

# --- companion-side endurance gate -------------------------------------------
# The flight controller already lands on its own low/critical battery failsafe
# (BATT_FS_LOW_ACT / BATT_FS_CRT_ACT, set by the base module). This is the
# companion's own earlier gate, so a long pattern ends in a controlled return
# rather than an autopilot failsafe part-way down a pass. Only enforced when the
# flight controller is actually reporting the value.
RASTER_MIN_BATTERY_PERCENT = float(os.getenv("RASTER_MIN_BATTERY_PERCENT", "25.0"))
RASTER_MIN_BATTERY_VOLTAGE_V = float(os.getenv("RASTER_MIN_BATTERY_VOLTAGE_V", "0.0"))

# --- telemetry-only dry run ---------------------------------------------------
# See run_dry_run(). Deliberately a separate, non-flight connection path.
RASTER_DRY_RUN = os.getenv("RASTER_DRY_RUN", "0") == "1"
DRY_RUN_TIMEOUT_S = float(os.getenv("RASTER_DRY_RUN_TIMEOUT_S", "40.0"))
DRY_RUN_SETTLE_S = float(os.getenv("RASTER_DRY_RUN_SETTLE_S", "3.0"))
DRY_RUN_REQUIRE_EKF = os.getenv("RASTER_DRY_RUN_REQUIRE_EKF", "1") == "1"

# --- base-module parameters this mission sets -------------------------------
# Same idiom the soft-bounce reference uses: configure the base module, never edit
# it.
#
# TAKEOFF_ALTITUDE_M is the real one. The base run_ground_autonomous_mission reads
# it at call time for both send_takeoff_command and wait_for_climb, so setting it
# here moves this mission to 2.0 m while leaving the base takeoff implementation,
# the soft-bounce mission and the frozen tree untouched.
mission.TAKEOFF_ALTITUDE_M = RASTER_TAKEOFF_ALTITUDE_M
# Cosmetic only: makes the base daemon's startup banner report the speed actually
# flown instead of the orbit default.
mission.ORBIT_SPEED_MPS = RASTER_SPEED_MPS

# Start the mission when the aircraft is actually put into GUIDED flight mode,
# matching the proven soft-bounce reference. CH8 keeps every other role: it must
# be GUIDED for the mission to start when a receiver is present, and moving it to
# MANUAL or LAND still aborts and hands the aircraft back to the pilot.
mission.TRIGGER_ON_GUIDED_MODE = os.getenv("TRIGGER_ON_GUIDED_MODE", "1") == "1"


# ---------------------------------------------------------------------------
# Runtime GPS / local-NED anchor
# ---------------------------------------------------------------------------
class _Snapshot:
    """Latest GPS and local-NED observations, each with its own arrival time."""

    def __init__(self):
        self.lat = 0.0
        self.lon = 0.0
        self.gps_time = 0.0
        self.north_m = 0.0
        self.east_m = 0.0
        self.local_time = 0.0


_snapshot = _Snapshot()


class Anchor:
    """One near-simultaneous observation tying WGS-84 to LOCAL_POSITION_NED."""

    def __init__(self, lat, lon, north_m, east_m, timestamp, skew_s):
        self.lat = lat
        self.lon = lon
        self.north_m = north_m
        self.east_m = east_m
        self.timestamp = timestamp
        self.skew_s = skew_s

    def to_local(self, lat, lon):
        """Converts a WGS-84 point into absolute local-NED north/east."""
        delta = planner.gps_to_ne(planner.GpsPoint(lat, lon),
                                  planner.GpsPoint(self.lat, self.lon))
        return self.north_m + delta.north, self.east_m + delta.east


def _message_hook(connection, msg):
    """
    Records latitude/longitude and local NED as messages arrive.

    The base TelemetryState keeps neither latitude nor longitude, and the base
    module must not be modified. A pymavlink message hook runs inside the existing
    single MAVLink receive path (the telemetry listener thread), so this adds no
    second reader on the serial link — which would corrupt the stream.
    """
    try:
        msg_type = msg.get_type()
        if msg_type == "GLOBAL_POSITION_INT":
            if msg.get_srcSystem() != connection.target_system:
                return
            _snapshot.lat = msg.lat / 1e7
            _snapshot.lon = msg.lon / 1e7
            _snapshot.gps_time = time.monotonic()
        elif msg_type == "LOCAL_POSITION_NED":
            if msg.get_srcSystem() != connection.target_system:
                return
            # Same reasoning as the base telemetry listener: a NaN frame (EKF in
            # CONST_POS_MODE) is not a position. Dropping it leaves local_time
            # stale, so capture_anchor() refuses rather than baking a NaN into
            # every waypoint through the anchor.
            if math.isfinite(msg.x) and math.isfinite(msg.y):
                _snapshot.north_m = msg.x
                _snapshot.east_m = msg.y
                _snapshot.local_time = time.monotonic()
    except Exception as exc:                       # never break the receive path
        mission.logger.debug(f"[RASTER] snapshot hook error: {exc}")


_base_configure_streams = mission.configure_ardupilot_message_streams


def _configure_streams_with_hook(master):
    """Requests the base streams, then attaches the snapshot hook to this link."""
    _base_configure_streams(master)
    if _message_hook not in master.message_hooks:
        master.message_hooks.append(_message_hook)
        mission.logger.info(
            "[RASTER] Position snapshot hook attached to the existing MAVLink "
            "receive path (no additional reader).")
    # Verbose flight-controller log. Same receive path, same reasoning: a second
    # MAVLink reader on this serial link would corrupt the stream, so the logger
    # is a hook, not a connection. Its disk writes happen on its own thread.
    try:
        fc_telemetry_logger.attach(master, mission.logger)
    except Exception as exc:                       # logging must never stop a flight
        mission.logger.warning(f"[FC LOG] could not attach verbose FC logger: {exc!r}")


mission.configure_ardupilot_message_streams = _configure_streams_with_hook


def capture_anchor(state):
    """
    Returns an :class:`Anchor`, or None with the reason logged.

    Requires a 3D fix, fresh GPS, fresh local position, and the two observations
    to be close together in time — a stale pairing would bake a position error
    straight into every waypoint.
    """
    now = time.monotonic()

    if state.gps_fix_type < RASTER_MIN_GPS_FIX_TYPE:
        mission.logger.warning(
            f"[RASTER ANCHOR] GPS fix type {state.gps_fix_type} is below the required "
            f"{RASTER_MIN_GPS_FIX_TYPE} (sats={state.satellites_visible}).")
        return None

    if _snapshot.gps_time <= 0.0 or now - _snapshot.gps_time > RASTER_GPS_MAX_AGE_S:
        age = now - _snapshot.gps_time if _snapshot.gps_time > 0 else float("inf")
        mission.logger.warning(
            f"[RASTER ANCHOR] No fresh GLOBAL_POSITION_INT (age={age:.2f}s, "
            f"limit={RASTER_GPS_MAX_AGE_S:.2f}s).")
        return None

    if not mission.local_position_is_fresh(state):
        mission.logger.warning("[RASTER ANCHOR] LOCAL_POSITION_NED is unavailable or stale.")
        return None

    skew = abs(_snapshot.gps_time - _snapshot.local_time)
    if skew > RASTER_ANCHOR_MAX_SKEW_S:
        mission.logger.warning(
            f"[RASTER ANCHOR] GPS and local-position samples are {skew:.3f}s apart, "
            f"above the {RASTER_ANCHOR_MAX_SKEW_S:.3f}s limit. Not anchoring on a "
            "mismatched pair.")
        return None

    anchor = Anchor(_snapshot.lat, _snapshot.lon, _snapshot.north_m, _snapshot.east_m,
                    now, skew)
    mission.logger.info(
        f"[RASTER ANCHOR] GPS {anchor.lat:.7f}, {anchor.lon:.7f} <-> local NED "
        f"N={anchor.north_m:+.2f}m E={anchor.east_m:+.2f}m (samples {skew * 1000.0:.0f}ms apart, "
        f"fix={state.gps_fix_type}, sats={state.satellites_visible}).")
    return anchor


# ---------------------------------------------------------------------------
# Planning and pre-flight validation
# ---------------------------------------------------------------------------
def build_plan():
    """
    Builds and verifies the raster plan. Raises planner.PlanError on any problem.

    This is anchor-independent: it is pure geometry relative to inner corner A,
    so it runs on the ground before the flight controller is even contacted. Both
    polygons are validated here, and the inner one is proved to sit inside the
    outer one, so a mistyped corner never reaches the aircraft.
    """
    outer_problems = planner.validate_outer_polygon(OUTER_CORNERS)
    if outer_problems:
        raise planner.PlanError("the outer safety geofence is not usable", outer_problems)

    plan = planner.plan_raster(RASTER_CORNERS, RASTER_PASS_SPACING_M,
                               RASTER_EDGE_INSET_M, RASTER_SPEED_MPS)
    problems, metrics = planner.verify_containment(plan)
    if problems:
        raise planner.PlanError("the generated raster path is not contained by the "
                                "mission polygon", problems)

    nested, nest_metrics = planner.verify_inner_inside_outer(
        plan, OUTER_CORNERS, OUTER_GEOFENCE_MARGIN_M)
    if nested:
        raise planner.PlanError(
            "the inner raster polygon is not safely contained by the outer safety "
            "geofence", nested)
    metrics["outer"] = nest_metrics
    return plan, metrics


def log_plan(plan, metrics):
    log = mission.logger.info
    log("=========================================================")
    log("=== POLYGON RASTER MISSION PLAN ===")
    log("=========================================================")
    for corner in plan.corners:
        log(f"  Corner {corner.name}: {corner.lat:.7f}, {corner.lon:.7f}")
    m = plan.measurements
    log(f"  Sides: AB={m['ab']:.2f}m BC={m['bc']:.2f}m CD={m['cd']:.2f}m DA={m['da']:.2f}m; "
        f"area={m['area']:.2f}m^2")
    log(f"  Corner angles: A={m['angle_a']:.1f} B={m['angle_b']:.1f} "
        f"C={m['angle_c']:.1f} D={m['angle_d']:.1f} deg (a rectangle is not required)")
    log(f"  Inward safety inset: {plan.inset_m:.2f}m -> inset area "
        f"{abs(planner.signed_area(plan.inset_uv)):.2f}m^2")
    log(f"  Raster direction: A->B, bearing "
        f"{math.degrees(math.atan2(plan.u_hat[1], plan.u_hat[0])) % 360.0:.1f} deg, alternating")
    log(f"  Passes: {plan.pass_count}, spacing {plan.pass_spacing_actual_m:.3f}m "
        f"(requested {plan.pass_spacing_requested_m:.3f}m), waypoints {len(plan.waypoints)}")
    log(f"  Path length {plan.path_length_m:.1f}m, estimated "
        f"{plan.estimated_time_s:.0f}s at {plan.speed_mps:.2f}m/s "
        f"(horizontal travel only)")
    log(f"  Containment: {metrics['waypoints_inside_inset']}/{metrics['waypoints']} waypoints and "
        f"{metrics['segments']} segments verified inside the inset polygon "
        f"({metrics['segment_samples']} samples); worst clearance "
        f"{metrics['min_segment_polygon_clearance_m']:+.3f}m from the polygon edge")
    outer_m = metrics.get("outer")
    if outer_m:
        log(f"  Outer safety geofence: area {outer_m['outer_area_m2']:.2f}m^2, sides "
            + ", ".join(f"{v:.2f}m" for v in outer_m['outer_sides_m'])
            + f", winding {outer_m['outer_winding']}")
        log(f"  Inner inside outer: minimum separation "
            f"{outer_m['min_edge_clearance_m']:.2f}m (corners "
            + ", ".join(f"{k}={v:.2f}m" for k, v in outer_m['corner_clearance_m'].items())
            + f"); raster path worst {outer_m['min_path_clearance_m']:.2f}m, "
            f"{outer_m['min_path_clearance_effective_m']:.2f}m against the effective boundary")
    log(f"  Outer safety geofence: {len(OUTER_CORNERS)} GPS corners, effective boundary "
        f"{OUTER_GEOFENCE_MARGIN_M:.2f}m inside them "
        "(independent safety layer, NOT the mission polygon, NOT an ArduPilot fence)")
    log(f"  Inner runtime breach tolerance: {RASTER_POLYGON_BREACH_MARGIN_M:.2f}m beyond the "
        "mission-polygon edge, enforced during the RASTER phase only")
    log(f"  Max planned distance from polygon centroid: "
        f"{metrics['max_dist_from_centroid_m']:.2f}m")


def validate_configuration():
    """Checks the flight parameters. Raises ValueError on anything unusable."""
    if RASTER_SPEED_MPS <= 0.0:
        raise ValueError("RASTER_SPEED_MPS must be positive")
    if RASTER_WAYPOINT_TOLERANCE_M <= 0.0 or RASTER_FINAL_TOLERANCE_M <= 0.0:
        raise ValueError("position tolerances must be positive")
    if RASTER_POLYGON_BREACH_MARGIN_M < 0.0:
        raise ValueError("RASTER_POLYGON_BREACH_MARGIN_M cannot be negative")
    if OUTER_GEOFENCE_MARGIN_M < 0.0:
        raise ValueError("OUTER_GEOFENCE_MARGIN_M cannot be negative")
    if RASTER_TOTAL_TIMEOUT_S <= 0.0:
        raise ValueError("RASTER_TOTAL_TIMEOUT_S must be positive")
    if RASTER_TIME_CONTINGENCY < 1.0:
        raise ValueError("RASTER_TIME_CONTINGENCY must be at least 1.0")
    if not RASTER_ALT_MIN_M < RASTER_ALT_MAX_M:
        raise ValueError("RASTER_ALT_MIN_M must be below RASTER_ALT_MAX_M")
    if not RASTER_ALT_MIN_M < RASTER_TAKEOFF_ALTITUDE_M < RASTER_ALT_MAX_M:
        raise ValueError(
            f"the horizontal altitude band {RASTER_ALT_MIN_M:.2f}-{RASTER_ALT_MAX_M:.2f}m "
            f"does not contain the {RASTER_TAKEOFF_ALTITUDE_M:.2f}m target altitude; the "
            "mission would abort the moment it started flying horizontally")

    problems = planner.validate_outer_polygon(OUTER_CORNERS)
    if problems:
        raise ValueError("the outer safety geofence is unusable: " + "; ".join(problems))


# ---------------------------------------------------------------------------
# Route feasibility against the whole-flight time budget
# ---------------------------------------------------------------------------
def estimate_route(plan, transit_m, return_m):
    """Straight-line distance and time for transit + pattern + return."""
    total_m = transit_m + plan.path_length_m + return_m
    straight_s = total_m / RASTER_SPEED_MPS if RASTER_SPEED_MPS > 0 else float("inf")
    return {
        "transit_m": transit_m,
        "pattern_m": plan.path_length_m,
        "return_m": return_m,
        "total_m": total_m,
        "straight_s": straight_s,
        "expected_s": straight_s * RASTER_TIME_CONTINGENCY,
    }


def check_time_budget(estimate, context="planned route"):
    """
    Refuses a route that cannot finish inside RASTER_TOTAL_TIMEOUT_S.

    Checked on the ground, before any horizontal command. The in-flight budget
    check still exists, but it fires between waypoints and leaves the aircraft
    part-way down a pass; this one prevents the flight instead.

    Nothing here raises the budget, the speed or the pass spacing to make a route
    fit. Those are operator decisions and the arithmetic is printed so they can be
    made from real numbers.
    """
    ok = estimate["expected_s"] <= RASTER_TOTAL_TIMEOUT_S
    line = mission.logger.info if ok else mission.logger.error
    line(f"[RASTER BUDGET] {context}: transit {estimate['transit_m']:.1f}m + pattern "
         f"{estimate['pattern_m']:.1f}m + return {estimate['return_m']:.1f}m = "
         f"{estimate['total_m']:.1f}m")
    line(f"[RASTER BUDGET] {estimate['straight_s']:.0f}s straight-line at "
         f"{RASTER_SPEED_MPS:.2f}m/s, x{RASTER_TIME_CONTINGENCY:.2f} contingency = "
         f"{estimate['expected_s']:.0f}s expected, against a "
         f"{RASTER_TOTAL_TIMEOUT_S:.0f}s budget "
         f"({RASTER_TOTAL_TIMEOUT_S - estimate['expected_s']:+.0f}s).")
    if not ok:
        mission.logger.error(
            "[RASTER BUDGET] REFUSING TO FLY: the planned horizontal route cannot "
            f"finish inside the budget. Over by "
            f"{estimate['expected_s'] - RASTER_TOTAL_TIMEOUT_S:.0f}s.")
        mission.logger.error(
            f"    What drives it: {estimate['pattern_m']:.1f}m of pattern at "
            f"{RASTER_PASS_SPACING_M:.2f}m spacing over a "
            f"{RASTER_SPEED_MPS:.2f}m/s speed cap.")
        mission.logger.error(
            "    Nothing is adjusted automatically. Raising RASTER_TOTAL_TIMEOUT_S, "
            "RASTER_SPEED_MPS or RASTER_PASS_SPACING_M is a deliberate decision, and "
            "the endurance of the aircraft, not this number, is the real limit.")
    return ok


# ---------------------------------------------------------------------------
# Whole-route containment against the effective outer geofence
# ---------------------------------------------------------------------------
def check_route_inside_outer(outer, route, context="planned route"):
    """
    Refuses the flight if any part of the route leaves the effective outer polygon.

    ``route`` is an ordered list of (label, north, east). Every SEGMENT between
    consecutive points is sampled, not just the points: the outer boundary is a
    polygon now, and for a general polygon a straight line between two interior
    points can leave and re-enter. (For the convex polygon actually configured it
    cannot — but the sampling does not depend on that being true, so a future
    non-convex boundary cannot silently pass.)
    """
    offenders = []
    worst = float("inf")
    worst_label = ""
    samples = 0

    for index, (label, north_m, east_m) in enumerate(route):
        c = outer.clearance_m(north_m, east_m)
        samples += 1
        if c < worst:
            worst, worst_label = c, label
        if c < outer.required_clearance_m:
            offenders.append((label, c))

    for index in range(len(route) - 1):
        (la, na, ea), (lb, nb, eb) = route[index], route[index + 1]
        length = math.hypot(nb - na, eb - ea)
        steps = max(2, int(length / ROUTE_SAMPLE_STEP_M) + 1)
        for k in range(1, steps):
            t = k / steps
            q_n, q_e = na + t * (nb - na), ea + t * (eb - ea)
            c = outer.clearance_m(q_n, q_e)
            samples += 1
            if c < worst:
                worst, worst_label = c, f"{la} -> {lb} (at {t * 100:.0f}%)"
            if c < outer.required_clearance_m:
                offenders.append((f"{la} -> {lb} (at {t * 100:.0f}%)", c))

    mission.logger.info(
        f"[OUTER GEOFENCE CHECK] {context}: {len(route)} points and "
        f"{len(route) - 1} segments, {samples} samples. Worst clearance to the raw "
        f"outer polygon is {worst:+.2f}m at {worst_label}; the enforced boundary is "
        f"{OUTER_GEOFENCE_MARGIN_M:.2f}m inside it, leaving {worst - outer.required_clearance_m:+.2f}m.")

    if offenders:
        mission.logger.error(
            f"[OUTER GEOFENCE CHECK] REFUSING TO FLY: {len(offenders)} sampled point(s) "
            f"on the route lie outside the effective outer geofence.")
        for label, c in offenders[:8]:
            mission.logger.error(
                f"    - {label}: clearance {c:+.2f}m, needs >= {outer.required_clearance_m:+.2f}m")
        mission.logger.error(
            "    The outer GPS polygon and its 0.50 m margin are NOT adjusted "
            "automatically. Move the takeoff point, or supply a different outer "
            "polygon deliberately. No horizontal command has been sent.")
        return False
    return True


# ---------------------------------------------------------------------------
# Boundary 1 of 2: the mission polygon, in absolute local NED
# ---------------------------------------------------------------------------
class MissionBoundary:
    """
    One convex polygon in the aircraft's own local-NED frame, plus how close to
    its edge the aircraft is allowed to get.

    Both boundaries use this class, and the ONLY difference between them is the
    sign of ``required_clearance_m``:

      inner mission polygon : required = -RASTER_POLYGON_BREACH_MARGIN_M
                              negative, so the aircraft may stray that far OUTSIDE
                              the edge before it counts as a breach.
      outer safety geofence : required = +OUTER_GEOFENCE_MARGIN_M
                              positive, so the aircraft must stay that far INSIDE
                              the edge; the enforced boundary is the supplied GPS
                              polygon shrunk inward.

    Convexity is guaranteed upstream — planner.validate_polygon for the inner ring
    and planner.validate_outer_polygon for the outer one both reject a non-convex
    or inconsistently wound corner order before any of this runs — so the
    half-plane test is exact, and it reuses the planner's own edge_halfplanes /
    clearance rather than a second implementation.
    """

    def __init__(self, corners_ne, required_clearance_m, name="boundary"):
        # edge_halfplanes needs counter-clockwise winding to produce inward
        # normals. Sign here is in (north, east) ordering, which is left-handed
        # relative to the usual (x, y); rather than reason about that, just
        # measure the winding and flip it if needed.
        ring = list(corners_ne)
        if planner.signed_area(ring) < 0.0:
            ring.reverse()
        self.ring = tuple(ring)
        self.planes = planner.edge_halfplanes(self.ring)
        self.required_clearance_m = required_clearance_m
        self.name = name

    def clearance_m(self, north_m, east_m):
        """Signed distance to the polygon edge: positive inside, negative outside."""
        return planner.clearance((north_m, east_m), self.planes)

    def contains(self, north_m, east_m):
        """True while the aircraft satisfies this boundary's required clearance."""
        return self.clearance_m(north_m, east_m) >= self.required_clearance_m

    def margin_m(self, north_m, east_m):
        """How much clearance is left before the enforced boundary is reached."""
        return self.clearance_m(north_m, east_m) - self.required_clearance_m


def build_mission_boundary(plan, anchor):
    """Projects the INNER mission polygon into local NED through the anchor."""
    corners_ne = [anchor.to_local(corner.lat, corner.lon) for corner in plan.corners]
    return MissionBoundary(corners_ne, -RASTER_POLYGON_BREACH_MARGIN_M,
                           name="inner mission polygon")


def build_outer_boundary(anchor, corners=None):
    """Projects the OUTER safety geofence into local NED through the anchor."""
    corners = OUTER_CORNERS if corners is None else corners
    corners_ne = [anchor.to_local(lat, lon) for lat, lon in corners]
    return MissionBoundary(corners_ne, OUTER_GEOFENCE_MARGIN_M,
                           name="outer safety geofence")


def check_boundary_separation(inner, outer):
    """
    Confirms a full inner excursion still leaves room before the outer boundary.

    The two allowances are additive and point the same way: the aircraft may sit
    RASTER_POLYGON_BREACH_MARGIN_M outside the inner edge before the inner check
    fires, and the outer check fires OUTER_GEOFENCE_MARGIN_M before the outer
    edge. What matters is what is left between those two, and that is worth
    measuring rather than assuming.
    """
    worst = float("inf")
    worst_at = None
    ring = inner.ring
    n = len(ring)
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(2, int(length / ROUTE_SAMPLE_STEP_M) + 1)
        for k in range(steps + 1):
            t = k / steps
            q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            c = outer.clearance_m(q[0], q[1])
            if c < worst:
                worst, worst_at = c, q
    remaining = worst - OUTER_GEOFENCE_MARGIN_M - RASTER_POLYGON_BREACH_MARGIN_M
    return {
        "min_inner_to_outer_m": worst,
        "at_ne": worst_at,
        "after_outer_margin_m": worst - OUTER_GEOFENCE_MARGIN_M,
        "remaining_m": remaining,
        "ok": remaining > 0.0,
    }


# ---------------------------------------------------------------------------
# The raster flight leg (installed over the base horizontal leg)
# ---------------------------------------------------------------------------
def stop_horizontal(master, reason):
    """
    Commands zero horizontal velocity. Never raises.

    Every safety exit calls this BEFORE it returns. The base module leaves the
    aircraft coasting on its last setpoint on two of its own abort paths (a failed
    ``check_autonomous_abort`` and a stale LOCAL_POSITION_NED both return without
    commanding a stop, relying on ArduPilot expiring the setpoint a few seconds
    later). Over a 3 m bounce that is a small overshoot; across a raster pattern
    flown near a polygon edge it is not, so the stop is made explicit here.
    """
    try:
        mission.send_local_ned_velocity(master, 0.0, 0.0, 0.0)
    except Exception as exc:
        mission.logger.error(f"[RASTER] Could not send horizontal stop ({reason}): {exc}")


def raster_safety_check(master, state, boundary, outer, context, phase):
    """
    The whole safety gate for one control-loop iteration. Returns False to abort.

    This REPLACES ``mission.horizontal_orbit_safety_check`` in the raster leg and
    is a strict superset of it. Every check the base function performs is
    performed here, in the same order, through the same base functions:

        base line 1060  check_autonomous_abort            -> step 1 below
        base line 1063  CH8=LAND / fence -> LAND mode      -> step 1 below
        base line 1066  local_position_is_fresh            -> step 2 below
        base line 1070  circular fence radius test         -> REPLACED by step 4
        base line 1071  run_horizontal_geofence_breach_land-> step 4 lands via
                                                              run_in_air_safety_land

    Order, matching the agreed runtime safety loop:
        1  check_autonomous_abort   (pilot takeover, CH8 LAND, RC loss, native
                                     FENCE_STATUS, mode change, heartbeat
                                     staleness, EKF health)
        2  LOCAL_POSITION_NED freshness
        3  altitude inside the horizontal-flight band
        4  actual position inside the effective OUTER GPS geofence   -> safety land
        5  actual position inside the INNER mission polygon (RASTER phase only)
        6  battery / endurance
    Every failure path calls stop_horizontal() before it returns.

    What it adds over the base function:
        * zero horizontal velocity BEFORE returning on every failure path;
        * a fixed-to-the-ground OUTER polygon in place of the base module's
          circle centred on the aircraft's own start point;
        * containment of the ACTUAL aircraft inside the inner mission polygon;
        * an altitude band across the horizontal phases;
        * a companion-side endurance gate.

    ``phase`` scopes exactly ONE of those: step 5, the inner polygon. Steps 1, 2,
    3, 4 and 6 run in every phase, because the aircraft is legitimately outside
    the inner area during transit and return but is never allowed outside the
    outer one.
    """
    # 1. Pilot takeover, CH8 LAND, RC loss, native fence, mode change, telemetry
    #    staleness, EKF health — the base module's own gate, unmodified.
    abort_reason = mission.check_autonomous_abort(state, context)
    if abort_reason:
        mission.logger.warning(abort_reason)
        stop_horizontal(master, "autonomous abort")
        if state.chan8_state == "LAND" or state.fence_breached:
            mission.set_flight_mode(master, "LAND", state)
        return False

    # 2. A position estimate old enough to be wrong is worse than none: every
    #    check below this line is computed from local_north_m / local_east_m.
    if not mission.local_position_is_fresh(state):
        mission.logger.warning(
            f"[LOCAL POSITION STALE] No fresh LOCAL_POSITION_NED during {context}.")
        stop_horizontal(master, "stale local position")
        return False

    # Read the pair once. The telemetry thread writes north and east on separate
    # lines with no lock, so re-reading state mid-check can mix two samples.
    north_m = state.local_north_m
    east_m = state.local_east_m

    # 3. Altitude band. Only reached once the horizontal phases have begun, so a
    #    climb through the band cannot trip it.
    altitude_m = state.relative_altitude_m
    if not RASTER_ALT_MIN_M <= altitude_m <= RASTER_ALT_MAX_M:
        mission.logger.error(
            f"[RASTER ALTITUDE] {altitude_m:.2f}m AGL is outside the "
            f"{RASTER_ALT_MIN_M:.2f}-{RASTER_ALT_MAX_M:.2f}m horizontal-flight band "
            f"during {context} (target {RASTER_TAKEOFF_ALTITUDE_M:.2f}m). "
            "Stopping the pattern.")
        stop_horizontal(master, "altitude out of band")
        return False

    # 4. BOUNDARY 2 — the OUTER GPS safety geofence. Enforced in EVERY phase; it
    #    is the containment boundary for the whole horizontal operation.
    if outer is not None and not outer.contains(north_m, east_m):
        clearance = outer.clearance_m(north_m, east_m)
        mission.logger.error(
            f"[OUTER GEOFENCE BREACH] Aircraft is {clearance:+.2f}m from the outer "
            f"safety polygon edge during {context} (phase {phase}); the enforced "
            f"boundary is {OUTER_GEOFENCE_MARGIN_M:.2f}m inside it. Position "
            f"N={north_m:+.2f}m E={east_m:+.2f}m.")
        stop_horizontal(master, "outer geofence breach")
        mission.logger.error(
            "[OUTER GEOFENCE BREACH] Horizontal motion stopped. Handing to the base "
            "in-air safety landing.")
        # The base module's own proven land-in-place path: LAND mode, monitor to
        # touchdown, disarm only once confirmed on the ground, then LOITER. Called
        # directly rather than via run_horizontal_geofence_breach_land, because
        # that wrapper logs the base module's CIRCULAR fence radius, which is not
        # the boundary that just fired and would be misleading in the log.
        mission.run_in_air_safety_land(master, state)
        return False

    # 5. BOUNDARY 1 — the inner mission polygon, tested against the real aircraft.
    #    RASTER phase only: during transit and return the aircraft is outside the
    #    polygon by design and this check would abort a correct flight.
    if (phase in POLYGON_ENFORCED_PHASES and boundary is not None
            and not boundary.contains(north_m, east_m)):
        outside_by = -boundary.clearance_m(north_m, east_m)
        mission.logger.error(
            f"[RASTER POLYGON BREACH] Aircraft is {outside_by:.2f}m outside the mission "
            f"polygon during {context} (phase {phase}, allowed excursion "
            f"{RASTER_POLYGON_BREACH_MARGIN_M:.2f}m). Position N={north_m:+.2f}m "
            f"E={east_m:+.2f}m. Stopping the pattern; the base mission takes over.")
        stop_horizontal(master, "mission polygon breach")
        return False

    # 6. Endurance. Only enforced when the flight controller actually reports it;
    #    battery_percent stays at -1.0 when there is no battery monitor.
    if 0.0 <= state.battery_percent < RASTER_MIN_BATTERY_PERCENT:
        mission.logger.error(
            f"[RASTER BATTERY] {state.battery_percent:.0f}% is below the "
            f"{RASTER_MIN_BATTERY_PERCENT:.0f}% companion minimum during {context}. "
            "Ending the pattern early so the return and landing happen on a "
            "controlled schedule rather than on an autopilot failsafe.")
        stop_horizontal(master, "low battery")
        return False
    if (RASTER_MIN_BATTERY_VOLTAGE_V > 0.0
            and 0.0 < state.battery_voltage_v < RASTER_MIN_BATTERY_VOLTAGE_V):
        mission.logger.error(
            f"[RASTER BATTERY] {state.battery_voltage_v:.2f}V is below the "
            f"{RASTER_MIN_BATTERY_VOLTAGE_V:.2f}V companion minimum during {context}. "
            "Ending the pattern early.")
        stop_horizontal(master, "low battery voltage")
        return False

    return True


def goto_point(master, state, north_m, east_m, tolerance_m, context, phase,
               boundary=None, outer=None, speed_mps=None):
    """
    Flies to one local-NED point, running the base geofence check every iteration.

    Why this does not simply call ``mission.move_to_local_ned_point``:
        that function derives its timeout from the distance between the TARGET and
        the GEOFENCE ORIGIN. For an orbit at a fixed radius those are the same
        thing, so the formula is sound there. A raster pass breaks the assumption:
        the far end of a pass can be 5.5 m of travel away while sitting only 1.5 m
        from the origin, which collapses the timeout to its 15 s floor and aborts
        part-way along the pass. Reusing it unchanged would have produced a
        spurious in-air safety landing mid-pattern.

    Everything else is deliberately identical to the base function: the same
    proportional control law with the same 0.05 m/s floor and the same 10 Hz
    setpoint streaming. The per-iteration safety gate is ``raster_safety_check``.
    Horizontal speed is capped at ``speed_mps`` (RASTER_SPEED_MPS by default) and
    eased down proportionally near the target.
    """
    speed_mps = RASTER_SPEED_MPS if speed_mps is None else speed_mps
    interval = 1.0 / mission.STREAM_HZ
    start_distance = math.hypot(north_m - state.local_north_m, east_m - state.local_east_m)
    timeout_s = max(15.0, 3.0 * start_distance / speed_mps)
    end_time = time.monotonic() + timeout_s

    while time.monotonic() < end_time:
        # abort gate -> freshness -> altitude -> outer geofence -> inner polygon
        # (RASTER phase only) -> endurance. Every failure path stops horizontal
        # motion first.
        if not raster_safety_check(master, state, boundary, outer, context, phase):
            return False

        error_north = north_m - state.local_north_m
        error_east = east_m - state.local_east_m
        distance = math.hypot(error_north, error_east)
        if distance <= tolerance_m:
            mission.send_local_ned_velocity(master, 0.0, 0.0, 0.0)
            return True

        # Speed limit: never above the configured raster speed, and eased down
        # proportionally near the target so the turn does not overshoot the
        # polygon edge the check above is watching.
        travel_speed = min(speed_mps, max(0.05, 0.8 * distance))
        mission.send_local_ned_velocity(master,
                                        travel_speed * error_north / distance,
                                        travel_speed * error_east / distance,
                                        0.0)
        time.sleep(interval)

    short_by = math.hypot(north_m - state.local_north_m, east_m - state.local_east_m)
    mission.logger.warning(
        f"[RASTER] Timed out after {timeout_s:.0f}s while {context} "
        f"({short_by:.2f}m short of the target).")
    stop_horizontal(master, "waypoint timeout")
    return False


def fly_raster_pattern(master, state) -> bool:
    """
    Replacement horizontal leg: cover the mission polygon, then return to origin.

    Returns True only if the whole pattern flew and the aircraft came back to the
    captured origin. On any failure it stops horizontal motion and returns False,
    leaving the landing decision to the base mission — which already chooses
    between the geofence-breach path, the in-air safety land, and yielding to the
    pilot.
    """
    if not mission.local_position_is_fresh(state):
        mission.logger.warning(
            "[RASTER] Cannot start: LOCAL_POSITION_NED is unavailable or stale.")
        return False

    # 1. The geofence origin is the post-takeoff hover position, exactly as the
    #    base mission and the soft-bounce reference define it. It is captured once
    #    and never changed.
    origin_north = state.local_north_m
    origin_east = state.local_east_m

    # 2. Rebuild the plan (cheap, and keeps this leg self-contained).
    try:
        plan, metrics = build_plan()
    except planner.PlanError as exc:
        mission.logger.error(f"[RASTER] {exc.summary}")
        for problem in exc.problems:
            mission.logger.error(f"    - {problem}")
        return False

    # 3. Anchor WGS-84 to local NED from a near-simultaneous snapshot.
    anchor = capture_anchor(state)
    if anchor is None:
        mission.logger.warning("[RASTER] No usable position anchor; not flying the pattern.")
        return False

    mission.logger.info("=========================================================")
    mission.logger.info("=== STARTING POLYGON RASTER COVERAGE PATTERN ===")
    mission.logger.info("=========================================================")
    mission.logger.info(
        f"[RASTER] Geofence origin (post-takeoff hover): N={origin_north:+.2f}m, "
        f"E={origin_east:+.2f}m.")

    # 4. Convert every waypoint into absolute local NED through the anchor.
    targets = []
    for w in plan.waypoints:
        north_m, east_m = anchor.to_local(w.lat, w.lon)
        targets.append((f"WP{w.index:02d} (pass {w.pass_index} {w.kind})", north_m, east_m, w))
    anchor_offset = math.hypot(targets[0][1] - origin_north, targets[0][2] - origin_east)
    mission.logger.info(
        f"[RASTER] {len(targets)} waypoints converted into local NED; the first is "
        f"{anchor_offset:.2f}m from the origin.")

    # 4b. Project BOTH polygons into local NED through the same anchor.
    boundary = build_mission_boundary(plan, anchor)
    outer = build_outer_boundary(anchor)
    origin_clearance = boundary.clearance_m(origin_north, origin_east)
    origin_outer_clearance = outer.clearance_m(origin_north, origin_east)

    mission.logger.info(
        f"[RASTER BOUNDARY] Inner mission polygon projected into local NED; allowed "
        f"excursion {RASTER_POLYGON_BREACH_MARGIN_M:.2f}m beyond its edge, enforced "
        "during the RASTER phase only.")
    mission.logger.info(
        f"[RASTER BOUNDARY] Outer safety geofence projected into local NED from "
        f"{len(OUTER_CORNERS)} GPS corners; enforced boundary is "
        f"{OUTER_GEOFENCE_MARGIN_M:.2f}m inside them, active in EVERY phase.")
    mission.logger.info(
        f"[RASTER BOUNDARY] Altitude band {RASTER_ALT_MIN_M:.2f}-{RASTER_ALT_MAX_M:.2f}m AGL "
        f"(target {RASTER_TAKEOFF_ALTITUDE_M:.2f}m); companion battery floor "
        f"{RASTER_MIN_BATTERY_PERCENT:.0f}%"
        + (f" / {RASTER_MIN_BATTERY_VOLTAGE_V:.2f}V"
           if RASTER_MIN_BATTERY_VOLTAGE_V > 0.0 else "")
        + (f"; flight controller reports {state.battery_percent:.0f}%"
           if state.battery_percent >= 0.0
           else "; flight controller reports no battery percentage, so the "
                "percentage gate is inactive"))

    # How much room is left between a full inner excursion and the enforced outer
    # boundary. Measured, not assumed.
    sep = check_boundary_separation(boundary, outer)
    mission.logger.info(
        f"[RASTER BOUNDARY] Inner polygon to outer polygon: minimum "
        f"{sep['min_inner_to_outer_m']:.2f}m; after the {OUTER_GEOFENCE_MARGIN_M:.2f}m "
        f"outer margin {sep['after_outer_margin_m']:.2f}m; after a full "
        f"{RASTER_POLYGON_BREACH_MARGIN_M:.2f}m inner excursion "
        f"{sep['remaining_m']:.2f}m remains.")
    if not sep["ok"]:
        mission.logger.error(
            "[RASTER BOUNDARY] REFUSING TO FLY: the inner breach tolerance and the "
            "outer safety margin overlap. An aircraft using its full allowed inner "
            "excursion would already be outside the enforced outer boundary, so the "
            "two layers are not independent. Reduce RASTER_POLYGON_BREACH_MARGIN_M or "
            "use a larger outer polygon — neither is changed automatically. No "
            "horizontal command has been sent.")
        return False

    # The origin may legitimately be outside the INNER polygon: the aircraft is
    # placed near the area, not on it. That is what the TRANSIT_TO_RASTER phase is
    # for, so it is reported and NOT treated as a refusal.
    if origin_clearance < 0.0:
        mission.logger.info(
            f"[RASTER BOUNDARY] The takeoff/hover origin is {-origin_clearance:.2f}m "
            "outside the inner mission polygon. That is allowed: inner containment is "
            "enforced during the RASTER phase only.")

    # It may NOT be outside the OUTER geofence, or inside its safety margin. The
    # aircraft is already there, so this is a refusal to move, not a boundary the
    # mission can fly out of.
    mission.logger.info(
        f"[RASTER ORIGIN CHECK] Takeoff/hover origin N={origin_north:+.2f}m "
        f"E={origin_east:+.2f}m sits {origin_outer_clearance:+.2f}m from the outer "
        f"polygon edge; the enforced boundary needs >= {OUTER_GEOFENCE_MARGIN_M:.2f}m, "
        f"leaving {outer.margin_m(origin_north, origin_east):+.2f}m.")
    if not outer.contains(origin_north, origin_east):
        if origin_outer_clearance < 0.0:
            why = (f"{-origin_outer_clearance:.2f}m OUTSIDE the outer safety polygon")
        else:
            why = (f"inside the {OUTER_GEOFENCE_MARGIN_M:.2f}m safety margin, only "
                   f"{origin_outer_clearance:.2f}m from the edge")
        mission.logger.error(
            f"[RASTER ORIGIN CHECK] REFUSING TO FLY: the takeoff/hover origin is {why}.")
        mission.logger.error(
            "    The aircraft is not in a safe place to begin a horizontal mission. It "
            "will NOT be flown into the allowed area, and the geofence is NOT moved to "
            "accommodate it. Land, reposition inside the outer polygon, and retry. No "
            "horizontal command has been sent.")
        return False

    # 5. Whole-route pre-check over ALL THREE PHASES, against the effective outer
    #    polygon. Segments are sampled, not just endpoints.
    route = [("takeoff/hover origin", origin_north, origin_east)]
    route.extend((label, n, e) for label, n, e, _ in targets)
    route.append(("return to origin", origin_north, origin_east))
    if not check_route_inside_outer(outer, route, "transit + pattern + return"):
        return False

    # 6. Time budget for the actual route from this origin.
    transit_m = anchor_offset
    return_m = math.hypot(targets[-1][1] - origin_north, targets[-1][2] - origin_east)
    if not check_time_budget(estimate_route(plan, transit_m, return_m),
                             "route from the captured origin"):
        return False

    # ------------------------------------------------------------------
    # PHASE 1 — TRANSIT_TO_RASTER: origin -> first raster waypoint.
    # Polygon containment is NOT required here. Everything else is.
    # ------------------------------------------------------------------
    deadline = time.monotonic() + RASTER_TOTAL_TIMEOUT_S
    first_label, first_north, first_east, _ = targets[0]
    mission.logger.info(
        f"[RASTER PHASE] {PHASE_TRANSIT}: flying {anchor_offset:.2f}m from the origin "
        f"to {first_label}. Inner-polygon containment is not enforced on this leg; the "
        "outer GPS geofence, telemetry, EKF, altitude and battery checks are.")
    if not goto_point(master, state, first_north, first_east,
                      RASTER_WAYPOINT_TOLERANCE_M, f"transit to {first_label}",
                      PHASE_TRANSIT, boundary=boundary, outer=outer):
        mission.logger.warning(
            f"[RASTER] Aborted during {PHASE_TRANSIT} — stopped by a safety check or "
            "timed out. Yielding to the base mission.")
        return False

    entry_clearance = boundary.clearance_m(state.local_north_m, state.local_east_m)
    mission.logger.info(
        f"[RASTER PHASE] {PHASE_TRANSIT} complete. Aircraft is {entry_clearance:+.2f}m "
        f"from the polygon edge; polygon containment is now ENFORCED "
        f"(allowed excursion {RASTER_POLYGON_BREACH_MARGIN_M:.2f}m).")

    # ------------------------------------------------------------------
    # PHASE 2 — RASTER: the passes. Actual aircraft position is held inside
    # the mission polygon on every control iteration.
    # ------------------------------------------------------------------
    current_pass = 0
    for index, (label, north_m, east_m, w) in enumerate(targets):
        if time.monotonic() > deadline:
            mission.logger.warning(
                f"[RASTER] Pattern budget of {RASTER_TOTAL_TIMEOUT_S:.0f}s exhausted at "
                f"{label}. Stopping horizontal motion.")
            stop_horizontal(master, "pattern budget exhausted")
            return False

        if w.pass_index != current_pass:
            current_pass = w.pass_index
            mission.logger.info(
                f"[RASTER] --- pass {current_pass}/{plan.pass_count}, direction "
                f"{w.direction}, step offset {w.v_m:.2f}m ---")

        # Waypoint 0 was the transit target and has already been reached; re-flying
        # it would only re-run its acceptance check.
        if index == 0:
            continue

        distance = math.hypot(north_m - state.local_north_m, east_m - state.local_east_m)
        mission.logger.info(
            f"[RASTER] -> {label}: target N={north_m:+.2f}m E={east_m:+.2f}m "
            f"({distance:.2f}m away, {math.hypot(north_m - origin_north, east_m - origin_east):.2f}m "
            f"from origin)")

        if not goto_point(master, state, north_m, east_m,
                          RASTER_WAYPOINT_TOLERANCE_M, f"raster {label}",
                          PHASE_RASTER, boundary=boundary, outer=outer):
            mission.logger.warning(
                f"[RASTER] Aborted at {label} — the leg was stopped by a safety check "
                "or timed out. Yielding to the base mission.")
            return False

    mission.logger.info(
        f"[RASTER] All {plan.pass_count} passes complete. Returning to the captured origin.")

    # ------------------------------------------------------------------
    # PHASE 3 — RETURN_TO_ORIGIN: back to the hover point so the base
    # mission lands where it took off. Leaving the polygon here is expected,
    # so containment is not enforced; every other check still is.
    # ------------------------------------------------------------------
    mission.logger.info(
        f"[RASTER PHASE] {PHASE_RETURN}: inner-polygon containment released; the outer "
        "GPS geofence, telemetry, EKF, altitude and battery checks remain active.")
    if not goto_point(master, state, origin_north, origin_east,
                      RASTER_FINAL_TOLERANCE_M, "raster return to origin",
                      PHASE_RETURN, boundary=boundary, outer=outer):
        mission.logger.warning("[RASTER] Could not confirm the return to origin.")
        return False

    final_distance = math.hypot(state.local_north_m - origin_north,
                                state.local_east_m - origin_east)
    mission.logger.info(
        f"[RASTER] Pattern complete; back within {final_distance:.2f}m of the origin. "
        "Horizontal motion stopped.")
    return True


# ===========================================================================
# TELEMETRY-ONLY DRY RUN  (RASTER_DRY_RUN=1)
# ===========================================================================
# For connecting the real MicoAir743v2 while the aircraft is disarmed and
# stationary, to see the actual GPS <-> LOCAL_POSITION_NED relationship and
# check the polygon against the configured safety radius with real numbers.
#
# This is a SEPARATE connection path, not the base daemon stopped early. The
# base main_daemon() writes flight parameters (FENCE_ENABLE, ARMING_CHECK,
# LAND_SPEED, the failsafe set) via PARAM_SET immediately after connecting, at
# ardupilot_horizontal_geofence_mission.py:2029, so there is no point in the base
# path early enough to be safe. Nothing below calls mission.main(),
# mission.main_daemon(), run_ground_autonomous_mission(), or any function that
# sends PARAM_SET.
#
# The ONLY outbound MAVLink this path can emit is MAV_CMD_SET_MESSAGE_INTERVAL
# (mission.request_message_interval), which is a runtime stream-rate request and
# writes nothing persistent. It sends no heartbeat either: without a GCS
# heartbeat ever being seen, disconnecting cannot arm a GCS failsafe.
# ---------------------------------------------------------------------------
DRY_RUN_STREAMS = (
    ("HEARTBEAT", "MAVLINK_MSG_ID_HEARTBEAT", 2.0),
    ("GLOBAL_POSITION_INT", "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 5.0),
    ("LOCAL_POSITION_NED", "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 5.0),
    ("GPS_RAW_INT", "MAVLINK_MSG_ID_GPS_RAW_INT", 2.0),
    ("EKF_STATUS_REPORT", "MAVLINK_MSG_ID_EKF_STATUS_REPORT", 2.0),
    ("SYS_STATUS", "MAVLINK_MSG_ID_SYS_STATUS", 2.0),
)


class DryRunArmed(Exception):
    """Raised the instant the flight controller reports itself armed."""


class DryRunState:
    """Telemetry gathered by the dry run. Deliberately not the base TelemetryState."""

    def __init__(self):
        self.system_id = 0
        self.component_id = 0
        self.mode = "UNKNOWN"
        self.is_armed = False
        self.heartbeat_time = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.global_time = 0.0
        self.north_m = 0.0
        self.east_m = 0.0
        self.down_m = 0.0
        self.local_time = 0.0
        self.gps_fix_type = 0
        self.satellites_visible = 0
        self.gps_time = 0.0
        self.ekf_flags = 0
        self.ekf_ok = False
        self.ekf_time = 0.0
        self.battery_voltage_v = 0.0
        self.battery_percent = -1.0
        self.battery_time = 0.0

    def ages(self, now):
        def age(stamp):
            return (now - stamp) if stamp > 0.0 else float("inf")
        return {
            "HEARTBEAT": age(self.heartbeat_time),
            "GLOBAL_POSITION_INT": age(self.global_time),
            "LOCAL_POSITION_NED": age(self.local_time),
            "GPS_RAW_INT": age(self.gps_time),
            "EKF_STATUS_REPORT": age(self.ekf_time),
            "SYS_STATUS": age(self.battery_time),
        }


def _dry_run_request_streams(master, log):
    """The one and only kind of outbound message this path emits."""
    log("Requesting telemetry stream rates (MAV_CMD_SET_MESSAGE_INTERVAL — runtime")
    log("only; this is a command, not a parameter, and persists nothing):")
    for name, attr, hz in DRY_RUN_STREAMS:
        msg_id = getattr(mission.mavutil.mavlink, attr, None)
        if msg_id is None:                       # dialect without this message
            log(f"    - {name}: not in this MAVLink dialect, skipped")
            continue
        mission.request_message_interval(master, msg_id, hz)
        log(f"    - {name} at {hz:.0f} Hz")


def _dry_run_collect(master, state, log):
    """
    Reads telemetry until every required message has arrived, or the time is up.

    Raises DryRunArmed the moment the flight controller reports ARMED.
    """
    deadline = time.monotonic() + DRY_RUN_TIMEOUT_S
    required = ("HEARTBEAT", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "GPS_RAW_INT")
    settled_at = None

    while time.monotonic() < deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        now = time.monotonic()
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            if msg.get_srcSystem() != master.target_system:
                continue
            if msg.get_srcComponent() not in (0, mission.mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                continue
            state.system_id = msg.get_srcSystem()
            state.component_id = msg.get_srcComponent()
            state.heartbeat_time = now
            state.mode = mission.get_mode_name(master, msg)
            state.is_armed = bool(msg.base_mode
                                  & mission.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if state.is_armed:
                raise DryRunArmed(
                    f"HEARTBEAT reports ARMED (mode={state.mode}, "
                    f"base_mode=0x{msg.base_mode:02X})")

        elif msg_type == "GLOBAL_POSITION_INT":
            state.lat = msg.lat / 1e7
            state.lon = msg.lon / 1e7
            state.global_time = now

        elif msg_type == "LOCAL_POSITION_NED":
            state.north_m = msg.x
            state.east_m = msg.y
            state.down_m = msg.z
            state.local_time = now

        elif msg_type == "GPS_RAW_INT":
            state.gps_fix_type = msg.fix_type
            state.satellites_visible = msg.satellites_visible
            state.gps_time = now

        elif msg_type == "EKF_STATUS_REPORT":
            state.ekf_flags = msg.flags
            state.ekf_ok = bool(msg.flags & (1 << 4))
            state.ekf_time = now

        elif msg_type == "SYS_STATUS":
            state.battery_voltage_v = msg.voltage_battery / 1000.0
            if msg.battery_remaining != -1:
                state.battery_percent = float(msg.battery_remaining)
            state.battery_time = now

        elif msg_type == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(
                "utf-8", errors="ignore")
            log(f"    [AP] {text.strip()}")

        ages = state.ages(time.monotonic())
        if all(ages[name] < 2.0 for name in required):
            if settled_at is None:
                settled_at = time.monotonic()
                log(f"All required telemetry is arriving; settling for "
                    f"{DRY_RUN_SETTLE_S:.1f}s before sampling the anchor.")
            elif time.monotonic() - settled_at >= DRY_RUN_SETTLE_S:
                return True

    return False


def _dry_run_geometry(state, plan, metrics, log):
    """
    Converts the polygon and every waypoint into the real aircraft's local frame
    and checks the result against the configured safety radius.

    Returns True only if the geometry passes.
    """
    # The runtime anchor. For a stationary disarmed aircraft the two observations
    # are of the same physical point, so their skew only has to be small enough
    # that neither sample is stale.
    # Wall clock on purpose: this stamp is only ever rendered for the operator
    # (strftime, below).  It is never used as an age or a deadline, so it is the
    # one place in the flight path that may still read time.time().
    anchor = Anchor(state.lat, state.lon, state.north_m, state.east_m,
                    time.time(), abs(state.global_time - state.local_time))

    log("")
    log("GPS <-> LOCAL_POSITION_NED RUNTIME ANCHOR")
    log(RULE)
    log(f"  anchor_lat          : {anchor.lat:.7f}")
    log(f"  anchor_lon          : {anchor.lon:.7f}")
    log(f"  anchor_local_north  : {anchor.north_m:+.3f} m")
    log(f"  anchor_local_east   : {anchor.east_m:+.3f} m")
    log(f"  timestamp           : {anchor.timestamp:.3f} "
        f"({time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(anchor.timestamp))})")
    log(f"  sample skew         : {anchor.skew_s * 1000.0:.0f} ms "
        f"(limit {RASTER_ANCHOR_MAX_SKEW_S * 1000.0:.0f} ms)")
    log("")
    log("  The EKF origin is NOT the GPS home and NOT the mission area. The offset")
    log("  between the two frames, as measured right now, is:")
    zero_n, zero_e = anchor.to_local(plan.corners[0].lat, plan.corners[0].lon)
    log(f"    corner A ({plan.corners[0].lat:.7f}, {plan.corners[0].lon:.7f}) sits at "
        f"local N={zero_n:+.2f}m E={zero_e:+.2f}m")

    # The reference every distance below is measured from: where the aircraft is
    # sitting right now.
    ref_n, ref_e = state.north_m, state.east_m

    def dist(n, e):
        return math.hypot(n - ref_n, e - ref_e)

    outer = build_outer_boundary(anchor)
    outer_measured = planner.measure_ring(list(outer.ring))
    effective_ring = planner.effective_outer_ring(list(outer.ring), OUTER_GEOFENCE_MARGIN_M)

    log("")
    log("OUTER SAFETY GEOFENCE IN THE REAL AIRCRAFT'S LOCAL FRAME")
    log(RULE)
    log("   Corner        latitude     longitude      north(m)   east(m)   dist from ref(m)")
    outer_names = ("G1", "G2", "G3", "G4")
    for name, (lat, lon) in zip(outer_names, OUTER_CORNERS):
        n, e = anchor.to_local(lat, lon)
        log(f"     {name:<4}    {lat:12.7f} {lon:12.7f}    "
            f"{n:+8.2f}  {e:+8.2f}       {dist(n, e):8.2f}")
    log(f"  area                 : {outer_measured['area']:.2f} m^2")
    log("  side lengths         : "
        + ", ".join(f"{outer_names[i]}{outer_names[(i + 1) % 4]}={v:.2f}m"
                    for i, v in enumerate(outer_measured['sides'])))
    log(f"  perimeter            : {outer_measured['perimeter']:.2f} m")
    log(f"  winding (as stored)  : {outer_measured['winding']} in (north,east), "
        "normalised counter-clockwise for the half-plane test")
    log(f"  inward safety margin : {OUTER_GEOFENCE_MARGIN_M:.2f} m")
    log(f"  effective boundary   : {len(effective_ring)} vertices, "
        f"{abs(planner.signed_area(effective_ring)):.2f} m^2 "
        f"({outer_measured['area'] - abs(planner.signed_area(effective_ring)):.2f} m^2 "
        "given up to the margin)")
    here_outer = outer.clearance_m(ref_n, ref_e)
    log(f"  aircraft clearance   : {here_outer:+.2f} m to the raw outer edge, "
        f"{outer.margin_m(ref_n, ref_e):+.2f} m to the enforced boundary")

    log("")
    log("INNER RASTER POLYGON CORNERS IN THE REAL AIRCRAFT'S LOCAL FRAME")
    log(RULE)
    log("   Corner        latitude     longitude      north(m)   east(m)   dist from ref(m)")
    corner_ne = []
    for corner in plan.corners:
        n, e = anchor.to_local(corner.lat, corner.lon)
        corner_ne.append((n, e))
        log(f"     {corner.name:<4}    {corner.lat:12.7f} {corner.lon:12.7f}    "
            f"{n:+8.2f}  {e:+8.2f}       {dist(n, e):8.2f}")

    log("")
    log("RASTER WAYPOINTS IN THE REAL AIRCRAFT'S LOCAL FRAME")
    log(RULE)
    log("    WP  Pass  Kind        north(m)   east(m)   dist from ref(m)")
    waypoint_ne = []
    worst_n = worst_e = 0.0
    max_radius = 0.0
    min_radius = float("inf")
    max_label = min_label = ""
    for w in plan.waypoints:
        n, e = anchor.to_local(w.lat, w.lon)
        waypoint_ne.append((w, n, e))
        d = dist(n, e)
        if d > max_radius:
            max_radius, max_label, worst_n, worst_e = d, f"WP{w.index:02d}", n, e
        if d < min_radius:
            min_radius, min_label = d, f"WP{w.index:02d}"
        log(f"    {w.index:3d}  {w.pass_index:4d}  {w.kind:<10}  {n:+8.2f}  {e:+8.2f}       "
            f"{d:8.2f}")

    first_w, first_n, first_e = waypoint_ne[0]
    last_w, last_n, last_e = waypoint_ne[-1]
    transit_m = math.hypot(first_n - ref_n, first_e - ref_e)
    return_m = math.hypot(last_n - ref_n, last_e - ref_e)
    log("")
    log("MISSION PHASES FROM THIS POSITION")
    log(RULE)
    log(f"  1. {PHASE_TRANSIT:<18} origin -> WP{first_w.index:02d}, {transit_m:.2f} m")
    log("     INNER polygon NOT enforced; OUTER geofence, telemetry, EKF,")
    log("     altitude and battery all enforced")
    log(f"  2. {PHASE_RASTER:<18} {plan.pass_count} passes, {plan.path_length_m:.2f} m of path, "
        f"~{plan.estimated_time_s:.0f} s at {plan.speed_mps:.2f} m/s")
    log("     INNER polygon ENFORCED against the ACTUAL aircraft position,")
    log("     OUTER geofence and everything else also enforced")
    log(f"  3. {PHASE_RETURN:<18} WP{last_w.index:02d} -> origin, {return_m:.2f} m, then the "
        "base soft landing")
    log("     INNER polygon NOT enforced; OUTER geofence and everything else enforced")

    # Boundary 1: does the aircraft's current position sit inside the polygon it
    # is being asked to survey? Not fatal on its own — the geofence origin is the
    # post-takeoff hover point, not this one — but it is the single most useful
    # number for deciding where to put the aircraft down before the flight.
    boundary = MissionBoundary(corner_ne, -RASTER_POLYGON_BREACH_MARGIN_M,
                               name="inner mission polygon")
    here_clearance = boundary.clearance_m(ref_n, ref_e)

    m = plan.measurements
    log("")
    log("INNER POLYGON AND RASTER PATH")
    log(RULE)
    log(f"  sides               : AB={m['ab']:.2f}m  BC={m['bc']:.2f}m  "
        f"CD={m['cd']:.2f}m  DA={m['da']:.2f}m")
    log(f"  diagonals           : AC={m['ac']:.2f}m  BD={m['bd']:.2f}m")
    log(f"  area                : {m['area']:.2f} m^2")
    log(f"  edge inset          : {plan.inset_m:.2f} m")
    log(f"  raster passes       : {plan.pass_count} at {plan.pass_spacing_actual_m:.3f}m "
        f"spacing ({len(plan.waypoints)} waypoints)")
    log(f"  raster path length  : {plan.path_length_m:.2f} m")
    log(f"  est. horizontal time: {plan.estimated_time_s:.0f} s at {plan.speed_mps:.2f} m/s "
        f"(travel only; budget is {RASTER_TOTAL_TIMEOUT_S:.0f}s)")
    log(f"  planned containment : {metrics['waypoints_inside_inset']}/{metrics['waypoints']} "
        f"waypoints, {metrics['segments']} segments verified inside the inset polygon")

    log("")
    log("DISTANCES FROM THE CAPTURED REFERENCE")
    log(RULE)
    log(f"  reference (aircraft now): local N={ref_n:+.2f}m E={ref_e:+.2f}m")
    log(f"  nearest waypoint        : {min_label} at {min_radius:.2f} m")
    log(f"  farthest waypoint       : {max_label} at {max_radius:.2f} m "
        f"(N={worst_n:+.2f}m E={worst_e:+.2f}m)")
    log(f"  transit leg length      : {transit_m:.2f} m")
    log(f"  return leg length       : {return_m:.2f} m")
    log(f"  aircraft vs polygon     : {here_clearance:+.2f} m from the edge "
        f"({'inside' if here_clearance >= 0.0 else 'OUTSIDE'} the mission polygon)")
    if here_clearance < 0.0:
        log("     -> allowed. The takeoff point does not have to be inside the polygon;")
        log("        the TRANSIT_TO_RASTER phase exists precisely for this.")

    # ------------------------------------------------------------------
    # INNER vs OUTER relationship
    # ------------------------------------------------------------------
    sep = check_boundary_separation(boundary, outer)
    corner_names = ("A", "B", "C", "D")
    log("")
    log("INNER / OUTER RELATIONSHIP")
    log(RULE)
    inner_outside = []
    for name, (n, e) in zip(corner_names, corner_ne):
        c = outer.clearance_m(n, e)
        log(f"  inner corner {name}: {c:+.3f} m from the outer edge, "
            f"{c - OUTER_GEOFENCE_MARGIN_M:+.3f} m from the enforced boundary")
        if c < 0.0:
            inner_outside.append(name)
    log(f"  minimum inner-to-outer separation      : {sep['min_inner_to_outer_m']:.3f} m")
    log(f"  after the {OUTER_GEOFENCE_MARGIN_M:.2f} m outer safety margin  : "
        f"{sep['after_outer_margin_m']:.3f} m")
    log(f"  inner runtime breach tolerance          : {RASTER_POLYGON_BREACH_MARGIN_M:.2f} m "
        "(allowed excursion beyond the inner edge, RASTER phase only)")
    log(f"  worst remaining separation              : {sep['remaining_m']:.3f} m")
    log("     i.e. an aircraft using its full allowed inner excursion at the tightest")
    log("     point still has this much left before the enforced outer boundary.")

    if inner_outside:
        log("")
        log(f"  RESULT: FAIL — inner corner(s) {', '.join(inner_outside)} lie outside the "
            "outer safety geofence.")
        return False
    if not sep["ok"]:
        log("")
        log("  RESULT: FAIL — the inner breach tolerance and the outer safety margin")
        log("          overlap; the two layers are not independent.")
        return False
    log("  inner polygon completely inside outer   : YES")

    # ------------------------------------------------------------------
    # Whole route against the effective outer geofence, and the time budget.
    # Same rules the in-flight pre-checks apply, so a dry-run PASS predicts an
    # in-flight acceptance instead of merely resembling it.
    # ------------------------------------------------------------------
    route = [("takeoff/hover origin", ref_n, ref_e)]
    route.extend((f"WP{w.index:02d}", n, e) for w, n, e in waypoint_ne)
    route.append(("return to origin", ref_n, ref_e))

    log("")
    log("TAKEOFF / HOME SUITABILITY AND WHOLE-ROUTE CONTAINMENT")
    log(RULE)
    origin_ok = outer.contains(ref_n, ref_e)
    log(f"  current position as a takeoff point : "
        f"{'SUITABLE' if origin_ok else 'NOT SUITABLE'}")
    log(f"  clearance to the raw outer edge      : {here_outer:+.3f} m")
    log(f"  clearance to the enforced boundary   : {outer.margin_m(ref_n, ref_e):+.3f} m")
    log(f"  inside the inner raster polygon      : "
        f"{'yes' if here_clearance >= 0.0 else 'no (allowed — TRANSIT_TO_RASTER exists)'}")
    log(f"  distance to the first raster waypoint: {transit_m:.2f} m")
    log(f"  farthest mission point               : {max_label} at {max_radius:.2f} m")
    if not origin_ok:
        log("")
        if here_outer < 0.0:
            log(f"  RESULT: FAIL — this position is {-here_outer:.2f} m OUTSIDE the outer")
            log("          safety geofence. The mission would refuse to move from here.")
        else:
            log(f"  RESULT: FAIL — this position is inside the {OUTER_GEOFENCE_MARGIN_M:.2f} m")
            log(f"          safety margin, only {here_outer:.2f} m from the outer edge.")
            log("          The mission would refuse to move from here.")
        log("          The geofence is NOT moved to accommodate the aircraft.")
        return False

    if not check_route_inside_outer(outer, route, "dry-run route from this position"):
        log("")
        log("  RESULT: FAIL — part of the planned route leaves the effective outer")
        log("          geofence from this takeoff position.")
        return False

    estimate = estimate_route(plan, transit_m, return_m)
    log("")
    log("TIME BUDGET")
    log(RULE)
    log(f"  transit {estimate['transit_m']:.1f} m + pattern {estimate['pattern_m']:.1f} m "
        f"+ return {estimate['return_m']:.1f} m = {estimate['total_m']:.1f} m")
    log(f"  straight-line at {RASTER_SPEED_MPS:.2f} m/s : {estimate['straight_s']:.0f} s")
    log(f"  x{RASTER_TIME_CONTINGENCY:.2f} contingency          : "
        f"{estimate['expected_s']:.0f} s expected")
    log(f"  configured budget              : {RASTER_TOTAL_TIMEOUT_S:.0f} s "
        f"({RASTER_TOTAL_TIMEOUT_S - estimate['expected_s']:+.0f} s)")
    if estimate["expected_s"] > RASTER_TOTAL_TIMEOUT_S:
        log("")
        log("  RESULT: FAIL — the planned horizontal route cannot finish inside the")
        log(f"          budget. Over by {estimate['expected_s'] - RASTER_TOTAL_TIMEOUT_S:.0f} s.")
        log(f"          Driver: {estimate['pattern_m']:.1f} m of pattern at "
            f"{RASTER_PASS_SPACING_M:.2f} m spacing over a {RASTER_SPEED_MPS:.2f} m/s cap.")
        log("          Nothing is adjusted automatically — budget, speed and spacing are")
        log("          all operator decisions, and aircraft endurance is the real limit.")
        return False

    log("")
    log(f"  RESULT: PASS — the whole route fits inside the effective outer geofence "
        f"and the {RASTER_TOTAL_TIMEOUT_S:.0f} s budget.")
    return True

def run_dry_run() -> int:
    """
    Telemetry-only connection to the real flight controller. Returns an exit code.

    Sends no arm, no disarm, no mode change, no takeoff, no land, no RTL, no
    velocity, no position target, and no PARAM_SET.
    """
    log = mission.logger.info
    log(BAR)
    log("=== RASTER DRY RUN — TELEMETRY ONLY, NO FLIGHT COMMANDS ===")
    log(BAR)
    log("This mode reads telemetry and computes geometry. It will not arm, change")
    log("mode, take off, land, send any velocity or position target, or write any")
    log("flight-controller parameter. The aircraft must be DISARMED and STATIONARY.")
    log("")

    # Geometry first: an unusable polygon must not get as far as opening the port.
    try:
        validate_configuration()
        plan, metrics = build_plan()
    except (ValueError, planner.PlanError) as exc:
        summary = getattr(exc, "summary", str(exc))
        log("")
        mission.logger.error(f"DRY RUN FAIL: {summary}")
        for problem in getattr(exc, "problems", []):
            mission.logger.error(f"    - {problem}")
        mission.logger.error("No flight controller connection was opened.")
        return 1

    # Only one process may read the flight controller. The senior mission daemon
    # holds this same lock, so taking it here is what stops a dry run from being
    # started while ardupilot-mission.service is live and quietly stealing half of
    # each other's MAVLink bytes.
    try:
        lock_handle = open(mission.LOCK_FILE, "w")
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        mission.logger.error(
            f"DRY RUN FAIL: another process already holds {mission.LOCK_FILE}.")
        mission.logger.error(
            "    The autonomous mission daemon is almost certainly running. Two "
            "readers on one serial link corrupt the MAVLink stream.")
        mission.logger.error(
            "    Check with:  systemctl is-active ardupilot-mission.service")
        return 1
    lock_handle.write(f"{os.getpid()}\n")
    lock_handle.flush()

    connection = mission.resolve_connection_string()
    if connection is None:
        mission.logger.error("DRY RUN FAIL: no flight controller found. Set "
                             "ARDUPILOT_CONNECTION or attach the board.")
        return 1

    is_network = connection.startswith("udp") or connection.startswith("tcp")
    if not is_network and not os.path.exists(connection):
        mission.logger.error(f"DRY RUN FAIL: {connection} does not exist.")
        return 1

    log(f"Opening {connection} at {mission.BAUD_RATE} baud (read path only)...")
    master = mission.mavutil.mavlink_connection(
        connection, baud=mission.BAUD_RATE, dialect="ardupilotmega")

    state = DryRunState()
    passed = False
    try:
        heartbeat = master.wait_heartbeat(timeout=15.0)
        if not heartbeat:
            mission.logger.error("DRY RUN FAIL: no heartbeat within 15s.")
            return 1
        if master.target_component in (0, mission.mavutil.mavlink.MAV_COMP_ID_ALL):
            master.target_component = mission.mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

        log(f"Heartbeat received from system {master.target_system}, "
            f"component {master.target_component}.")

        # Check armed on the very first heartbeat, BEFORE anything is transmitted,
        # so an armed aircraft is abandoned without this process having sent a
        # single MAVLink byte.
        if bool(heartbeat.base_mode & mission.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            state.is_armed = True
            raise DryRunArmed(
                "the first heartbeat already reports ARMED "
                f"(base_mode=0x{heartbeat.base_mode:02X}); nothing was transmitted")

        _dry_run_request_streams(master, log)
        log("")

        complete = _dry_run_collect(master, state, log)
        now = time.monotonic()
        ages = state.ages(now)

        log("")
        log("REAL FLIGHT CONTROLLER STATE")
        log(RULE)
        log(f"  system ID           : {state.system_id or master.target_system}")
        log(f"  component ID        : {state.component_id or master.target_component}")
        log(f"  flight mode         : {state.mode}")
        log(f"  armed               : {state.is_armed}")
        log(f"  GPS fix type        : {state.gps_fix_type} "
            f"(need >= {RASTER_MIN_GPS_FIX_TYPE})")
        log(f"  satellites visible  : {state.satellites_visible}")
        log(f"  latitude            : {state.lat:.7f}")
        log(f"  longitude           : {state.lon:.7f}")
        log(f"  LOCAL_POSITION_NED  : north={state.north_m:+.3f}m  east={state.east_m:+.3f}m  "
            f"down={state.down_m:+.3f}m")
        log(f"  EKF flags           : {state.ekf_flags} "
            f"(POS_HORIZ_ABS {'SET' if state.ekf_ok else 'CLEAR'})")
        log(f"  battery             : {state.battery_voltage_v:.2f} V, "
            + (f"{state.battery_percent:.0f}%" if state.battery_percent >= 0.0
               else "percentage not reported"))
        log("  telemetry ages:")
        for name in ("HEARTBEAT", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED",
                     "GPS_RAW_INT", "EKF_STATUS_REPORT", "SYS_STATUS"):
            age = ages[name]
            log(f"    - {name:<20} {'never received' if age == float('inf') else f'{age:.2f}s'}")

        # --- pass conditions, each reported rather than merely totalled --------
        problems = []
        if state.is_armed:
            problems.append("the flight controller is ARMED")
        if ages["HEARTBEAT"] > mission.TELEMETRY_MAX_AGE_S:
            problems.append(f"no fresh HEARTBEAT (age {ages['HEARTBEAT']:.2f}s)")
        if state.gps_fix_type < RASTER_MIN_GPS_FIX_TYPE:
            problems.append(
                f"GPS fix type {state.gps_fix_type} is below the required "
                f"{RASTER_MIN_GPS_FIX_TYPE} (sats={state.satellites_visible})")
        if ages["GLOBAL_POSITION_INT"] > RASTER_GPS_MAX_AGE_S:
            problems.append(
                f"no fresh GLOBAL_POSITION_INT (age {ages['GLOBAL_POSITION_INT']:.2f}s, "
                f"limit {RASTER_GPS_MAX_AGE_S:.2f}s)")
        if ages["LOCAL_POSITION_NED"] > mission.LOCAL_POSITION_MAX_AGE_S:
            problems.append(
                f"no fresh LOCAL_POSITION_NED (age {ages['LOCAL_POSITION_NED']:.2f}s, "
                f"limit {mission.LOCAL_POSITION_MAX_AGE_S:.2f}s)")
        skew = abs(state.global_time - state.local_time)
        if state.global_time > 0.0 and state.local_time > 0.0 and skew > RASTER_ANCHOR_MAX_SKEW_S:
            problems.append(
                f"GPS and local-position samples are {skew:.3f}s apart, above the "
                f"{RASTER_ANCHOR_MAX_SKEW_S:.3f}s anchor limit")
        # The real mission needs an absolute horizontal position estimate: the
        # base check_autonomous_abort aborts on ekf_ok being False, so a dry run
        # that ignored it would pass an aircraft that cannot fly the pattern.
        if DRY_RUN_REQUIRE_EKF and not state.ekf_ok:
            problems.append(
                f"EKF does not report POS_HORIZ_ABS (flags={state.ekf_flags}); the "
                "mission's own check_autonomous_abort would abort on this")
        if not complete:
            problems.append(
                f"required telemetry did not settle within {DRY_RUN_TIMEOUT_S:.0f}s")

        if problems:
            log("")
            mission.logger.error("DRY RUN FAIL — telemetry preconditions not met:")
            for problem in problems:
                mission.logger.error(f"    - {problem}")
            mission.logger.error("No geometry was computed and no command was sent.")
            return 1

        passed = _dry_run_geometry(state, plan, metrics, log)

    except DryRunArmed as exc:
        mission.logger.error("")
        mission.logger.error("!" * 74)
        mission.logger.error("!!! ABORT: THE FLIGHT CONTROLLER IS ARMED !!!")
        mission.logger.error(f"!!! {exc}")
        mission.logger.error("!!! The dry run requires a DISARMED, stationary aircraft.")
        mission.logger.error("!!! Closing the link now. No command of any kind was sent.")
        mission.logger.error("!" * 74)
        return 1
    except Exception as exc:
        mission.logger.exception(f"DRY RUN FAIL: unexpected error — {exc}")
        return 1
    finally:
        try:
            master.close()
            log("MAVLink connection closed.")
        except Exception:
            pass

    log("")
    log(BAR)
    log(f"=== RASTER DRY RUN: {'PASS' if passed else 'FAIL'} ===")
    log(BAR)
    return 0 if passed else 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    # Logging first, so a refusal below is recorded in the mission log and not
    # only on the terminal.
    mission.setup_logging()

    mission.logger.info("==========================================================")
    mission.logger.info("=== ARDUPILOT POLYGON RASTER COVERAGE MISSION ===")
    mission.logger.info("==========================================================")
    mission.logger.info(f"Project directory : {PROJECT_DIR}")
    mission.logger.info(f"Mission log       : {mission.LOG_FILE}")
    mission.logger.info(f"Trigger file      : {mission.TRIGGER_FILE}")
    mission.logger.info(f"Recordings        : {mission.video_recorder.RECORDINGS_DIR}")

    # Telemetry-only dry run. Dispatched here, before anything installs the
    # raster leg or starts the base daemon, so no flight path is even reachable.
    if RASTER_DRY_RUN:
        sys.exit(run_dry_run())

    # Validate parameters and the polygon BEFORE touching the flight controller.
    # A bad polygon must not get as far as connecting, let alone arming.
    try:
        validate_configuration()
    except ValueError as exc:
        mission.logger.error("=========================================================")
        mission.logger.error(f"REFUSING TO START: invalid configuration — {exc}")
        mission.logger.error("No flight controller connection has been opened.")
        mission.logger.error("=========================================================")
        sys.exit(2)

    try:
        plan, metrics = build_plan()
    except planner.PlanError as exc:
        mission.logger.error("=========================================================")
        mission.logger.error(f"REFUSING TO START: {exc.summary}")
        for problem in exc.problems:
            mission.logger.error(f"    - {problem}")
        mission.logger.error("")
        mission.logger.error("The mission polygon is not usable, so no raster path exists.")
        mission.logger.error("No flight controller connection has been opened and no flight")
        mission.logger.error("command has been generated. Fix the corners and try again.")
        mission.logger.error("=========================================================")
        sys.exit(2)
    except Exception as exc:
        mission.logger.exception(f"REFUSING TO START: unexpected planning failure — {exc}")
        sys.exit(2)

    log_plan(plan, metrics)

    # The base mission resolves this symbol at runtime, so replacing it changes
    # only the horizontal leg and leaves every proven safety, takeoff, landing,
    # trigger and reconnect path intact.
    mission.fly_horizontal_geofence_orbit = fly_raster_pattern
    mission.main()


if __name__ == "__main__":
    main()
