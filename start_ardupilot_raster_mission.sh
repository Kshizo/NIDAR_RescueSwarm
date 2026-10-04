#!/bin/bash
set -euo pipefail

# ---------------------------------------------------------------------------
# Launcher for the ArduPilot POLYGON RASTER COVERAGE mission daemon.
#
# This is the RASTER launcher. It is deliberately separate from the senior
# soft-bounce launcher at /home/aahswarm/ardupilot_testing/start_ardupilot_mission.sh
# and must never be used to start that mission.
#
# Started by: ardupilot-raster-mission.service
#
# The daemon connects to the MicoAir743v2 over USB MAVLink, then WAITS in
# STANDBY. It starts a mission only on an observed FRESH transition into GUIDED
# flight mode (TRIGGER_ON_GUIDED_MODE=1). Being already in GUIDED at startup
# does NOT start a mission.
# ---------------------------------------------------------------------------

VENV_PATH="/home/aahswarm/px4_mavsdk_env"
PROJECT_PATH="/home/aahswarm/ardupilotRasterFix"

# --- real-flight guards ----------------------------------------------------
# These are stated explicitly rather than left to defaults so that a stale
# environment inherited from a shell or from systemd can never turn the real
# flight daemon into a dry run, and can never disable ArduPilot's arming checks.
export RASTER_DRY_RUN=0        # real mission, NOT the telemetry-only dry run
export BENCH_MODE=0            # normal ArduPilot pre-arm / arming checks stay ON
export TRIGGER_ON_GUIDED_MODE=1  # GUIDED-mode transition is the only launch trigger
export AUTOSTART_ON_READY=0    # never start just because the link came up
export AUTOSTART_WITHOUT_RC=0  # never start just because there is no receiver
export CONFIGURE_FENCE=0       # companion-side polygons only; no native FENCE_* fence
# Takeoff altitude and the horizontal-flight altitude band.
#
# 2026-09-17: raised from 2.0 m at the pilot's request — the airframe sags slowly
# after takeoff, so 2.0 m left too little room under it.
#
# The band is deliberately ASYMMETRIC about the target. The observed error has a
# direction: this aircraft drifts DOWN, not up. A symmetric +/-0.50 m band would
# move the floor up with the target and buy no extra sag margin at all, so the
# floor is set 0.70 m below the target and the ceiling 0.50 m above.
#
#   floor 1.80 m   <-- 0.70 m of sag allowed before [RASTER ALTITUDE] aborts
#   target 2.50 m
#   ceiling 3.00 m
#
# NOTE this is margin, not a fix. Every raster leg commands a vertical velocity of
# exactly 0.0 (goto_point, ardupilot_raster_mission.py:975) and leaves altitude
# entirely to the autopilot's own hold, so a standing sag is never corrected by
# the mission — it just takes longer to reach the floor. At 1.00 m/s the pattern
# is ~108 s, so 0.70 m of allowance tolerates a sag of about 6 mm/s. If the real
# rate is worse than that, the leg needs a proportional vertical term, not a
# wider band.
export TAKEOFF_ALTITUDE_M=2.5
export RASTER_ALT_MIN_M=1.80
export RASTER_ALT_MAX_M=3.00

export RASTER_PASS_SPACING_M=1.0

# Sweep axis.  "long" is the original behaviour: on the current 20.3 x 6.0 m plot
# the passes run along the 20.3 m side, so there are 6 passes of ~20 m and the
# aircraft flies 40 s in one direction before its first turn.  That is what made
# the 2026-09-15 flight look like a straight line out and back rather than a
# raster, and the pilot took over 5 s before that first turn.
#
# "short" runs the passes along the 6.0 m side instead: 21 passes of ~3-5 m, a
# turn roughly every 10 s, and the first turn 24 s after the pattern starts.
#
# "short" is the default here because on THIS plot it is also barely more
# expensive.  Measured from the 2026-09-15 origin, whole route including transit
# and return: long 126.0 m / 252 s, short 132.4 m / 265 s - 13 seconds.  The long
# sweep loses most of its turn-count advantage to an 8.0 m dead transition,
# because the plot is a trapezoid (BC=3.71 m, DA=5.99 m) and the last passes clip
# to stubs near corner A.  The short sweep's transitions are all one spacing,
# 0.98 m.
#
# Set RASTER_AXIS=long to restore the original sweep.  Do that only on a plot
# whose passes stay short enough to turn often, or after a flight that has already
# proved the pattern to the pilot.
export RASTER_AXIS=short

# Report every leg in progress at this interval.  Without it a 40 s pass produces
# exactly two log lines and there is no way to tell tracking from stuck.
export RASTER_PROGRESS_INTERVAL_S=3.0
# Speed is limited by overshoot at the pass ends, not by the budget.  A pass-end
# waypoint sits 0.20m inside the polygon and the breach abort fires 0.50m outside
# it, so there is 0.70m of overshoot to spend.  At 0.50m/s the autopilot's stop
# distance plus tracking lag uses ~0.25m, leaving 0.45m.
#
# 2026-09-17: raised to 1.00m/s at the pilot's request.  This is ABOVE the speed
# the 0.70m overshoot budget supports.  Scaling that measured 0.25m from 0.50m/s:
# if overshoot is lag-dominated it grows linearly to ~0.50m (0.20m spare); if it
# is deceleration-dominated it grows with the square to ~1.00m, which is 0.30m
# PAST the breach tolerance.  Which of the two it is has not been measured.
#
# The consequence of being wrong is a mission abort, not a loss of containment:
# tripping RASTER_POLYGON_BREACH_MARGIN_M stops the pattern and hands over to the
# base safety land, and the outer geofence still has 1.66m of clearance to the
# planned path at its worst point.  The aircraft stays inside the test area either
# way; the flight just ends early.
#
# RASTER_AXIS=short also means 20 pass ends per flight instead of the 6 the long
# sweep had, so there are more than three times as many chances to trip it.
#
# If the first flight aborts with [RASTER POLYGON BREACH] at a pass end, drop this
# to 0.80 (overshoot 0.40-0.64m, still inside 0.70m) rather than widening the
# breach tolerance — the tolerance is a safety layer, the speed is not.
export RASTER_SPEED_MPS=1.00        # was 0.50
export RASTER_TOTAL_TIMEOUT_S=650   # ceiling, not duration.  Whole route (transit +
                                    # 108.3m pattern + return) at 1.00m/s is ~128s
                                    # x1.35 = ~173s, well inside this.

source "${VENV_PATH}/bin/activate"
cd "${PROJECT_PATH}"

# exec: systemd supervises the Python process directly, and stdout/stderr stay
# attached to journald.
exec python -u "${PROJECT_PATH}/ardupilot_raster_mission.py"
