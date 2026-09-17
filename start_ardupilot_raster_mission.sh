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
PROJECT_PATH="/home/aahswarm/ardupilotPatterenTest"

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
export RASTER_PASS_SPACING_M=1.0
# Speed is limited by overshoot at the pass ends, not by the budget.  A pass-end
# waypoint sits 0.20m inside the polygon and the breach abort fires 0.50m outside
# it, so there is 0.70m of overshoot to spend.  At 0.50m/s the autopilot's stop
# distance plus tracking lag uses ~0.25m, leaving 0.45m.  0.80m/s leaves 0.25m and
# 1.00m/s only 0.10m, which is why this is not raised further without a test flight.
export RASTER_SPEED_MPS=0.50        # was 0.30: 109.8m pattern 366s -> 220s
export RASTER_TOTAL_TIMEOUT_S=650   # ceiling, not duration.  Whole route (transit +
                                    # 109.8m pattern + return) at 0.50m/s is ~270s
                                    # x1.35 = ~365s, well inside this.

source "${VENV_PATH}/bin/activate"
cd "${PROJECT_PATH}"

# exec: systemd supervises the Python process directly, and stdout/stderr stay
# attached to journald.
exec python -u "${PROJECT_PATH}/ardupilot_raster_mission.py"
