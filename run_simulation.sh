#!/bin/bash
# Run the real mission daemons against the simulated flight controller.
#
# Nothing here touches the physical flight controller, the serial link, or the
# frozen reference tree at /home/aahswarm/ardupilot_testing. Every scenario runs
# with its own lock file, log file, trigger file and recordings directory inside a
# scratch work directory, and talks to simulator.py over loopback UDP.
#
#   ./run_simulation.sh                 run every scenario
#   ./run_simulation.sh dryrun          the nine telemetry-only dry-run tests
#   ./run_simulation.sh raster          the seventeen raster flight tests
#   ./run_simulation.sh base            the eight base-mission scenarios
#   ./run_simulation.sh geometry        just the offline-refusal tests (fast)
#   ./run_simulation.sh boundaries      just the outer/inner boundary tests
#   ./run_simulation.sh faults          just the telemetry/altitude/battery tests
#
#   Raster flight tests:
#     budget        TEST 0   configured 0.40 m spacing exceeds the time budget
#     normal        TEST 1   origin inside both polygons, full mission
#     outsideinner  TEST 2   origin outside INNER, inside OUTER: allowed
#     originoutside TEST 3   origin outside OUTER: refuse before motion
#     originmargin  TEST 4   origin inside the 0.50 m outer margin: refuse
#     transitouter  TEST 5   OUTER breach during TRANSIT
#     rasterinner   TEST 6   INNER breach during RASTER
#     rasterouter   TEST 7   OUTER breach during RASTER
#     returnouter   TEST 9   OUTER breach during RETURN
#     stale         TEST 10  LOCAL_POSITION_NED goes stale mid-pattern
#     ekf           TEST 11  EKF loses POS_HORIZ_ABS mid-pattern
#     altlow        TEST 12  altitude below 1.50 m mid-pattern
#     althigh       TEST 13  altitude above 2.50 m mid-pattern
#     battery       TEST 14  battery below the companion floor mid-pattern
#     badouter      TEST 15  invalid OUTER polygon: refuse before the mission
#     badinner      TEST 16  invalid INNER polygon: refuse before the mission
#     notnested     TEST 17  inner partly outside outer: refuse before the mission
#   (TEST 8, return outside INNER but inside OUTER, is covered by TEST 1 and
#    TEST 2: both fly the return leg with inner containment released.)
#
#   Telemetry-only dry-run tests (RASTER_DRY_RUN=1, no flight commands at all):
#     dry1  TEST 18  valid telemetry, disarmed          -> PASS
#     dry2  TEST 19  flight controller ARMED            -> FAIL immediately
#     dry3  TEST 20  no GPS fix                         -> FAIL
#     dry4  TEST 21  LOCAL_POSITION_NED absent/stale    -> FAIL
#     dry5  TEST 22  EKF without POS_HORIZ_ABS          -> FAIL
#     dry6           takeoff point outside the OUTER    -> FAIL
#     dry7           takeoff point inside the margin    -> FAIL
#     dry8           takeoff point outside INNER only   -> PASS
#     dry9           configured 0.40 m spacing          -> FAIL on the budget
#   Each also asserts, from the SIMULATOR's log, that the flight controller
#   received no arm, mode change, takeoff, land, velocity or PARAM_SET.
#
#   Base-mission scenarios (unchanged intent, repaired for the current module):
#     flight, rc, guarded, nothrust, land, takeover, rcloss, fence
#
# Exit status 0 if every check behaved as expected.
#
# Set SIM_WORK_DIR to keep the per-scenario logs after the run.

set -uo pipefail
cd "$(dirname "$0")"

PY="${PY:-/home/aahswarm/px4_mavsdk_env/bin/python}"
PROJECT_DIR="$(pwd)"
BASE_MODULE="ardupilot_horizontal_geofence_mission"
RASTER_SCRIPT="ardupilot_raster_mission.py"

if [ -n "${SIM_WORK_DIR:-}" ]; then
  WORK="$SIM_WORK_DIR"; mkdir -p "$WORK"; KEEP_WORK=1
else
  WORK="$(mktemp -d /tmp/ardupilot_sim.XXXXXX)"; KEEP_WORK=0
fi
PORT_BASE=${PORT_BASE:-14700}
PASSES=0
FAILURES=0
FAILED_SCENARIOS=""

cleanup() { [ "$KEEP_WORK" = 0 ] && rm -rf "$WORK"; }
trap cleanup EXIT

# The simulated aircraft must start inside the mission polygon for a raster test
# to be meaningful, so place it at the polygon's centroid. Computed by the planner
# itself rather than hardcoded, which also proves the planner imports cleanly.
read -r POLY_LAT POLY_LON <<<"$("$PY" -c "
import raster_plan_preview as p
lat, lon = p.centroid_gps(p.plan_raster(p.DEFAULT_CORNERS))
print(f'{lat:.7f} {lon:.7f}')")"
if [ -z "${POLY_LAT:-}" ]; then
  echo "FATAL: could not compute the polygon centroid from raster_plan_preview.py" >&2
  exit 2
fi

echo "Mission logic simulation — no real flight controller is touched."
echo "  work dir        : $WORK"
echo "  polygon centre  : $POLY_LAT, $POLY_LON  (simulated start point)"
echo

# ---------------------------------------------------------------------------
# Scenario plumbing
# ---------------------------------------------------------------------------
# start_sim <name> <port> <motors> [extra sim env...]
start_sim() {
  local name=$1 port=$2 motors=$3; shift 3
  env SIM_CONN="udpout:127.0.0.1:$port" \
      SIM_MOTORS="$motors" \
      CH8_FILE="$WORK/ch8_$name" \
      FENCE_FILE="$WORK/fence_$name" \
      SIM_STALE_LOCAL_FILE="$WORK/stale_$name" \
      SIM_DRIFT_FILE="$WORK/drift_$name" \
      SIM_VZ_DRIFT_FILE="$WORK/vz_$name" \
      SIM_BATT_FILE="$WORK/batt_$name" \
      SIM_EKF_FILE="$WORK/ekf_$name" \
      "$@" \
      "$PY" simulator.py > "$WORK/$name.sim.log" 2>&1 &
  echo $!
}

# base_scenario <name> <port-offset> <motors> <initial-ch8> <duration> [midrun]
#
# video_recorder.RECORDINGS_DIR is a module constant with no env override, and
# video_recorder.py is not ours to modify, so it is repointed at import time by a
# one-line shim. Without it a base-mission scenario would aim its recordings at
# the frozen tree.
base_scenario() {
  local name=$1 offset=$2 motors=$3 ch8=$4 secs=$5 midrun=${6:-}
  local port=$((PORT_BASE + offset))
  local out="$WORK/$name.log"

  echo "$ch8" > "$WORK/ch8_$name"
  echo 0 > "$WORK/fence_$name"
  echo 0 > "$WORK/stale_$name"
  echo 0 > "$WORK/drift_$name"
  echo 0 > "$WORK/vz_$name"
  echo 0 > "$WORK/batt_$name"
  echo 0 > "$WORK/ekf_$name"
  mkdir -p "$WORK/recordings_$name"

  env LOG_FILE="$WORK/$name.file.log" \
      ARDUPILOT_CONNECTION="udp:127.0.0.1:$port" \
      ARDUPILOT_LOCK_FILE="$WORK/$name.lock" \
      MISSION_TRIGGER_FILE="$WORK/TRIGGER_$name" \
      RECORDINGS_DIR="$WORK/recordings_$name/" \
      CONFIGURE_FENCE=0 \
      ORBIT_RADIUS_M="${ORBIT_RADIUS_M:-1.0}" \
      ORBIT_SPEED_MPS="${ORBIT_SPEED_MPS:-0.5}" \
      "$PY" -c "
import os, video_recorder
video_recorder.RECORDINGS_DIR = os.environ['RECORDINGS_DIR']
import ${BASE_MODULE} as m
m.main()
" > "$out" 2>&1 &
  local daemon=$!
  sleep 1
  local sim; sim=$(start_sim "$name" "$port" "$motors")

  if [ -n "$midrun" ]; then eval "$midrun"; fi
  sleep "$secs"

  kill "$daemon" "$sim" 2>/dev/null
  sleep 2
  kill -9 "$daemon" "$sim" 2>/dev/null
  wait "$daemon" "$sim" 2>/dev/null
  echo "$out"
}

# raster_scenario <name> <port-offset> <duration> [midrun]
# Extra environment for both daemon and simulator comes from the RASTER_ENV and
# SIM_ENV arrays set by the caller.
raster_scenario() {
  local name=$1 offset=$2 secs=$3 midrun=${4:-}
  local port=$((PORT_BASE + offset))
  local out="$WORK/$name.log"

  echo 0 > "$WORK/ch8_$name"
  echo 0 > "$WORK/fence_$name"
  echo 0 > "$WORK/stale_$name"
  echo 0 > "$WORK/drift_$name"
  echo 0 > "$WORK/vz_$name"
  echo 0 > "$WORK/batt_$name"
  echo 0 > "$WORK/ekf_$name"
  mkdir -p "$WORK/recordings_$name"

  env LOG_FILE="$WORK/$name.file.log" \
      ARDUPILOT_CONNECTION="udp:127.0.0.1:$port" \
      ARDUPILOT_LOCK_FILE="$WORK/$name.lock" \
      MISSION_TRIGGER_FILE="$WORK/TRIGGER_$name" \
      RECORDINGS_DIR="$WORK/recordings_$name/" \
      CONFIGURE_FENCE=0 \
      RASTER_PASS_SPACING_M=0.40 \
      EDGE_INSET_M=0.20 \
      RASTER_SPEED_MPS=0.20 \
      "${RASTER_ENV[@]}" \
      "$PY" "$RASTER_SCRIPT" > "$out" 2>&1 &
  local daemon=$!
  sleep 1
  local sim; sim=$(start_sim "$name" "$port" 1 \
      SIM_HOME_LAT="$POLY_LAT" SIM_HOME_LON="$POLY_LON" "${SIM_ENV[@]}")

  if [ -n "$midrun" ]; then eval "$midrun"; fi
  sleep "$secs"

  kill "$daemon" "$sim" 2>/dev/null
  sleep 2
  kill -9 "$daemon" "$sim" 2>/dev/null
  wait "$daemon" "$sim" 2>/dev/null
  echo "$out"
}

# dryrun_scenario <name> <port-offset> — runs RASTER_DRY_RUN=1 against the
# simulator and records its exit code.
#
# The simulator is started FIRST here, unlike the flight scenarios: the dry run
# opens the link once and gives up after its own timeout instead of retrying
# forever, so there must be something to hear from when it opens.
#
# Extra environment comes from the DRY_ENV and SIM_ENV arrays set by the caller.
# <stale> presets the SIM_STALE_LOCAL_FILE flag before the simulator starts, so a
# scenario can withhold LOCAL_POSITION_NED from the very first frame.
#
# The exit code is written to a file rather than a variable: this function is
# called through $(...), so anything it assigns happens in a subshell and is lost.
dryrun_scenario() {
  local name=$1 offset=$2 stale=${3:-0}
  local port=$((PORT_BASE + offset))
  local out="$WORK/$name.log"

  echo 0 > "$WORK/ch8_$name"
  echo 0 > "$WORK/fence_$name"
  echo "$stale" > "$WORK/stale_$name"
  echo 0 > "$WORK/drift_$name"
  echo 0 > "$WORK/vz_$name"
  echo 0 > "$WORK/batt_$name"
  echo 0 > "$WORK/ekf_$name"

  local sim; sim=$(start_sim "$name" "$port" 1 \
      SIM_HOME_LAT="$POLY_LAT" SIM_HOME_LON="$POLY_LON" "${SIM_ENV[@]}")
  sleep 1

  env LOG_FILE="$WORK/$name.file.log" \
      ARDUPILOT_CONNECTION="udp:127.0.0.1:$port" \
      ARDUPILOT_LOCK_FILE="$WORK/$name.lock" \
      MISSION_TRIGGER_FILE="$WORK/TRIGGER_$name" \
      RECORDINGS_DIR="$WORK/recordings_$name/" \
      RASTER_DRY_RUN=1 \
      RASTER_DRY_RUN_TIMEOUT_S="${DRY_TIMEOUT:-25}" \
      RASTER_DRY_RUN_SETTLE_S=1.0 \
      RASTER_PASS_SPACING_M=0.40 \
      EDGE_INSET_M=0.20 \
      RASTER_SPEED_MPS=0.20 \
      "${DRY_ENV[@]}" \
      "$PY" "$RASTER_SCRIPT" > "$out" 2>&1
  echo $? > "$WORK/$name.status"

  kill "$sim" 2>/dev/null
  sleep 1
  kill -9 "$sim" 2>/dev/null
  wait "$sim" 2>/dev/null
  echo "$out"
}

# Asserts that a telemetry-only run emitted no flight command of any kind. The
# evidence is the SIMULATOR's log, i.e. what the flight controller actually
# received, not what the mission believes it sent.
check_no_commands() {
  local name=$1
  local simlog="$WORK/$name.sim.log"
  check "no arm/disarm command reached the FC"  "$simlog" reject "ARMED|DISARMED|arm REJECTED"
  check "no mode change reached the FC"         "$simlog" reject "SET_MODE|DO_SET_MODE"
  check "no takeoff command reached the FC"     "$simlog" reject "TAKEOFF"
  check "no PARAM_SET reached the FC"           "$simlog" reject "PARAM_SET"
  check "no velocity/position target reached FC" "$simlog" reject "SET_POSITION_TARGET_LOCAL_NED"
  check "mission sent no LAND/RTL"              "$WORK/$name.log" reject "Setting flight mode"
  check "never ran the autonomous mission"      "$WORK/$name.log" reject "AUTONOMOUS GROUND FLIGHT|STARTING POLYGON RASTER COVERAGE"
  check "never started video recording"         "$WORK/$name.log" reject "Starting onboard video recording"
}

check_status() { # check_status <scenario-name> <label> <expected-exit-code>
  local name=$1 label=$2 want=$3
  local got; got=$(cat "$WORK/$name.status" 2>/dev/null || echo "missing")
  if [ "$got" = "$want" ]; then
    printf '    \033[32mPASS\033[0m  %s\n' "$label"; PASSES=$((PASSES+1))
  else
    printf '    \033[31mFAIL\033[0m  %s  (exit %s, wanted %s)\n' "$label" "$got" "$want"
    FAILURES=$((FAILURES+1))
  fi
}

# wait_for <logfile> <regex> <timeout-seconds>
#
# Blocks until the mission log shows it has reached a given point, then returns.
# Fault injection is triggered off this rather than off a fixed sleep: the phase
# a fault lands in is the whole point of most of these tests, and a sleep that is
# correct on one machine lands in the wrong phase on a slower one.
wait_for() {
  local log=$1 pattern=$2 limit=${3:-90} waited=0
  while [ "$waited" -lt "$limit" ]; do
    [ -f "$log" ] && grep -aqE "$pattern" "$log" && return 0
    sleep 1; waited=$((waited + 1))
  done
  echo "    (wait_for timed out after ${limit}s waiting for: $pattern)" >&2
  return 1
}

check() { # check <label> <logfile> <expect|reject> <pattern>
  local label=$1 log=$2 mode=$3 pattern=$4
  local found=no
  grep -aqE "$pattern" "$log" && found=yes
  if { [ "$mode" = expect ] && [ "$found" = yes ]; } || \
     { [ "$mode" = reject ] && [ "$found" = no ]; }; then
    printf '    \033[32mPASS\033[0m  %s\n' "$label"; PASSES=$((PASSES+1))
  else
    printf '    \033[31mFAIL\033[0m  %s  (%s "%s")\n' "$label" "$mode" "$pattern"
    FAILURES=$((FAILURES+1))
    case " $FAILED_SCENARIOS " in *" $log "*) ;; *) FAILED_SCENARIOS="$FAILED_SCENARIOS $log";; esac
  fi
}

# ===========================================================================
# RASTER TESTS
# ===========================================================================
# ===========================================================================
# RASTER FLIGHT TESTS
# ===========================================================================
# Simulated takeoff positions, all computed from the two configured polygons
# rather than hand-picked, and all re-derivable with raster_plan_preview:
#
#   INSIDE_BOTH    inner centroid          outer +3.90 m, inner +2.14 m
#   OUTSIDE_INNER  outside the survey area outer +3.08 m, inner -0.62 m
#   OUTSIDE_OUTER  outside the geofence    outer -3.04 m
#   IN_MARGIN      inside the 0.50 m band  outer +0.39 m
INSIDE_BOTH=(SIM_HOME_LAT=13.3458462 SIM_HOME_LON=74.7940208)
OUTSIDE_INNER=(SIM_HOME_LAT=13.3458948 SIM_HOME_LON=74.7940342)
OUTSIDE_OUTER=(SIM_HOME_LAT=13.3459545 SIM_HOME_LON=74.7940331)
IN_MARGIN=(SIM_HOME_LAT=13.3459231 SIM_HOME_LON=74.7940331)

# Flight-test fixture. The CONFIGURED mission is 0.40 m spacing, which on this
# 56.8 m^2 area is 33 passes / 140 m / ~700 s of raster alone and is refused by
# the time-budget gate (that refusal is TEST 0). These tests are about the phase
# machine and the two boundaries, not about coverage density, so they run a
# coarser 5.0 m spacing — 3 passes, 20.3 m — with a budget to match. Both are
# explicit fixtures, not changes to the shipped configuration.
FLIGHT_FIXTURE=(RASTER_PASS_SPACING_M=5.0 RASTER_TOTAL_TIMEOUT_S=400)

run_budget() {
  echo "[TEST 0 budget] the CONFIGURED 0.40 m spacing must be refused on the ground"
  RASTER_ENV=()
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario budget 10 45 'sleep 6; touch "$WORK/TRIGGER_budget"')
  check "planned the polygon"                    "$log" expect "POLYGON RASTER MISSION PLAN"
  check "33 passes at 0.40 m spacing"            "$log" expect "Passes: 33"
  check "refused on the time budget"             "$log" expect "RASTER BUDGET. REFUSING TO FLY"
  check "named the pattern as the driver"        "$log" expect "What drives it: 140\.[0-9]m of pattern"
  check "refused to adjust anything itself"      "$log" expect "Nothing is adjusted automatically"
  check "never started the transit"              "$log" reject "RASTER PHASE. TRANSIT_TO_RASTER"
  check "never flew a pass"                      "$log" reject "pass 1/"
  check "handed back to the base safety landing" "$log" expect "EXECUTING SAFETY LAND"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_normal() {
  echo "[TEST 1 normal] origin inside both polygons: transit -> raster -> return -> land"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario normal 11 300 'sleep 6; touch "$WORK/TRIGGER_normal"')
  check "validated the outer geofence"           "$log" expect "Outer safety geofence: area 204\."
  check "confirmed inner inside outer"           "$log" expect "Inner inside outer: minimum separation 1\.5"
  check "did not refuse to start"                "$log" reject "REFUSING TO START"
  check "took off to the 2.0 m target"           "$log" expect "target 2\.0m|target 2\.0 m"
  check "held the mission altitude"              "$log" expect "Target altitude reached and stable"
  check "anchored GPS to local NED"              "$log" expect "RASTER ANCHOR. GPS"
  check "projected the outer geofence"           "$log" expect "Outer safety geofence projected"
  check "checked the origin against the outer"   "$log" expect "RASTER ORIGIN CHECK"
  check "checked the whole route"                "$log" expect "OUTER GEOFENCE CHECK.*points and"
  check "route fitted the outer geofence"        "$log" reject "OUTER GEOFENCE CHECK. REFUSING"
  check "budget accepted under the fixture"      "$log" reject "RASTER BUDGET. REFUSING"
  check "ran TRANSIT_TO_RASTER"                  "$log" expect "RASTER PHASE. TRANSIT_TO_RASTER"
  check "inner not enforced during transit"      "$log" expect "Inner-polygon containment is not enforced"
  check "enabled inner containment for raster"   "$log" expect "polygon containment is now ENFORCED"
  check "flew pass 1"                            "$log" expect "pass 1/3"
  check "flew pass 3"                            "$log" expect "pass 3/3"
  check "completed every pass"                   "$log" expect "All 3 passes complete"
  check "ran RETURN_TO_ORIGIN"                   "$log" expect "RASTER PHASE. RETURN_TO_ORIGIN"
  check "released inner for the return"          "$log" expect "containment released"
  check "returned to the captured origin"        "$log" expect "Pattern complete; back within"
  check "landed and disarmed"                    "$log" expect "Touchdown confirmed"
  check "mission reported success"               "$log" expect "Mission cycle finished successfully"
  check "no inner breach"                        "$log" reject "RASTER POLYGON BREACH"
  check "no outer breach"                        "$log" reject "OUTER GEOFENCE BREACH"
  check "no altitude abort"                      "$log" reject "RASTER ALTITUDE"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_outsideinner() {
  echo "[TEST 2 outsideinner] origin OUTSIDE inner, safely inside outer: must be allowed"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${OUTSIDE_INNER[@]}")
  local log
  log=$(raster_scenario outsideinner 12 300 'sleep 6; touch "$WORK/TRIGGER_outsideinner"')
  check "saw the origin outside the inner"       "$log" expect "outside the inner mission polygon"
  check "said that is allowed"                   "$log" expect "enforced during the RASTER phase only"
  check "origin check passed on the outer"       "$log" reject "RASTER ORIGIN CHECK. REFUSING"
  check "did NOT refuse the mission"             "$log" reject "REFUSING TO FLY"
  check "transited into the area"                "$log" expect "polygon containment is now ENFORCED"
  check "flew every pass"                        "$log" expect "All 3 passes complete"
  check "returned outside the inner again"       "$log" expect "RASTER PHASE. RETURN_TO_ORIGIN"
  check "returned to the captured origin"        "$log" expect "Pattern complete; back within"
  check "landed and disarmed"                    "$log" expect "Touchdown confirmed"
  check "no inner breach"                        "$log" reject "RASTER POLYGON BREACH"
  check "no outer breach"                        "$log" reject "OUTER GEOFENCE BREACH"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_originoutside() {
  echo "[TEST 3 originoutside] origin OUTSIDE the outer polygon: refuse before any motion"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${OUTSIDE_OUTER[@]}")
  local log
  log=$(raster_scenario originoutside 13 90 'sleep 6; touch "$WORK/TRIGGER_originoutside"')
  check "anchored before deciding"               "$log" expect "RASTER ANCHOR. GPS"
  check "ran the origin check"                   "$log" expect "RASTER ORIGIN CHECK"
  check "refused the origin"                     "$log" expect "RASTER ORIGIN CHECK. REFUSING TO FLY"
  check "said it is outside the outer polygon"   "$log" expect "OUTSIDE the outer safety polygon"
  check "refused to fly into the area"           "$log" expect "will NOT be flown into the allowed area"
  check "refused to move the geofence"           "$log" expect "geofence is NOT moved"
  check "never started the transit"              "$log" reject "RASTER PHASE. TRANSIT_TO_RASTER"
  check "never flew a pass"                      "$log" reject "pass 1/"
  check "handed back to the base safety landing" "$log" expect "EXECUTING SAFETY LAND"
  check "landed and disarmed"                    "$log" expect "Touchdown confirmed on ground and aircraft disarmed"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_originmargin() {
  echo "[TEST 4 originmargin] origin inside the raw outer but within the 0.50 m margin: refuse"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${IN_MARGIN[@]}")
  local log
  log=$(raster_scenario originmargin 14 90 'sleep 6; touch "$WORK/TRIGGER_originmargin"')
  check "ran the origin check"                   "$log" expect "RASTER ORIGIN CHECK"
  check "refused the origin"                     "$log" expect "RASTER ORIGIN CHECK. REFUSING TO FLY"
  check "named the safety margin"                "$log" expect "inside the 0\.50m safety margin"
  check "did NOT claim it was outside"           "$log" reject "OUTSIDE the outer safety polygon"
  check "never started the transit"              "$log" reject "RASTER PHASE. TRANSIT_TO_RASTER"
  check "never flew a pass"                      "$log" reject "pass 1/"
  check "handed back to the base safety landing" "$log" expect "EXECUTING SAFETY LAND"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_transitouter() {
  echo "[TEST 5 transitouter] wind carries it across the OUTER polygon during TRANSIT"
  # 0.6 m/s against a 0.20 m/s cap: the aircraft cannot make headway and is
  # carried steadily outward. The inner allowance is opened to 20 m so boundary 1
  # cannot fire first and mask boundary 2 — a fixture, not a flight setting.
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}" RASTER_POLYGON_BREACH_MARGIN_M=20.0)
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_DRIFT_E_MPS=0.6)
  local log
  log=$(raster_scenario transitouter 15 200 \
        'echo 1 > "$WORK/drift_transitouter"; sleep 6; touch "$WORK/TRIGGER_transitouter"')
  check "entered the transit phase"              "$log" expect "RASTER PHASE. TRANSIT_TO_RASTER"
  check "inner layer deliberately widened"       "$log" expect "excursion 20\.00m"
  check "inner layer did NOT fire"               "$log" reject "RASTER POLYGON BREACH"
  check "detected the outer breach"              "$log" expect "OUTER GEOFENCE BREACH"
  check "stopped horizontal motion"              "$log" expect "Horizontal motion stopped"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "landed and disarmed"                    "$log" expect "Touchdown confirmed on ground and aircraft disarmed"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_rasterinner() {
  echo "[TEST 6 rasterinner] aircraft leaves the INNER polygon during RASTER"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_DRIFT_E_MPS=0.5)
  local log
  log=$(raster_scenario rasterinner 16 240 \
        'sleep 6; touch "$WORK/TRIGGER_rasterinner";
         wait_for "$WORK/rasterinner.log" "containment is now ENFORCED" 150;
         echo 1 > "$WORK/drift_rasterinner"')
  check "reached the raster phase"               "$log" expect "polygon containment is now ENFORCED"
  check "detected the inner breach"              "$log" expect "RASTER POLYGON BREACH"
  check "reported how far outside"               "$log" expect "outside the mission polygon"
  check "inner fired before the outer"           "$log" reject "OUTER GEOFENCE BREACH"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "landed and disarmed"                    "$log" expect "Touchdown confirmed on ground and aircraft disarmed"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_rasterouter() {
  echo "[TEST 7 rasterouter] aircraft leaves the OUTER polygon during RASTER"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}" RASTER_POLYGON_BREACH_MARGIN_M=20.0)
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_DRIFT_E_MPS=0.5)
  local log
  log=$(raster_scenario rasterouter 17 240 \
        'sleep 6; touch "$WORK/TRIGGER_rasterouter";
         wait_for "$WORK/rasterouter.log" "containment is now ENFORCED" 150;
         echo 1 > "$WORK/drift_rasterouter"')
  check "reached the raster phase"               "$log" expect "polygon containment is now ENFORCED"
  check "inner layer deliberately widened"       "$log" expect "excursion 20\.00m"
  check "inner layer did NOT fire"               "$log" reject "RASTER POLYGON BREACH"
  check "detected the outer breach"              "$log" expect "OUTER GEOFENCE BREACH"
  check "stopped horizontal motion"              "$log" expect "Horizontal motion stopped"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_returnouter() {
  echo "[TEST 9 returnouter] wind carries it across the OUTER polygon during RETURN"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}" RASTER_POLYGON_BREACH_MARGIN_M=20.0)
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_DRIFT_E_MPS=0.6)
  local log
  log=$(raster_scenario returnouter 18 300 \
        'sleep 6; touch "$WORK/TRIGGER_returnouter";
         wait_for "$WORK/returnouter.log" "RETURN_TO_ORIGIN" 220;
         echo 1 > "$WORK/drift_returnouter"')
  check "completed the pattern first"            "$log" expect "All 3 passes complete"
  check "reached the return phase"               "$log" expect "RASTER PHASE. RETURN_TO_ORIGIN"
  check "inner released for the return"          "$log" expect "containment released"
  check "detected the outer breach"              "$log" expect "OUTER GEOFENCE BREACH"
  check "stopped horizontal motion"              "$log" expect "Horizontal motion stopped"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_stale() {
  echo "[TEST 10 stale] LOCAL_POSITION_NED stops arriving during the horizontal phase"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario stale 19 240 \
        'sleep 6; touch "$WORK/TRIGGER_stale";
         wait_for "$WORK/stale.log" "containment is now ENFORCED" 150;
         echo 1 > "$WORK/stale_stale"')
  check "reached the raster phase"               "$log" expect "polygon containment is now ENFORCED"
  check "detected the stale position"            "$log" expect "LOCAL POSITION STALE"
  check "aborted the leg"                        "$log" expect "stopped by a safety check"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_ekf() {
  echo "[TEST 11 ekf] EKF loses POS_HORIZ_ABS during the horizontal phase"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario ekf 20 240 \
        'sleep 6; touch "$WORK/TRIGGER_ekf";
         wait_for "$WORK/ekf.log" "containment is now ENFORCED" 150;
         echo 167 > "$WORK/ekf_ekf"')
  check "reached the raster phase"               "$log" expect "polygon containment is now ENFORCED"
  check "detected the EKF fault"                 "$log" expect "EKF UNHEALTHY"
  check "aborted the leg"                        "$log" expect "stopped by a safety check"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_altlow() {
  echo "[TEST 12 altlow] altitude sinks below 1.50 m during the horizontal phase"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_VZ_DRIFT_MPS=-0.20)
  local log
  log=$(raster_scenario altlow 21 240 \
        'sleep 6; touch "$WORK/TRIGGER_altlow";
         wait_for "$WORK/altlow.log" "containment is now ENFORCED" 150;
         echo 1 > "$WORK/vz_altlow"')
  check "climbed to 2 m first"                   "$log" expect "Target altitude reached and stable"
  check "reached the raster phase"               "$log" expect "polygon containment is now ENFORCED"
  check "detected the low altitude"              "$log" expect "RASTER ALTITUDE"
  check "named the band"                         "$log" expect "1\.50-2\.50m horizontal-flight band"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_althigh() {
  echo "[TEST 13 althigh] altitude climbs above 2.50 m during the horizontal phase"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_VZ_DRIFT_MPS=0.20)
  local log
  log=$(raster_scenario althigh 22 240 \
        'sleep 6; touch "$WORK/TRIGGER_althigh";
         wait_for "$WORK/althigh.log" "containment is now ENFORCED" 150;
         echo 1 > "$WORK/vz_althigh"')
  check "climbed to 2 m first"                   "$log" expect "Target altitude reached and stable"
  check "reached the raster phase"               "$log" expect "polygon containment is now ENFORCED"
  check "detected the high altitude"             "$log" expect "RASTER ALTITUDE"
  check "named the band"                         "$log" expect "1\.50-2\.50m horizontal-flight band"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_battery() {
  echo "[TEST 14 battery] battery falls below the companion floor during the pattern"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario battery 23 240 \
        'sleep 6; touch "$WORK/TRIGGER_battery";
         wait_for "$WORK/battery.log" "containment is now ENFORCED" 150;
         echo 9 > "$WORK/batt_battery"')
  check "reached the raster phase"               "$log" expect "polygon containment is now ENFORCED"
  check "detected the low battery"               "$log" expect "RASTER BATTERY"
  check "named the companion floor"              "$log" expect "below the 25% companion minimum"
  check "invoked the senior safety landing"      "$log" expect "EXECUTING SAFETY LAND"
  check "did NOT complete the pattern"           "$log" reject "All 3 passes complete"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_badouter() {
  echo "[TEST 15 badouter] invalid OUTER polygon: refuse before the mission"
  # G2 and G3 swapped: the order crosses a diagonal instead of walking the
  # perimeter, so G1->G2->G3->G4 is a bow-tie.
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}"
              "OUTER_GEOFENCE_CORNERS=13.345914,74.793969;13.345743,74.794098;13.345728,74.794010;13.345931,74.794055")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario badouter 24 25 'sleep 4; touch "$WORK/TRIGGER_badouter"')
  check "refused to start"                       "$log" expect "REFUSING TO START"
  check "named the outer geofence"               "$log" expect "outer safety geofence is (un|not )usable"
  check "named the geometric reason"             "$log" expect "not convex and simple"
  check "never connected to the FC"              "$log" reject "Connected to ArduPilot"
  check "never armed"                            "$log" reject "confirmed ARMED"
  check "never commanded takeoff"                "$log" reject "MAV_CMD_NAV_TAKEOFF"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_badinner() {
  echo "[TEST 16 badinner] invalid INNER polygon: refuse before the mission"
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}"
              "RASTER_CORNERS=13.345786,74.794012;13.345911,74.794024;13.345794,74.794055;13.345902,74.793990")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario badinner 25 25 'sleep 4; touch "$WORK/TRIGGER_badinner"')
  check "refused to start"                       "$log" expect "REFUSING TO START"
  check "named the geometric reason"             "$log" expect "not convex and simple|does not wind consistently"
  check "said no connection was opened"          "$log" expect "No flight controller connection has been opened"
  check "never connected to the FC"              "$log" reject "Connected to ArduPilot"
  check "never armed"                            "$log" reject "confirmed ARMED"
  check "no exceptions"                          "$log" reject "Traceback"
}

run_notnested() {
  echo "[TEST 17 notnested] inner polygon partly OUTSIDE the outer: refuse before the mission"
  # The outer polygon shifted 3 m east, so inner corners A and D stick out of it.
  RASTER_ENV=("${FLIGHT_FIXTURE[@]}"
              "OUTER_GEOFENCE_CORNERS=13.345914,74.7939967;13.345728,74.7940377;13.345743,74.7941257;13.345931,74.7940827")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log
  log=$(raster_scenario notnested 26 25 'sleep 4; touch "$WORK/TRIGGER_notnested"')
  check "refused to start"                       "$log" expect "REFUSING TO START"
  check "named the containment failure"          "$log" expect "not safely contained by the outer"
  check "named an offending corner"              "$log" expect "inner corner [AD] is .* OUTSIDE the outer"
  check "never connected to the FC"              "$log" reject "Connected to ArduPilot"
  check "never armed"                            "$log" reject "confirmed ARMED"
  check "no exceptions"                          "$log" reject "Traceback"
}

# ===========================================================================
# BASE-MISSION SCENARIOS
# ===========================================================================
run_flight() {
  echo "[flight] full autonomous base mission, triggered by file"
  local log
  log=$(base_scenario flight 0 1 0 95 'sleep 6; touch "$WORK/TRIGGER_flight"')
  check "entered GUIDED and armed"        "$log" expect "confirmed ARMED"
  check "takeoff accepted"                "$log" expect "Takeoff command ACCEPTED"
  check "left the ground"                 "$log" expect "Liftoff detected"
  check "held target altitude"            "$log" expect "Target altitude reached and stable"
  check "captured the fixed origin"       "$log" expect "Fixed origin captured"
  check "flew the horizontal orbit"       "$log" expect "orbit"
  check "returned to the fixed origin"    "$log" expect "Returning to the fixed horizontal origin"
  check "soft landing completed"          "$log" expect "Touchdown confirmed"
  check "mission reported success"        "$log" expect "Mission cycle finished successfully"
  check "no exceptions"                   "$log" reject "Traceback"
}

run_rc() {
  echo "[rc] CH8 moved MANUAL -> GUIDED starts a mission"
  local log
  log=$(base_scenario rc 1 1 1000 46 'sleep 7; echo 1500 > "$WORK/ch8_rc"')
  check "trigger armed by MANUAL"         "$log" expect "CH8 trigger armed"
  check "mission started on the move"     "$log" expect "CH8=GUIDED on GROUND -> Starting"
  check "reached target altitude"          "$log" expect "Target altitude reached and stable"
  check "no exceptions"                   "$log" reject "Traceback"
}

run_guarded() {
  echo "[guarded] CH8 already in GUIDED at startup must NOT fly"
  local log
  log=$(base_scenario guarded 2 1 1500 22)
  check "refused the stale switch"        "$log" expect "switch was already there"
  check "did NOT arm"                     "$log" reject "confirmed ARMED"
  check "did NOT take off"                "$log" reject "Liftoff detected"
}

run_nothrust() {
  echo "[nothrust] armed but producing no climb"
  local log
  log=$(base_scenario nothrust 3 0 0 30 'sleep 6; touch "$WORK/TRIGGER_nothrust"')
  check "armed"                           "$log" expect "confirmed ARMED"
  check "detected no liftoff"             "$log" expect "NO LIFTOFF"
  check "disarmed instead of idling"      "$log" expect "confirmed DISARMED"
  check "no exceptions"                   "$log" reject "Traceback"
}

run_land() {
  echo "[land] CH8 flicked to LAND while airborne -> safety land"
  local log
  log=$(base_scenario land 4 1 1000 52 'sleep 7; echo 1500 > "$WORK/ch8_land"; sleep 9; echo 1900 > "$WORK/ch8_land"')
  check "took off first"                  "$log" expect "Liftoff detected"
  check "saw the LAND command"            "$log" expect "CH8 LAND|CH8=LAND while airborne"
  check "executed a safety land"          "$log" expect "EXECUTING SAFETY LAND"
  check "landed and disarmed"             "$log" expect "Touchdown confirmed on ground and aircraft disarmed"
  check "no exceptions"                   "$log" reject "Traceback"
}

run_takeover() {
  echo "[takeover] pilot takes over mid-flight (CH8 -> MANUAL)"
  local log
  log=$(base_scenario takeover 5 1 1000 52 'sleep 7; echo 1500 > "$WORK/ch8_takeover"; sleep 9; echo 1000 > "$WORK/ch8_takeover"')
  check "took off first"                  "$log" expect "Liftoff detected"
  check "detected pilot takeover"         "$log" expect "CH8 MANUAL. Pilot takeover"
  check "yielded to the pilot"            "$log" expect "Yielding control|MANUAL FLIGHT"
  check "no exceptions"                   "$log" reject "Traceback"
}

run_rcloss() {
  echo "[rcloss] RC link lost mid-flight -> abort and land"
  local log
  log=$(base_scenario rcloss 6 1 1000 52 'sleep 7; echo 1500 > "$WORK/ch8_rcloss"; sleep 9; echo 0 > "$WORK/ch8_rcloss"')
  check "took off first"                  "$log" expect "Liftoff detected"
  check "detected the RC loss"            "$log" expect "RC LOST"
  check "landed rather than continuing"   "$log" expect "SAFETY LAND|LAND"
  check "no exceptions"                   "$log" reject "Traceback"
}

run_fence() {
  echo "[fence] ArduPilot geofence breach while airborne -> land in place"
  local log
  log=$(base_scenario fence 7 1 0 52 'sleep 6; touch "$WORK/TRIGGER_fence"; sleep 10; echo 1 > "$WORK/fence_fence"')
  check "took off first"                  "$log" expect "Liftoff detected"
  check "detected the breach"             "$log" expect "GEOFENCE.*Breach detected"
  check "landed in place, no RTL"         "$log" expect "GEOFENCE.*Commanding soft LAND|BREACH — EXECUTING SAFE LAND"
  check "no exceptions"                   "$log" reject "Traceback"
}

# ===========================================================================
# TELEMETRY-ONLY DRY-RUN TESTS  (RASTER_DRY_RUN=1)
# ===========================================================================
# Every one of these asserts the same thing at the end: the flight controller
# received no arm, no mode change, no takeoff, no land, no velocity/position
# target and no PARAM_SET. That assertion reads the SIMULATOR's log, so it is
# evidence about what arrived at the autopilot rather than a claim by the code
# under test.
run_dry1() {
  echo "[TEST 18 dry1] valid telemetry, disarmed, origin inside both -> PASS"
  local NAME=dry1
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log; log=$(dryrun_scenario dry1 30)
  check_status "$NAME" "exited 0 (PASS)" 0
  check "announced telemetry-only mode"    "$log" expect "TELEMETRY ONLY, NO FLIGHT COMMANDS"
  check "requested stream rates only"      "$log" expect "MAV_CMD_SET_MESSAGE_INTERVAL"
  # CURRENT AIRCRAFT
  check "reported system/component ID"     "$log" expect "system ID +: 1"
  check "reported the flight mode"         "$log" expect "flight mode +:"
  check "reported disarmed"                "$log" expect "armed +: False"
  check "reported GPS fix"                 "$log" expect "GPS fix type +: 3"
  check "reported satellites"              "$log" expect "satellites visible +: 16"
  check "reported latitude/longitude"      "$log" expect "latitude +: 13\."
  check "reported LOCAL_POSITION_NED"      "$log" expect "LOCAL_POSITION_NED +: north="
  check "reported EKF"                     "$log" expect "EKF flags +: 831"
  check "reported battery"                 "$log" expect "battery +:"
  check "reported telemetry ages"          "$log" expect "telemetry ages:"
  # ANCHOR
  check "captured the runtime anchor"      "$log" expect "GPS <-> LOCAL_POSITION_NED RUNTIME ANCHOR"
  check "anchor GPS"                       "$log" expect "anchor_lat +: 13\."
  check "anchor local N/E"                 "$log" expect "anchor_local_north +:"
  check "anchor sample skew"               "$log" expect "sample skew +:"
  # OUTER
  check "projected the outer geofence"     "$log" expect "OUTER SAFETY GEOFENCE IN THE REAL"
  check "listed G1-G4"                     "$log" expect "G4 +13\."
  check "outer area"                       "$log" expect "area +: 204\.[0-9]+ m\^2"
  check "outer side lengths"               "$log" expect "side lengths +: G1G2="
  check "outer winding"                    "$log" expect "winding \(as stored\)"
  check "effective 0.50 m boundary"        "$log" expect "inward safety margin : 0\.50 m"
  check "aircraft clearance to outer"      "$log" expect "aircraft clearance +:"
  # INNER
  check "projected the inner polygon"      "$log" expect "INNER RASTER POLYGON CORNERS"
  check "inner area"                       "$log" expect "area +: 56\.[0-9]+ m\^2"
  check "inner side lengths"               "$log" expect "sides +: AB=4\.7"
  check "pass and waypoint count"          "$log" expect "raster passes +: 3 at .*\(6 waypoints\)"
  check "path length"                      "$log" expect "raster path length +: 20\."
  check "expected flight time"             "$log" expect "est\. horizontal time:"
  # RELATIONSHIP
  check "inner/outer relationship"         "$log" expect "INNER / OUTER RELATIONSHIP"
  check "separation at A/B/C/D"            "$log" expect "inner corner D:"
  check "minimum separation"               "$log" expect "minimum inner-to-outer separation +: 1\.5"
  check "separation after outer margin"    "$log" expect "after the 0\.50 m outer safety margin"
  check "inner breach tolerance printed"   "$log" expect "inner runtime breach tolerance +: 0\.50 m"
  check "worst remaining separation"       "$log" expect "worst remaining separation +: 0\.5"
  check "confirmed inner inside outer"     "$log" expect "inner polygon completely inside outer +: YES"
  # TAKEOFF / HOME
  check "takeoff suitability"              "$log" expect "current position as a takeoff point : SUITABLE"
  check "distance to first waypoint"       "$log" expect "distance to the first raster waypoint"
  check "farthest mission point"           "$log" expect "farthest mission point"
  check "whole-route containment"          "$log" expect "OUTER GEOFENCE CHECK.*points and"
  check "time budget"                      "$log" expect "TIME BUDGET"
  check "overall PASS"                     "$log" expect "RASTER DRY RUN: PASS"
  check "closed the link"                  "$log" expect "MAVLink connection closed"
  check "no exceptions"                    "$log" reject "Traceback"
  check_no_commands dry1
}

run_dry2() {
  echo "[TEST 19 dry2] flight controller reports ARMED -> immediate FAIL, no commands"
  local NAME=dry2
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_START_ARMED=1)
  local log; log=$(dryrun_scenario dry2 31)
  check_status "$NAME" "exited 1 (FAIL)" 1
  check "raised the armed abort"           "$log" expect "THE FLIGHT CONTROLLER IS ARMED"
  check "said no command was sent"         "$log" expect "No command of any kind was sent"
  check "closed the link"                  "$log" expect "MAVLink connection closed"
  check "computed no geometry"             "$log" reject "RUNTIME ANCHOR"
  check "did not report PASS"              "$log" reject "RASTER DRY RUN: PASS"
  check "no exceptions"                    "$log" reject "Traceback"
  check_no_commands dry2
}

run_dry3() {
  echo "[TEST 20 dry3] no GPS fix -> FAIL"
  local NAME=dry3
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_GPS_FIX=0 SIM_SATS=0)
  local log; log=$(dryrun_scenario dry3 32)
  check_status "$NAME" "exited 1 (FAIL)" 1
  check "reported the preconditions failed" "$log" expect "DRY RUN FAIL — telemetry preconditions"
  check "named the GPS fix"                 "$log" expect "GPS fix type 0 is below the required 3"
  check "computed no geometry"              "$log" reject "RUNTIME ANCHOR"
  check "did not report PASS"               "$log" reject "RASTER DRY RUN: PASS"
  check "no exceptions"                     "$log" reject "Traceback"
  check_no_commands dry3
}

run_dry4() {
  echo "[TEST 21 dry4] LOCAL_POSITION_NED absent/stale -> FAIL"
  local NAME=dry4
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log; log=$(dryrun_scenario dry4 33 1)
  check_status "$NAME" "exited 1 (FAIL)" 1
  check "reported the preconditions failed" "$log" expect "DRY RUN FAIL — telemetry preconditions"
  check "named the local position"          "$log" expect "no fresh LOCAL_POSITION_NED"
  check "showed it was never received"      "$log" expect "LOCAL_POSITION_NED +never received"
  check "computed no geometry"              "$log" reject "RUNTIME ANCHOR"
  check "did not report PASS"               "$log" reject "RASTER DRY RUN: PASS"
  check "no exceptions"                     "$log" reject "Traceback"
  check_no_commands dry4
}

run_dry5() {
  echo "[TEST 22 dry5] EKF without POS_HORIZ_ABS -> FAIL"
  local NAME=dry5
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${INSIDE_BOTH[@]}" SIM_EKF_FLAGS=167)
  local log; log=$(dryrun_scenario dry5 34)
  check_status "$NAME" "exited 1 (FAIL)" 1
  check "reported the preconditions failed" "$log" expect "DRY RUN FAIL — telemetry preconditions"
  check "named the EKF"                     "$log" expect "EKF does not report POS_HORIZ_ABS"
  check "explained the in-flight effect"    "$log" expect "check_autonomous_abort would abort"
  check "computed no geometry"              "$log" reject "RUNTIME ANCHOR"
  check "did not report PASS"               "$log" reject "RASTER DRY RUN: PASS"
  check "no exceptions"                     "$log" reject "Traceback"
  check_no_commands dry5
}

run_dry6() {
  echo "[DRY 6] takeoff point OUTSIDE the outer polygon -> FAIL, geofence not moved"
  local NAME=dry6
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${OUTSIDE_OUTER[@]}")
  local log; log=$(dryrun_scenario dry6 35)
  check_status "$NAME" "exited 1 (FAIL)" 1
  check "still captured the anchor"         "$log" expect "RUNTIME ANCHOR"
  check "still printed the geometry"        "$log" expect "INNER / OUTER RELATIONSHIP"
  check "marked the position unsuitable"    "$log" expect "as a takeoff point : NOT SUITABLE"
  check "said it is outside the outer"      "$log" expect "OUTSIDE the outer"
  check "refused to move the geofence"      "$log" expect "geofence is NOT moved"
  check "overall FAIL"                      "$log" expect "RASTER DRY RUN: FAIL"
  check "no exceptions"                     "$log" reject "Traceback"
  check_no_commands dry6
}

run_dry7() {
  echo "[DRY 7] takeoff point inside the 0.50 m outer margin -> FAIL"
  local NAME=dry7
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${IN_MARGIN[@]}")
  local log; log=$(dryrun_scenario dry7 36)
  check_status "$NAME" "exited 1 (FAIL)" 1
  check "marked the position unsuitable"    "$log" expect "as a takeoff point : NOT SUITABLE"
  check "named the safety margin"           "$log" expect "inside the 0\.50 m"
  check "did NOT claim it was outside"      "$log" reject "is .* m OUTSIDE the outer"
  check "overall FAIL"                      "$log" expect "RASTER DRY RUN: FAIL"
  check "no exceptions"                     "$log" reject "Traceback"
  check_no_commands dry7
}

run_dry8() {
  echo "[DRY 8] takeoff point outside the INNER but inside the OUTER -> PASS"
  local NAME=dry8
  DRY_ENV=("${FLIGHT_FIXTURE[@]}")
  SIM_ENV=("${OUTSIDE_INNER[@]}")
  local log; log=$(dryrun_scenario dry8 37)
  check_status "$NAME" "exited 0 (PASS)" 0
  check "saw it outside the inner polygon"  "$log" expect "inside the inner raster polygon +: no"
  check "said that is allowed"              "$log" expect "TRANSIT_TO_RASTER exists"
  check "marked the position suitable"      "$log" expect "as a takeoff point : SUITABLE"
  check "listed the three phases"           "$log" expect "MISSION PHASES FROM THIS POSITION"
  check "overall PASS"                      "$log" expect "RASTER DRY RUN: PASS"
  check "no exceptions"                     "$log" reject "Traceback"
  check_no_commands dry8
}

run_dry9() {
  echo "[DRY 9] the CONFIGURED 0.40 m spacing must FAIL the dry-run time budget"
  local NAME=dry9
  DRY_ENV=()
  SIM_ENV=("${INSIDE_BOTH[@]}")
  local log; log=$(dryrun_scenario dry9 38)
  check_status "$NAME" "exited 1 (FAIL)" 1
  check "reported the geometry first"       "$log" expect "INNER / OUTER RELATIONSHIP"
  check "reported 33 passes"                "$log" expect "raster passes +: 33"
  check "reported the time budget"          "$log" expect "TIME BUDGET"
  check "failed on the budget"              "$log" expect "cannot finish inside the"
  check "named the driver"                  "$log" expect "Driver: 140\."
  check "refused to adjust anything"        "$log" expect "Nothing is adjusted automatically"
  check "overall FAIL"                      "$log" expect "RASTER DRY RUN: FAIL"
  check "no exceptions"                     "$log" reject "Traceback"
  check_no_commands dry9
}

RASTER_ENV=()
SIM_ENV=()
DRY_ENV=()

case "${1:-all}" in
  budget)        run_budget ;;
  normal)        run_normal ;;
  outsideinner)  run_outsideinner ;;
  originoutside) run_originoutside ;;
  originmargin)  run_originmargin ;;
  transitouter)  run_transitouter ;;
  rasterinner)   run_rasterinner ;;
  rasterouter)   run_rasterouter ;;
  returnouter)   run_returnouter ;;
  stale)         run_stale ;;
  ekf)           run_ekf ;;
  altlow)        run_altlow ;;
  althigh)       run_althigh ;;
  battery)       run_battery ;;
  badouter)      run_badouter ;;
  badinner)      run_badinner ;;
  notnested)     run_notnested ;;
  dry1)          run_dry1 ;;
  dry2)          run_dry2 ;;
  dry3)          run_dry3 ;;
  dry4)          run_dry4 ;;
  dry5)          run_dry5 ;;
  dry6)          run_dry6 ;;
  dry7)          run_dry7 ;;
  dry8)          run_dry8 ;;
  dry9)          run_dry9 ;;
  geometry)      run_badouter; echo; run_badinner; echo; run_notnested; echo; run_budget ;;
  boundaries)    run_originoutside; echo; run_originmargin; echo; run_transitouter; echo
                 run_rasterinner; echo; run_rasterouter; echo; run_returnouter ;;
  faults)        run_stale; echo; run_ekf; echo; run_altlow; echo; run_althigh; echo
                 run_battery ;;
  dryrun)        run_dry1; echo; run_dry2; echo; run_dry3; echo; run_dry4; echo
                 run_dry5; echo; run_dry6; echo; run_dry7; echo; run_dry8; echo
                 run_dry9 ;;
  raster)        run_budget; echo; run_normal; echo; run_outsideinner; echo
                 run_originoutside; echo; run_originmargin; echo; run_transitouter; echo
                 run_rasterinner; echo; run_rasterouter; echo; run_returnouter; echo
                 run_stale; echo; run_ekf; echo; run_altlow; echo; run_althigh; echo
                 run_battery; echo; run_badouter; echo; run_badinner; echo
                 run_notnested ;;
  flight)        run_flight ;;
  rc)            run_rc ;;
  guarded)       run_guarded ;;
  nothrust)      run_nothrust ;;
  land)          run_land ;;
  takeover)      run_takeover ;;
  rcloss)        run_rcloss ;;
  fence)         run_fence ;;
  safety)        run_land; echo; run_takeover; echo; run_rcloss; echo; run_fence ;;
  base)          run_flight; echo; run_rc; echo; run_guarded; echo; run_nothrust; echo
                 run_land; echo; run_takeover; echo; run_rcloss; echo; run_fence ;;
  all)           "$0" dryrun && "$0" raster && "$0" base; exit $? ;;
  *) echo "unknown scenario: $1" >&2; exit 2 ;;
esac

echo
echo "-------------------------------------------"
if [ "$FAILURES" -eq 0 ]; then
  echo "ALL CHECKS PASSED  ($PASSES/$PASSES)"
  exit 0
fi
echo "CHECKS FAILED  ($PASSES passed, $FAILURES failed)"
echo "logs with failures:$FAILED_SCENARIOS"
[ "$KEEP_WORK" = 1 ] && echo "work dir kept at $WORK"
exit 1
