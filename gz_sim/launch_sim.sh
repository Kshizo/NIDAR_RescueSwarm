#!/usr/bin/env bash
# NIDAR RescueSwarm Mission 1 sim: randomised world + Gazebo + N ArduCopter SITL instances.
#
#   ./launch_sim.sh                 new random world, Gazebo GUI, 2 drones
#   SEED=7 ./launch_sim.sh          reproducible world
#   HEADLESS=1 ./launch_sim.sh      no Gazebo window (sensors still render)
#   NO_REGEN=1 ./launch_sim.sh      reuse generated/ from the last run
#   WORLD_ARGS="--pois 10 --field 316x316" ./launch_sim.sh   full 10 ha search area
#
# Then, in another terminal:  python3 swarm_mission.py
# MAVLink per drone i: tcp:127.0.0.1:(5760+10*i) is the mission's link;
# tcp:127.0.0.1:(5762+10*i) is free for QGroundControl / MAVProxy to watch.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
# shellcheck disable=SC1091
source ./env.sh

DRONES="${DRONES:-2}"
ARDUCOPTER="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
[ -x "$ARDUCOPTER" ] || { echo "Build SITL first: cd $ARDUPILOT_DIR && ./waf configure --board sitl && ./waf copter"; exit 1; }
[ -f "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" ] || { echo "Build ardupilot_gazebo first (see README)"; exit 1; }

# Only one sim at a time: a second one cannot bind the SITL/FDM ports and would
# overwrite generated/ under the first one's feet.
if pgrep -x arducopter >/dev/null || pgrep -f "gz sim server" >/dev/null; then
    echo "[launch] another sim is already running:"
    pgrep -a arducopter | cut -c1-100 || true
    pgrep -af "gz sim" | grep -v pgrep | cut -c1-100 || true
    echo "[launch] stop it first (Ctrl-C in its terminal, or: pkill -INT -x arducopter; pkill -INT -f 'gz sim')"
    exit 1
fi

if [ -z "${NO_REGEN:-}" ]; then
    # shellcheck disable=SC2086
    python3 world_gen.py --drones "$DRONES" ${SEED:+--seed "$SEED"} ${WORLD_ARGS:-}
fi

read -r HOME_LOC < <(python3 -c "
import json; m = json.load(open('generated/mission_area.json'))
o = m['origin']; print(f\"{o['lat']},{o['lon']},{o['alt']},{m['field_heading_deg']}\")")

PIDS=()
cleanup() {
    echo; echo "[launch] stopping sim..."
    # SITL ignores SIGTERM while blocked on the JSON socket, so use SIGINT, then SIGKILL.
    for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    sleep 2
    for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mkdir -p generated/logs
GZ_FLAGS=(-r -v2)
[ -n "${HEADLESS:-}" ] && GZ_FLAGS+=(-s --headless-rendering)
gz sim "${GZ_FLAGS[@]}" generated/nidar_field.sdf > generated/logs/gazebo.log 2>&1 &
PIDS+=($!)
echo "[launch] Gazebo started (log: generated/logs/gazebo.log)"
sleep 3

for ((i = 0; i < DRONES; i++)); do
    dir="generated/sitl_$i"
    mkdir -p "$dir"
    (cd "$dir" && exec "$ARDUCOPTER" --model JSON --speedup 1 --slave 0 -I "$i" --sysid $((i + 1)) \
        --home "$HOME_LOC" --defaults "$NIDAR_SIM_DIR/params/nidar_copter.parm" --wipe \
        --sim-address 127.0.0.1 > "../logs/sitl_$i.log" 2>&1) &
    PIDS+=($!)
    echo "[launch] SITL drone$i  sysid $((i + 1))  mission link tcp:127.0.0.1:$((5760 + 10 * i))  GCS tcp:127.0.0.1:$((5762 + 10 * i))"
done

echo "[launch] home $HOME_LOC. Ctrl-C to stop everything."
echo "[launch] next:  python3 swarm_mission.py"
wait -n "${PIDS[@]}" || true
echo "[launch] a sim process exited early. Last lines of each log:"
for f in generated/logs/gazebo.log generated/logs/sitl_*.log; do
    echo "--- $f"; tail -n 5 "$f"
done
