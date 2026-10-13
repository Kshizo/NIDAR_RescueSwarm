# Source this: sets Gazebo + ArduPilot paths for the NIDAR sim.
# Override ARDUPILOT_DIR / ARDUPILOT_GAZEBO_DIR before sourcing if yours live elsewhere.
NIDAR_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NIDAR_SIM_DIR
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
export ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$NIDAR_SIM_DIR/generated/models:$NIDAR_SIM_DIR/models:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# gz CLI ships with ROS 2 Jazzy's vendor packages on this machine.
if ! command -v gz >/dev/null 2>&1 && [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi
