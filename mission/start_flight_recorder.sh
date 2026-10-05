#!/bin/bash
set -euo pipefail

VENV_PATH="/home/aahswarm/px4_mavsdk_env"
PROJECT_PATH="/home/aahswarm/NIDAR_RescueSwarm/mission"

source "${VENV_PATH}/bin/activate"
cd "${PROJECT_PATH}"

exec python "${PROJECT_PATH}/flight_recorder.py"
