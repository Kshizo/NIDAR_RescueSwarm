#!/bin/bash
set -euo pipefail

# Launcher for the 3 m soft-bounce ArduPilot mission daemon.
VENV_PATH="/home/aahswarm/px4_mavsdk_env"
PROJECT_PATH="/home/aahswarm/ardupilot_testing"

source "${VENV_PATH}/bin/activate"
cd "${PROJECT_PATH}"

exec python "${PROJECT_PATH}/ardupilot_3m_soft_bounce_mission.py"
