#!/usr/bin/env bash
# DEPLOY.md §6 — Stage 4: live floor run.
#
# Robot drives on the floor at cruise_speed under FSM control. Keep a GO
# card in your pocket (arms IDLE, unlocks STOPPED) and a teleop terminal
# open as e-stop.
#
# Usage:
#   tools/qr_stage4_floor.sh                  # default
#   tools/qr_stage4_floor.sh arg:=value       # extra launch args
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=./qr_logging_env.sh
source "$SCRIPT_DIR/qr_logging_env.sh"

_qr_log_command "stage4_floor" -- \
    command ros2 launch qr_nav qr_nav.launch.py "$@"
