#!/usr/bin/env bash
# DEPLOY.md §5 — Stage 3: full launch with wheels off the ground.
#
# Robot must be lifted onto a stable box. Used for turn calibration —
# adjust turn_90_duration in config/qr_params.yaml until 90° is repeatable.
#
# Usage:
#   tools/qr_stage3_wheels_off.sh                 # default
#   tools/qr_stage3_wheels_off.sh arg:=value      # extra launch args
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=./qr_logging_env.sh
source "$SCRIPT_DIR/qr_logging_env.sh"

_qr_log_command "stage3_wheels_off" -- \
    command ros2 launch qr_nav qr_nav.launch.py "$@"
