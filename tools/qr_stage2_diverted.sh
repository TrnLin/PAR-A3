#!/usr/bin/env bash
# DEPLOY.md §4 — Stage 2: full launch with motors diverted.
#
# /cmd_vel is remapped to /cmd_vel_dummy so the base controller never moves.
# Safe to run with the robot on the floor.
#
# Usage:
#   tools/qr_stage2_diverted.sh                      # default remap
#   tools/qr_stage2_diverted.sh extra_launch_arg:=v  # extra launch args
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=./qr_logging_env.sh
source "$SCRIPT_DIR/qr_logging_env.sh"

_qr_log_command "stage2_diverted" -- \
    command ros2 launch qr_nav qr_nav.launch.py \
    cmd_vel_topic:=/cmd_vel_dummy "$@"
