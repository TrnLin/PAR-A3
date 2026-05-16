#!/usr/bin/env bash
# DEPLOY.md §3 — Stage 1: detector-only run (robot stationary).
#
# Runs just qr_detector with the tuned params YAML so the FSM never publishes
# velocity commands. Use to walk all 7 cards past the camera and confirm each
# fires the expected /qr_command string.
#
# Usage:
#   tools/qr_stage1_detect.sh
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=./qr_logging_env.sh
source "$SCRIPT_DIR/qr_logging_env.sh"

# Locate the installed params YAML via ros2's own package index.
PKG_PREFIX="$(command ros2 pkg prefix qr_nav 2>/dev/null || true)"
if [ -z "$PKG_PREFIX" ]; then
    echo "qr_stage1_detect.sh: ros2 pkg prefix qr_nav failed -- did you" \
         "'source install/setup.bash'?" >&2
    exit 1
fi
PARAMS_FILE="$PKG_PREFIX/share/qr_nav/config/qr_params.yaml"

_qr_log_command "stage1_detect" -- \
    command ros2 run qr_nav qr_detector \
    --ros-args --params-file "$PARAMS_FILE" "$@"
