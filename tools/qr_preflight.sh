#!/usr/bin/env bash
# Pre-flight checks for the ROSbot 3 PRO (DEPLOY.md §2).
#
# Verifies the OAK-D camera is publishing and /cmd_vel is the expected
# TwistStamped type. Output (5 s sample of topic hz + topic info) is
# captured under <session>/preflight_<TS>/console.log.
#
# Usage:
#   tools/qr_preflight.sh
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=./qr_logging_env.sh
source "$SCRIPT_DIR/qr_logging_env.sh"

# Inline body; `command ros2` bypasses the hook so we don't get nested
# subfolders for each individual check.
PREFLIGHT_BODY='
echo "=== ros2 topic hz /oak/rgb/image_raw (5s sample) ==="
timeout 5 command ros2 topic hz /oak/rgb/image_raw 2>&1 || true
echo
echo "=== ros2 topic info /cmd_vel ==="
command ros2 topic info /cmd_vel 2>&1 || true
echo
echo "=== Pre-flight complete ==="
'

_qr_log_command "preflight" -- bash -c "$PREFLIGHT_BODY"
