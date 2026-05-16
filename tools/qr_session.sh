#!/usr/bin/env bash
# Start a new qr_nav logging session.
#
# Optional — wrapped `ros2` commands auto-start a session if none is active.
# Use this when you want a deliberate session boundary at the top of a shift.
#
# Usage:
#   tools/qr_session.sh
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=./qr_logging_env.sh
source "$SCRIPT_DIR/qr_logging_env.sh"

_qr_start_session >/dev/null
