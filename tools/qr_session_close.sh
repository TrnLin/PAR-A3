#!/usr/bin/env bash
# Close the active qr_nav logging session.
#
# Appends end timestamp + per-subfolder exit codes to session_summary.txt
# and clears the ~/.qr_nav_current_session sentinel. Idempotent.
#
# Usage:
#   tools/qr_session_close.sh
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=./qr_logging_env.sh
source "$SCRIPT_DIR/qr_logging_env.sh"

qr_session_close
