#!/usr/bin/env bash
# One-command install + build + run wrapper for the qr_nav ROS 2 package.
#
# Handles the steps a fresh checkout normally requires before launching:
#   1. Install Python tooling deps (`pip install -r requirements.txt`) inside
#      a local virtualenv at <repo>/venv. Used for QR card generation and
#      lint; the ROS nodes themselves use system packages.
#   2. Build the colcon workspace from the repo root with --symlink-install.
#   3. Source the resulting overlay in the current shell.
#   4. Dispatch to one of the existing tools/qr_stage*.sh scripts so logging
#      and topic remaps stay consistent with the staged DEPLOY workflow.
#
# The wrapper deliberately requires an explicit --mode flag instead of
# launching live floor motion by default; building successfully should never
# automatically command the wheels.
#
# Usage:
#   tools/qr_install_and_run.sh --mode stage2              # safe diverted run
#   tools/qr_install_and_run.sh --mode stage4              # live floor run
#   tools/qr_install_and_run.sh --mode preflight           # camera + /cmd_vel checks
#   tools/qr_install_and_run.sh --mode stage1              # detector-only
#   tools/qr_install_and_run.sh --mode stage3              # wheels off the ground
#   tools/qr_install_and_run.sh --mode stage2 --skip-build --no-venv
#   tools/qr_install_and_run.sh --mode stage2 -- session_id:=trial_a
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
    cat <<'EOF'
Usage: tools/qr_install_and_run.sh --mode MODE [options] [-- <extra launch args>]

Run modes (delegated to existing tools/qr_*.sh scripts):
  preflight   Camera + /cmd_vel checks (no robot motion).
  stage1      Detector-only run (robot stationary).
  stage2      Full launch with /cmd_vel diverted to /cmd_vel_dummy.
  stage3      Full launch, robot lifted with wheels off the ground.
  stage4      Live floor run (real motion published to /cmd_vel).

Options:
  --mode MODE    Required. Selects which stage script to run after build.
  --skip-build   Skip "colcon build --packages-select qr_nav --symlink-install".
  --no-venv      Skip Python venv creation + pip install of requirements.txt.
  -h, --help     Show this help and exit.

Anything after a literal "--" is forwarded as extra arguments to the
underlying stage script. Useful for passing launch arguments to qr_nav,
for example:
  tools/qr_install_and_run.sh --mode stage2 -- session_id:=trial_a

Examples:
  tools/qr_install_and_run.sh --mode stage2
  tools/qr_install_and_run.sh --mode stage4 -- session_id:=demo_run
  tools/qr_install_and_run.sh --mode stage2 --skip-build --no-venv
EOF
}

MODE=""
SKIP_BUILD=0
NO_VENV=0
EXTRA_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --mode)
            if [ $# -lt 2 ] || [ -z "${2:-}" ]; then
                echo "qr_install_and_run.sh: --mode requires an argument" >&2
                exit 2
            fi
            MODE="$2"
            shift 2
            ;;
        --mode=*)
            MODE="${1#--mode=}"
            shift
            ;;
        --skip-build)
            SKIP_BUILD=1
            shift
            ;;
        --no-venv)
            NO_VENV=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            while [ $# -gt 0 ]; do
                EXTRA_ARGS+=("$1")
                shift
            done
            ;;
        *)
            echo "qr_install_and_run.sh: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ -z "$MODE" ]; then
    echo "qr_install_and_run.sh: --mode is required" >&2
    usage >&2
    exit 2
fi

case "$MODE" in
    preflight) STAGE_SCRIPT="$SCRIPT_DIR/qr_preflight.sh" ;;
    stage1)    STAGE_SCRIPT="$SCRIPT_DIR/qr_stage1_detect.sh" ;;
    stage2)    STAGE_SCRIPT="$SCRIPT_DIR/qr_stage2_diverted.sh" ;;
    stage3)    STAGE_SCRIPT="$SCRIPT_DIR/qr_stage3_wheels_off.sh" ;;
    stage4)    STAGE_SCRIPT="$SCRIPT_DIR/qr_stage4_floor.sh" ;;
    *)
        echo "qr_install_and_run.sh: unknown mode: $MODE" >&2
        usage >&2
        exit 2
        ;;
esac

if [ ! -x "$STAGE_SCRIPT" ]; then
    echo "qr_install_and_run.sh: stage script not executable: $STAGE_SCRIPT" >&2
    exit 1
fi

# --- Prerequisite checks ----------------------------------------------------

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "qr_install_and_run.sh: required command not found: $1" >&2
        echo "    Source your ROS 2 underlay (e.g. 'source /opt/ros/jazzy/setup.bash')" >&2
        echo "    and install python3 + python3-colcon-common-extensions before re-running." >&2
        exit 1
    fi
}

require_cmd python3
require_cmd ros2
require_cmd colcon

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "qr_install_and_run.sh: ROS_DISTRO is not set -- source your ROS 2" \
         "underlay (e.g. 'source /opt/ros/jazzy/setup.bash') before re-running." >&2
    exit 1
fi
if [ "${ROS_DISTRO}" != "jazzy" ]; then
    echo "qr_install_and_run.sh: warning: ROS_DISTRO=${ROS_DISTRO}," \
         "this project is tuned for 'jazzy'." >&2
fi

# --- Python venv + requirements --------------------------------------------

if [ "$NO_VENV" -eq 0 ]; then
    REQ_FILE="$REPO_ROOT/requirements.txt"
    if [ ! -f "$REQ_FILE" ]; then
        echo "qr_install_and_run.sh: requirements.txt not found at $REQ_FILE" >&2
        exit 1
    fi
    VENV_DIR="$REPO_ROOT/venv"
    if [ ! -d "$VENV_DIR" ]; then
        echo "qr_install_and_run.sh: creating venv at $VENV_DIR"
        python3 -m venv "$VENV_DIR"
    fi
    # shellcheck disable=SC1091
    . "$VENV_DIR/bin/activate"
    python -m pip install --upgrade pip
    python -m pip install -r "$REQ_FILE"
    deactivate
fi

# --- colcon build -----------------------------------------------------------

if [ "$SKIP_BUILD" -eq 0 ]; then
    echo "qr_install_and_run.sh: building qr_nav from $REPO_ROOT"
    (cd "$REPO_ROOT" && colcon build --packages-select qr_nav --symlink-install)
fi

# --- Source workspace overlay ----------------------------------------------

SETUP_BASH="$REPO_ROOT/install/setup.bash"
if [ ! -f "$SETUP_BASH" ]; then
    echo "qr_install_and_run.sh: missing $SETUP_BASH -- run without" \
         "--skip-build, or build the workspace manually." >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$SETUP_BASH"

# --- Dispatch to stage script ----------------------------------------------

echo "qr_install_and_run.sh: dispatching to $(basename "$STAGE_SCRIPT")"
if [ "${#EXTRA_ARGS[@]}" -gt 0 ]; then
    exec "$STAGE_SCRIPT" "${EXTRA_ARGS[@]}"
else
    exec "$STAGE_SCRIPT"
fi
