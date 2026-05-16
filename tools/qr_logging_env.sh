#!/usr/bin/env bash
# qr_nav auto-logging hook.
#
# Source from ~/.bashrc (Linux/robot) or ~/.zshrc (macOS) once:
#   echo 'source "$HOME/par-a3/repo/tools/qr_logging_env.sh"' >> ~/.bashrc
# (Replace the path with the absolute path to your checkout.)
#
# After sourcing, every qr_nav-relevant `ros2` command typed in this shell is
# auto-recorded under <repo>/logs/session_<TS>/<slug>_<TS>/, holding
# console.log + cmd_meta.txt (+ qr_log.csv for FSM launches). Non-matching
# `ros2` invocations and everything else (colcon, python3, ...) pass through
# unchanged.
#
# Public commands:
#   qr_session_close   # finalize the active session's summary, clear sentinel

# --- Resolve script location (works in both bash and zsh when sourced) ---
_QR_LOGGING_ENV_FILE="${BASH_SOURCE[0]:-$0}"
_QR_LOGGING_TOOLS_DIR="$(cd "$(dirname "$_QR_LOGGING_ENV_FILE")" && pwd)"
_QR_LOGGING_REPO_ROOT="$(cd "$_QR_LOGGING_TOOLS_DIR/.." && pwd)"

_QR_SENTINEL="$HOME/.qr_nav_current_session"

# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------

_qr_ts() {
    date +%Y%m%d_%H%M%S
}

_qr_pipe_exit_code() {
    # Real exit code of the first command in a pipeline (not tee's).
    if [ -n "${ZSH_VERSION:-}" ]; then
        echo "${pipestatus[1]}"
    else
        echo "${PIPESTATUS[0]}"
    fi
}

_qr_topic_to_slug() {
    # /qr_command -> qr_command, /oak/rgb/image_raw -> oak_rgb_image_raw
    printf '%s' "$1" | sed 's|^/||;s|/|_|g'
}

_qr_start_session() {
    local ts session_dir meta
    ts="$(_qr_ts)"
    session_dir="$_QR_LOGGING_REPO_ROOT/logs/session_$ts"
    mkdir -p "$session_dir"

    meta="$session_dir/session_meta.txt"
    {
        echo "session_start_ts: $ts"
        echo "operator:         ${USER:-unknown}"
        echo "hostname:         $(hostname 2>/dev/null || echo unknown)"
        echo "ros_distro:       ${ROS_DISTRO:-unknown}"
        echo "repo_root:        $_QR_LOGGING_REPO_ROOT"
        if command -v git >/dev/null 2>&1; then
            local rev branch
            rev="$(git -C "$_QR_LOGGING_REPO_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
            branch="$(git -C "$_QR_LOGGING_REPO_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
            echo "git_rev:          $rev"
            echo "git_branch:       $branch"
        else
            echo "git_rev:          (git not installed)"
        fi
    } > "$meta"

    printf '%s\n' "$session_dir" > "$_QR_SENTINEL"
    echo "[qr_logging] started session: $session_dir" >&2
    printf '%s\n' "$session_dir"
}

_qr_active_session() {
    # Returns the active session folder, auto-starting one if none exists.
    if [ -f "$_QR_SENTINEL" ]; then
        local sess
        sess="$(cat "$_QR_SENTINEL")"
        if [ -d "$sess" ]; then
            printf '%s\n' "$sess"
            return 0
        fi
    fi
    _qr_start_session
}

_qr_log_command() {
    # Usage: _qr_log_command <slug> [--] <cmd> [args...]
    # Records cmd output under <session>/<slug>_<TS>/console.log.
    # Exports QR_NAV_RUN_DIR so data_logger_node lands its CSV in the same folder.
    local slug="$1"
    shift
    [ "${1:-}" = "--" ] && shift

    local session ts subdir log_file meta_file rc
    session="$(_qr_active_session)"
    ts="$(_qr_ts)"
    subdir="$session/${slug}_${ts}"
    mkdir -p "$subdir"
    log_file="$subdir/console.log"
    meta_file="$subdir/cmd_meta.txt"

    {
        echo "slug:       $slug"
        echo "start_ts:   $ts"
        echo "cwd:        $PWD"
        echo "argv:       $*"
    } > "$meta_file"

    export QR_NAV_RUN_DIR="$subdir"
    echo "[qr_logging] -> $subdir" >&2

    "$@" 2>&1 | tee "$log_file"
    rc="$(_qr_pipe_exit_code)"

    {
        echo "end_ts:     $(_qr_ts)"
        echo "exit_code:  $rc"
    } >> "$meta_file"

    unset QR_NAV_RUN_DIR
    return "$rc"
}

_qr_log_teleop_note() {
    # teleop_twist_keyboard is interactive (readline); tee'ing it corrupts the
    # UI. Pass through unmodified but drop a note in session_summary.txt so we
    # know it ran.
    local session
    session="$(_qr_active_session)"
    echo "$(_qr_ts) teleop_twist_keyboard used (passthrough, no log)" \
        >> "$session/session_summary.txt"
}

# ---------------------------------------------------------------------------
# Public
# ---------------------------------------------------------------------------

qr_session_close() {
    if [ ! -f "$_QR_SENTINEL" ]; then
        echo "[qr_logging] no active session" >&2
        return 0
    fi
    local session
    session="$(cat "$_QR_SENTINEL")"
    if [ ! -d "$session" ]; then
        rm -f "$_QR_SENTINEL"
        echo "[qr_logging] sentinel pointed at missing dir; cleared" >&2
        return 0
    fi

    local summary="$session/session_summary.txt"
    {
        echo "=============================================="
        echo "session_end_ts: $(_qr_ts)"
        echo "subfolders:"
        local sub rc
        for sub in "$session"/*/; do
            [ -d "$sub" ] || continue
            rc="(running)"
            if [ -f "$sub/cmd_meta.txt" ]; then
                rc="$(grep '^exit_code:' "$sub/cmd_meta.txt" 2>/dev/null \
                      | awk '{print $2}')"
                [ -z "$rc" ] && rc="(running)"
            fi
            echo "  - $(basename "$sub")  exit=$rc"
        done
    } >> "$summary"

    rm -f "$_QR_SENTINEL"
    echo "[qr_logging] closed session: $session" >&2
}

# ---------------------------------------------------------------------------
# ros2 hook — transparent override
# ---------------------------------------------------------------------------

ros2() {
    local sub="${1:-}"

    case "$sub" in
        launch)
            if [ "${2:-}" = "qr_nav" ]; then
                _qr_log_command "launch_qr_nav" -- command ros2 "$@"
                return $?
            fi
            ;;
        run)
            local pkg="${2:-}"
            local exe="${3:-}"
            if [ "$pkg" = "qr_nav" ]; then
                _qr_log_command "run_${exe}" -- command ros2 "$@"
                return $?
            elif [ "$pkg" = "teleop_twist_keyboard" ]; then
                _qr_log_teleop_note
                command ros2 "$@"
                return $?
            fi
            ;;
        topic)
            local action="${2:-}"
            local topic="${3:-}"
            case "$action" in
                echo)
                    case "$topic" in
                        /qr_command|/qr_detections|/nav_state|/cmd_vel*)
                            _qr_log_command \
                                "topic_echo_$(_qr_topic_to_slug "$topic")" \
                                -- command ros2 "$@"
                            return $?
                            ;;
                    esac
                    ;;
                hz)
                    case "$topic" in
                        /oak/*)
                            _qr_log_command \
                                "topic_hz_$(_qr_topic_to_slug "$topic")" \
                                -- command ros2 "$@"
                            return $?
                            ;;
                    esac
                    ;;
                info)
                    if [ "$topic" = "/cmd_vel" ]; then
                        _qr_log_command "topic_info_cmd_vel" \
                            -- command ros2 "$@"
                        return $?
                    fi
                    ;;
            esac
            ;;
    esac

    command ros2 "$@"
}
