# qr_nav run logs

This folder holds **every recorded run** of the QR navigation pipeline. One folder per operator shift; one subfolder per command inside that shift. Folders are committed to git so every demo, calibration, and pre-flight is preserved alongside the code that produced it.

For the operator deployment workflow that produces these logs, see [`../src/docs/DEPLOY.md`](../src/docs/DEPLOY.md).

---

## Layout

```
logs/
  README.md
  .gitkeep
  session_<TS>/                          # one per operator shift
    session_meta.txt                     # operator, hostname, git rev, ROS_DISTRO, start TS
    session_summary.txt                  # appended by qr_session_close.sh
    preflight_<TS>/
      console.log                        # full stdout/stderr
      cmd_meta.txt                       # argv, start/end TS, exit code
    stage1_detect_<TS>/
      console.log
      cmd_meta.txt
    stage2_diverted_<TS>/
      console.log
      qr_log.csv                         # CSV from data_logger_node
      cmd_meta.txt
    topic_echo_nav_state_<TS>/
      console.log
      cmd_meta.txt
    stage4_floor_<TS>/
      console.log
      qr_log.csv
      cmd_meta.txt
```

CSV and `console.log` from a launch are always in the same folder: the auto-logging hook exports `QR_NAV_RUN_DIR` before starting the launch, and `data_logger_node` writes its CSV into that directory.

---

## Enable auto-logging (one-time setup)

On both the robot and the laptop, add one line to your shell init file:

```bash
echo 'source "$HOME/par-a3/repo/tools/qr_logging_env.sh"' >> ~/.bashrc   # Linux/robot
echo 'source "$HOME/Documents/Coding/PAR-A3/repo/tools/qr_logging_env.sh"' >> ~/.zshrc  # macOS laptop
exec $SHELL -l
```

Adjust the path to match your checkout. After this, every relevant `ros2` command typed in any new shell is auto-recorded under `logs/session_<TS>/`. Non-matching `ros2` calls and other commands (`colcon`, `python3`, …) pass through unchanged.

What gets auto-logged:

- `ros2 launch qr_nav qr_nav.launch.py …`
- `ros2 run qr_nav <qr_detector|command_interpreter|qr_logger>`
- `ros2 topic echo /qr_command|/qr_detections|/nav_state|/cmd_vel*`
- `ros2 topic hz /oak/*`
- `ros2 topic info /cmd_vel`

`teleop_twist_keyboard` is intentionally passed through unmodified (`tee` corrupts its readline UI), but a one-line note is written to `session_summary.txt` so you know it was used.

---

## Record a shift

Just run the commands from [`DEPLOY.md`](../src/docs/DEPLOY.md) §2–§6 as usual. They'll be auto-grouped under one `session_<TS>/` folder.

For cleaner folder names (`stage4_floor_<TS>` instead of `launch_qr_nav_<TS>`), use the per-stage shortcut scripts:

| Stage | Shortcut |
|-------|----------|
| §2 Pre-flight | `tools/qr_preflight.sh` |
| §3 Stage 1 (detector only) | `tools/qr_stage1_detect.sh` |
| §4 Stage 2 (motors diverted) | `tools/qr_stage2_diverted.sh` |
| §5 Stage 3 (wheels off) | `tools/qr_stage3_wheels_off.sh` |
| §6 Stage 4 (floor run) | `tools/qr_stage4_floor.sh` |

At the end of the shift:

```bash
tools/qr_session_close.sh
```

This appends end timestamp + per-subfolder exit codes to `session_summary.txt` and clears the active-session sentinel (`~/.qr_nav_current_session`). Skipping it is harmless — the next session auto-starts cleanly; you just lose the closing summary.

---

## Harvest from the robot

Logs land at `~/par-a3/repo/logs/session_*/` on the robot. To pull them to the laptop's repo:

```bash
# from the laptop, in the repo root
scp -r 'husarion@192.168.1.150:~/par-a3/repo/logs/session_*' ./logs/
git add logs/session_*
git commit -m "logs: <date> <description>"
```

---

## Notes

- **Runs are intentionally tracked in git.** If a shift is scrap (bad calibration, false start), delete the `session_<TS>/` folder before committing.
- **`*.log` files**: there's an unrelated `*.log` rule in `repo/.gitignore` (under "Django stuff"). It's countered by an explicit `!logs/**/*.log` un-ignore so our `console.log` files stay tracked.
- **Manual session start**: `tools/qr_session.sh` creates a fresh session immediately. Useful if you want a deliberate boundary at the top of a shift without running any wrapped command first.
- **Ad-hoc `ros2 run qr_logger` outside the wrapper** still works: with `log_directory: ""` (the new default in `qr_params.yaml`), it falls back to `<repo>/logs/qr_log_<TS>.csv`. Set `log_directory` to a non-empty path in the YAML to override (e.g. `/tmp/qr_nav_logs` for legacy behaviour).
