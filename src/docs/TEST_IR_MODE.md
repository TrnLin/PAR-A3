# Testing IR Mode (Dark-Room Operation)

Hands-on test procedures for the adaptive-lighting feature: how to confirm the OAK-D Pro's IR floodlight + mono camera path actually works on the bot, end-to-end, before relying on it for the demo.

This guide is the **validation** counterpart to [`DEPLOY.md`](DEPLOY.md) §7 (which is the *bring-up* procedure). DEPLOY.md tells you how to enable the feature; this file tells you how to prove it works and what to capture for the report.

If anything below fails, see *[Troubleshooting](#troubleshooting)* at the bottom before changing code — most issues are param-name drift, not bugs.

---

## 0. Prerequisites

Confirm all of these are true *before* starting any of the tests below.

- [ ] `qr_nav` builds cleanly on the robot (DEPLOY.md §1 done).
- [ ] Stages 1–4 of DEPLOY.md pass — i.e. RGB-only operation works end-to-end.
- [ ] OAK driver is up: `ros2 topic list | grep oak` shows multiple `/oak/...` topics including at least `/oak/rgb/image_raw` and one mono stream (typically `/oak/left/image_raw`).
- [ ] Adaptive lighting is enabled in [`qr_params.yaml`](../config/qr_params.yaml): `adaptive_lighting: true` (the default).
- [ ] A printed QR card (any of the 7) is within reach. **GO** is the safest one to use during validation tests.
- [ ] You have **two SSH sessions** to the robot open.
- [ ] Teleop e-stop is available in a third session:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
  ```
- [ ] The robot is **on a box with wheels off the ground** OR is using `cmd_vel_topic:=/cmd_vel_dummy`. Do not run any of these tests with motors live until Test 4 is clean.

---

## 1. Hardware sanity check (no qr_nav running)

Confirm the IR hardware on the OAK-D Pro is reachable from ROS. This catches every "feature works on my dev machine but the bot is broken" failure mode in 30 seconds.

```bash
# Mono camera publishes
ros2 topic hz /oak/left/image_raw         # expect ~10 Hz, same as RGB
ros2 topic info /oak/left/image_raw -v   # confirm RELIABLE / VOLATILE, encoding mono8

# IR + projector params exist
ros2 param list /oak 2>/dev/null | grep -iE 'flood|laser|ir'
# Expect at least:
#   i_floodlight_brightness        (or floodlight_brightness on older releases)
#   i_laser_dot_projector_current  (or laser_dot_projector_current)
```

**If the topic or param names differ**, update [`qr_params.yaml`](../config/qr_params.yaml) (`mono_image_topic`, `floodlight_param_name`, `dot_projector_param_name`) — no code change. Then `colcon build --packages-select qr_nav --symlink-install && source install/setup.bash` and re-run this section.

### 1.1 Manual IR floodlight toggle (visual check)

The 850 nm IR floodlight is mostly invisible but should produce a faint red glow on the OAK-D's front face. In a dark room (or pointing a phone camera at the OAK — phone cameras see IR), toggle it manually:

```bash
ros2 param set /oak i_laser_dot_projector_current 0
ros2 param set /oak i_floodlight_brightness 1000
# look at the OAK-D front face: faint red glow on a smartphone screen,
# or barely-visible deep red to the naked eye in a dark room.

ros2 param set /oak i_floodlight_brightness 0
# glow disappears.
```

If `ros2 param set` returns `Set parameter failed`, the param name is wrong — re-run the `ros2 param list` step above and match exactly.

---

## 2. Test 1 — Static mono decode under IR

Confirm the *detector code* can decode a QR card on the mono stream with manual IR. This isolates the perception path from the handshake logic.

### 2.1 Setup

Temporarily force the detector to subscribe directly to the mono topic, no handshake involved:

```bash
# In one SSH session, manually pre-arm the IR floodlight (Test 1 doesn't
# involve the adaptive switch, so the detector won't manage IR for us).
ros2 param set /oak i_floodlight_brightness 1000
ros2 param set /oak i_laser_dot_projector_current 0

# Then launch the detector with the rgb topic OVERRIDDEN to the mono topic.
ros2 run qr_nav qr_detector --ros-args \
  --params-file ~/par-a3/install/qr_nav/share/qr_nav/config/qr_params.yaml \
  -p rgb_image_topic:=/oak/left/image_raw \
  -p adaptive_lighting:=false
```

> The `-p` overrides feed the mono topic in directly and *disable* the lighting monitor, so the only thing we're testing is "does cv2.QRCodeDetector decode a QR card from an IR-illuminated mono frame?".

### 2.2 Procedure

In a second session:

```bash
ros2 topic echo /qr_command
```

Turn the room lights **off**. Hold a printed QR card 30–60 cm from the OAK-D, perpendicular to the lens.

- Expected: `/qr_command` fires with the card's command within 1–2 seconds.
- Detector log should be unchanged from RGB operation — same min_bbox_area, same cooldown.

### 2.3 Pass criteria

- [ ] All 7 cards decode on the mono+IR feed (use the same card set as DEPLOY.md §3 Stage 1).
- [ ] No `cv_bridge conversion failed` errors in the detector log.
- [ ] No crashes from `cv2.contourArea` (i.e. the `np.float32` cast is still in place).

If any card fails, see [Troubleshooting #1: "Card won't decode under IR"](#1-card-wont-decode-under-ir).

### 2.4 Clean up

```bash
# Restore IR to off so subsequent tests start clean.
ros2 param set /oak i_floodlight_brightness 0
```

Stop the manual detector launch (Ctrl-C).

---

## 3. Test 2 — Adaptive lighting transitions (stationary, telemetry only)

Confirm the lighting monitor **observes** correctly without any sensor switching getting in the way.

### 3.1 Setup

Full launch, motors diverted (so even if a switch fires accidentally, wheels can't move):

```bash
ros2 launch qr_nav qr_nav.launch.py cmd_vel_topic:=/cmd_vel_dummy
```

Three observer sessions:

```bash
# Session B
ros2 topic echo /qr_detector/lighting_state

# Session C
ros2 topic echo /qr_detector/lighting_metrics

# Session D (optional, watch the handshake even though it shouldn't fire yet)
ros2 topic echo /qr_detector/switch_request
```

### 3.2 Procedure

Starting in a normally-lit room:

1. **Baseline.** `lighting_state` should report `BRIGHT`. `lighting_metrics` `luma_ema` should be in the 80–200 range. Note the value.
2. **Dim halfway.** Lower the lights to roughly half. Wait ≥ `state_dwell_sec` (default 2 s).
   - Expected: `lighting_state` flips to `DIM` once `luma_ema` is below `luma_dim_threshold` (default 60) and has been stable for the dwell window.
   - `switch_request` should fire **once**, with payload `"DIM"` (since DIM still uses the RGB sensor, this is a same-sensor "request" — the detector still emits it, the interpreter is already in IDLE so it acks immediately with `/nav_state`).
3. **Off completely.** Cover the lens with your hand for a tighter test, or kill the room lights entirely.
   - Expected: `lighting_state` flips `DIM → DARK` after another dwell window.
   - Detector log shows `Performing switch: subscribing to /oak/left/image_raw as MONO`.
   - `IR applied: floodlight=1000 mA, projector=0 mA` confirms the IR set succeeded.
4. **Recover.** Lights back on, gradually.
   - Expected: `DARK → DIM → BRIGHT` as you cross the upper hysteresis thresholds (`luma_dark_threshold + luma_bright_hysteresis = 45`, `luma_dim_threshold + luma_bright_hysteresis = 80`).
   - Floodlight drops to 300 mA in DIM, 0 mA in BRIGHT (logged each time).

### 3.3 Pass criteria

- [ ] State transitions happen in order BRIGHT ↔ DIM ↔ DARK; never skips a state.
- [ ] No oscillation at the boundary (transitions are stable for at least `state_dwell_sec`).
- [ ] At least one `IR applied` log line per direction, with the expected floodlight current.
- [ ] In `lighting_metrics`, `sensor_mode` is `RGB` for BRIGHT/DIM and `MONO` for DARK.
- [ ] `dot_projector_current` remains 0 throughout (look at the `IR applied` log lines).

If transitions flicker, see [Troubleshooting #2: "State flickers between two values"](#2-state-flickers-between-two-values).

---

## 4. Test 3 — End-to-end adaptive switch with motion

This is the real-world test. Wheels off the ground or on `cmd_vel_dummy` — your choice based on confidence in Tests 1 and 2.

### 4.1 Setup

```bash
ros2 launch qr_nav qr_nav.launch.py cmd_vel_topic:=/cmd_vel_dummy
```

Sessions to watch (split across panes if possible):

```bash
ros2 topic echo /nav_state
ros2 topic echo /qr_detector/lighting_state
ros2 topic echo /qr_detector/switch_request
ros2 topic echo /qr_detector/switch_complete
ros2 topic echo /cmd_vel_dummy
```

### 4.2 Procedure — happy path

1. **Arm the robot.** Lights on, show `GO`. Verify `nav_state` flips `IDLE → DRIVING`, `/cmd_vel_dummy` shows `linear.x = 0.08`.
2. **Kill the lights.** Watch the cascade:
   - `lighting_state: BRIGHT → DIM` (no sensor switch — same RGB topic).
   - `lighting_state: DIM → DARK`.
   - `switch_request: "DARK"` is published.
   - `nav_state: DRIVING → STOPPED` immediately (DRIVING is treated as "no in-flight action to finish").
   - Detector log: `Performing switch: subscribing to /oak/left/image_raw as MONO`.
   - `IR applied: floodlight=1000 mA, projector=0 mA`.
3. **Show any valid card to the (now mono+IR) camera.** Within 1–2 seconds:
   - `switch_complete: "ok"` is published.
   - `nav_state: STOPPED → DRIVING`.
   - `/cmd_vel_dummy` resumes `linear.x = 0.08`.
4. **Lights back on.** State recovers `DARK → DIM → BRIGHT`. Switch fires again the other way (`switch_request: "BRIGHT"`), robot stops, swaps back to RGB, validates, resumes.

### 4.3 Procedure — mid-turn switch (the critical edge case)

This validates the `pending_switch` deferred-handshake logic.

1. Lights on. Show `GO` → `DRIVING`.
2. Show `TURN_LEFT`. `nav_state` flips to `TURNING`. `/cmd_vel_dummy` now shows `angular.z = 0.4`.
3. **Immediately** (within the first second of the turn), kill the lights.
4. Watch closely:
   - `lighting_state` flips to `DARK` after dwell.
   - `switch_request: "DARK"` is published.
   - **`nav_state` MUST stay `TURNING`** until the turn completes its full `turn_90_duration` (default 4 s).
   - Detector log: `Switch requested (DARK); deferring until current turn completes`.
   - At turn completion, log: `Turn complete — honouring pending switch: transitioning to STOPPED`. `nav_state: TURNING → STOPPED` (not DRIVING).
   - Sensor switch + IR set as in §4.2 step 2.
5. Show a card to validate, robot resumes.

If `nav_state` ever flips from `TURNING` to anything other than `STOPPED` during step 4, the `pending_switch` logic regressed — file it as a bug.

### 4.4 Procedure — validation timeout

This validates the fail-safe.

1. Lights on, show `GO`, then kill the lights as in §4.2 steps 1–2 (so the robot is now `STOPPED` with mono+IR active).
2. **Do not show any QR card.** Wait for `validation_timeout_sec` (default 10 s).
3. Expected:
   - Detector log: `Switch validation timed out on MONO after 10.0s — robot stays STOPPED until operator sends GO`.
   - `switch_complete: "timeout"`.
   - `nav_state` stays `STOPPED`.
4. Now show a `GO` card. Robot resumes manually.

### 4.5 Pass criteria

- [ ] §4.2 happy path completes without operator intervention beyond showing one card.
- [ ] §4.3 mid-turn test: `nav_state` never leaves `TURNING` early.
- [ ] §4.4 timeout test: robot does not auto-resume after 10 s without a card.
- [ ] All `switch_request` / `switch_complete` messages are accounted for (no orphaned requests).
- [ ] `/cmd_vel_dummy` only carries non-zero velocity when `nav_state ∈ {DRIVING, TURNING, RECOVERING}`.

---

## 5. Test 4 — Floor run (motors live)

**Only run this if Tests 1–3 are 100% clean.** Same procedure as DEPLOY.md §6 Stage 4, but with a real dark-room transition partway through.

```bash
ros2 launch qr_nav qr_nav.launch.py
```

1. Lights on. `GO` to arm. Drive a short straight.
2. While the robot is moving, kill the lights.
3. Expected behaviour:
   - Robot stops within ~`state_dwell_sec` after the lighting state confirms `DARK`.
   - You hear the depthai driver's IR LED kick on (silent — visual only).
   - Show a card to validate; robot resumes.
4. Hand on teleop e-stop the whole time.

### 5.1 Safety notes

- The robot **will stop on its own** when the lighting transition fires. That's by design, not a fault. Don't intervene unless it stays stopped past the validation timeout.
- If validation times out and you don't show a card, robot is stuck in `STOPPED` indefinitely. Show a `GO` card or hit e-stop and shut down cleanly.
- Open-loop turns drift more on a darker, colder battery. Recheck `turn_90_duration` calibration if the dark-room test is far from your initial calibration session.

---

## 6. What to capture for the report

Each run of Test 3 or Test 4 produces:

- A CSV at `/tmp/qr_nav_logs/qr_log_<timestamp>.csv` with the new `lighting_state` column. Pull off with `scp` before powering down (see DEPLOY.md §8).
- Detector log lines tagged `Lighting state:`, `Switch requested`, `Performing switch`, `IR applied`, `Switch validated`. Save the terminal output of the detector session.

Metrics you should report:

| Metric | How to compute |
|--------|----------------|
| Lighting transition latency | Time between you flipping the lights and `lighting_state` flipping. Read from CSV timestamps. |
| Switch-to-validation latency | Time between `switch_request` published and `switch_complete: ok` published. Detector log. |
| Validation timeout rate | Over N test runs, count `switch_complete: timeout` vs `ok`. |
| Decode accuracy under IR | Same procedure as DEPLOY.md detection accuracy, but cards held in IR-only illumination. |
| State-distribution per run | The "Lighting-state distribution" lines in the detector's shutdown summary. |

---

## Troubleshooting

### 1. Card won't decode under IR

| Likely cause | Check |
|---|---|
| Floodlight not actually on | `ros2 param get /oak i_floodlight_brightness` — should be 1000 in DARK. |
| Floodlight saturating up-close cards | Move card to 60+ cm or reduce `ir_floodlight_dark_mA` to 700. |
| Glossy/laminated card causing IR specular hot-spot | Print matte; rotate card off-perpendicular by ~10° to test. |
| Dot projector accidentally on (speckle dots visible in frame) | `ros2 param get /oak i_laser_dot_projector_current` — must be 0. The detector force-sets this on every transition, so if it's non-zero something else is fighting us. |
| Bbox below threshold for mono resolution | Mono cameras are 1280×800 vs RGB 1920×1080. Lower `min_bbox_area` in YAML (try 300). |

### 2. State flickers between two values

`lighting_state` toggling rapidly at the threshold means dwell + hysteresis aren't enough for your room.

- Raise `state_dwell_sec` from 2.0 to 3.0.
- Raise `luma_bright_hysteresis` from 20 to 30.
- Lower `luma_ema_alpha` from 0.1 to 0.05 (smoother but slower).

### 3. `IR applied` log never fires

| Likely cause | Check |
|---|---|
| `/oak` node not up | `ros2 node list \| grep oak` — must show the camera node. |
| Param names wrong | `ros2 param list /oak \| grep -iE 'flood\|laser'` — match exactly against `floodlight_param_name` / `dot_projector_param_name` in YAML. |
| rclpy too old | Detector log will say `AsyncParameterClient is unavailable; IR control disabled`. Manual `ros2 param set` as workaround. |
| Network not granting param service access | Try `ros2 param set /oak i_floodlight_brightness 0` manually — if that fails, it's not a qr_nav bug. |

### 4. `switch_request` fires but `nav_state` never reaches STOPPED

Interpreter not receiving the message.

```bash
ros2 topic info /qr_detector/switch_request
# "Subscription count: 1"  <-- the interpreter
```

If 0, the `command_interpreter_node` isn't running or didn't pick up the new code. Rebuild with `--symlink-install` and verify the banner mentions the switch handshake.

### 5. Robot auto-resumes when it shouldn't (validation skips)

Detector should publish `switch_complete: ok` only on a valid QR decode on the **new** sensor. If you see `ok` without showing a card:

- Check `lighting_metrics`: `decode_window_filled` resets to 0 on a switch. If it's already non-zero immediately after a switch, the resubscription didn't happen — the detector is still reading the old sensor. Inspect the detector log for `Performing switch: subscribing to ...`.

### 6. Need to disable adaptive lighting for the demo

Kill switch in [`qr_params.yaml`](../config/qr_params.yaml):

```yaml
qr_detector:
  ros__parameters:
    adaptive_lighting: false
```

Rebuild. The detector reverts to RGB-only operation: no lighting monitor, no IR control, no handshake. Tests 1–4 will not run in this mode (and their failure is not a bug).

---

## Quick reference

| Goal | Command |
|------|---------|
| Mono camera publishing? | `ros2 topic hz /oak/left/image_raw` |
| IR params reachable? | `ros2 param list /oak \| grep -iE 'flood\|laser\|ir'` |
| Manually enable IR floodlight | `ros2 param set /oak i_floodlight_brightness 1000` |
| Manually disable IR floodlight | `ros2 param set /oak i_floodlight_brightness 0` |
| Force dot projector off | `ros2 param set /oak i_laser_dot_projector_current 0` |
| Watch lighting state | `ros2 topic echo /qr_detector/lighting_state` |
| Watch lighting metrics (luma + decode rate) | `ros2 topic echo /qr_detector/lighting_metrics` |
| Watch switch handshake | `ros2 topic echo /qr_detector/switch_request /qr_detector/switch_complete` |
| Force-test detector on mono only | `ros2 run qr_nav qr_detector --ros-args -p rgb_image_topic:=/oak/left/image_raw -p adaptive_lighting:=false` |
| Disable adaptive lighting | Edit [`qr_params.yaml`](../config/qr_params.yaml) → `adaptive_lighting: false`, rebuild |
