# qr_nav — End-to-End Deployment Guide (ROSbot 3 PRO)

Operator guide for taking the `qr_nav` package from a fresh laptop checkout to a calibrated, logging floor run on the Husarion ROSbot 3 PRO. Companion to [`README.md`](README.md) (architecture & API reference).

This guide is the consolidation of every gotcha hit during initial bring-up. Read top to bottom and don't escalate stages until the previous one is clean.

---

## 0. State going in (already done in the repo)

- **Cards generated** under [`../../../qr_cards/`](../../../qr_cards/) — print [`all_commands.pdf`](../../../qr_cards/all_commands.pdf) at **100% scale** (no "fit to page"). 7 A4 pages, one command per page. Cards are extracted from `VALID_COMMANDS` via AST in [`tools/generate_qr_cards.py`](../../../tools/generate_qr_cards.py) so they can never silently drift from the strings the detector accepts.
- **Code matches the actual robot:**
  - Camera topic: `/oak/rgb/image_raw` (Husarion's OAK-D namespace, **not** the depthai-ros default `/camera/camera/...`).
  - Camera QoS: `RELIABLE` on the subscriber (matches Husarion's publisher).
  - `/cmd_vel` message type: `geometry_msgs/msg/TwistStamped` (ROS 2 Jazzy + Husarion convention, **not** plain `Twist`).
  - `cv2.contourArea` cast to `np.float32` (OpenCV 4.6 on Jazzy rejects `float64`).
- **Config tuned for safe first run** in [`config/qr_params.yaml`](config/qr_params.yaml):
  - `cruise_speed: 0.08 m/s` (demo: 0.2)
  - `turn_speed: 0.4 rad/s` (demo: 0.8)
  - `turn_90_duration: 4.0 s`, `turn_180_duration: 8.0 s` (re-derived for new turn_speed)
  - `recovery_timeout: 3.0 s` (demo: 5.0)

## Robot facts to keep in mind

| Item                       | Value                                                         |
| -------------------------- | ------------------------------------------------------------- |
| SSH                        | `husarion@192.168.1.150`                                      |
| OS / kernel                | Ubuntu 24.04 / Pi 5                                           |
| ROS distro                 | **Jazzy**                                                     |
| Workspace on robot         | `~/par-a3/`                                                   |
| Camera image rate observed | ~10 Hz (CPU-bound, not 30 — fine at 0.08 m/s)                 |
| `/cmd_vel` traffic         | Web UI (`foxglove_bridge`) and joystick (`joy2twist`), no mux |

---

## 1. Ship & build

From your laptop:

```bash
cd ~/Documents/Coding/PAR-A3
scp -r repo/src/qr_nav husarion@192.168.1.150:~/par-a3/repo/src/
```

On the robot (one-time setup the first time, then just rebuild):

```bash
ssh husarion@192.168.1.150
sudo apt install -y ros-jazzy-cv-bridge python3-opencv          # one-time
echo 'source ~/par-a3/install/setup.bash' >> ~/.bashrc           # one-time
cd ~/par-a3
colcon build --packages-select qr_nav --symlink-install
source install/setup.bash
```

`--symlink-install` matters: future `.py` edits don't need a rebuild — just `scp` and re-launch. (YAML edits still need a quick `colcon build`, but it's instant.)

---

## 2. Pre-flight (3 minutes, no robot motion)

Two SSH sessions on the robot, both read-only.

**Session A — camera is alive:**

```bash
ros2 topic hz /oak/rgb/image_raw
```

Should settle around 8–11 Hz. If it says "does not appear to be published yet", the OAK driver isn't up — bring it up before going further.

**Session B — `/cmd_vel` plumbing intact:**

```bash
ros2 topic info /cmd_vel
```

Must show `Type: geometry_msgs/msg/TwistStamped`. If it ever changes (firmware update etc.), our nodes need updating.

**Optional — confirm teleop works** so you have a manual e-stop ready for later:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

The `--ros-args -p stamped:=true` flag is critical — without it teleop publishes plain `Twist` to a `TwistStamped` topic and the base controller drops every message. Drive briefly, **Ctrl-C** before continuing.

---

## 3. Stage 1 — Detection only (robot stationary)

Single SSH session:

```bash
ros2 run qr_nav qr_detector \
  --ros-args --params-file ~/par-a3/install/qr_nav/share/qr_nav/config/qr_params.yaml
```

The startup banner **must** include:

```text
QR Detector Node started — topic: /oak/rgb/image_raw, rate: 15.0 Hz, min_bbox_area: 500, qos: RELIABLE
```

If `qos:` says anything else, the YAML didn't load — re-source `install/setup.bash`.

Second session:

```bash
ros2 topic echo /qr_command
```

Walk **each of the 7 printed cards** past the OAK-D camera, ~30–60 cm away, one at a time. Confirm the matching string fires for **all 7**. Don't move on until 7/7 work.

If the detector crashes with `(-215:Assertion failed) depth == CV_32F || CV_32S`, your local copy is missing the `np.float32` cast — pull the latest [`qr_detector_node.py`](qr_nav/qr_detector_node.py) from the laptop and rebuild.

---

## 4. Stage 2 — Full launch, motors diverted

Robot can be on the floor — `/cmd_vel` is remapped to `/cmd_vel_dummy` via the `cmd_vel_topic` launch argument, so the base controller never sees our messages. (Note: `ros2 launch` does not accept `--ros-args --remap` directly; remaps have to come through a declared launch argument — see [`qr_nav.launch.py`](launch/qr_nav.launch.py).)

```bash
ros2 launch qr_nav qr_nav.launch.py cmd_vel_topic:=/cmd_vel_dummy
```

Two more sessions:

```bash
ros2 topic echo /cmd_vel_dummy
ros2 topic echo /nav_state
```

Walk through this script with the cards. Every message has a `header` block (`stamp`, `frame_id: base_link`); the values you care about are nested under `twist:`.

| Card you show              | Expected `/cmd_vel_dummy` `twist`                | Expected `/nav_state`           |
| -------------------------- | ------------------------------------------------ | ------------------------------- |
| (none, just launched)      | `0.0, 0.0`                                       | `IDLE`                          |
| `GO` (first card)          | `0.08, 0.0`                                      | `IDLE` → `DRIVING`              |
| `STOP`                     | `0.0, 0.0`                                       | `STOPPED`                       |
| `TURN_LEFT` (while STOPPED)| (no change)                                      | (ignored — STOPPED only takes GO) |
| `GO` (while STOPPED)       | `0.08, 0.0`                                      | `STOPPED` → `DRIVING`           |
| `TURN_LEFT`                | `0.0, +0.4` for 4 s, then back to `0.08, 0.0`    | `TURNING` → `DRIVING`           |
| `TURN_RIGHT`               | `0.0, -0.4` for 4 s                              | `TURNING` → `DRIVING`           |
| `U_TURN`                   | `0.0, +0.4` for 8 s                              | `TURNING` → `DRIVING`           |
| `SPEED_UP`                 | `0.13, 0.0`                                      | `DRIVING`                       |
| `SPEED_DOWN`               | back to `0.08, 0.0`                              | `DRIVING`                       |

If you show a turn card or `SPEED_UP`/`SPEED_DOWN` as the *first* card out of `IDLE`, the FSM executes the command and ends in `DRIVING` — `IDLE` only protects against motion until the operator deliberately arms the bot.

If any line in the table fails, fix it before letting wheels move.

---

## 5. Stage 3 — Wheels off the ground (calibrate turns)

Lift the ROSbot onto a stable box. Full launch, **no remap**:

```bash
ros2 launch qr_nav qr_nav.launch.py
```

Re-run each card. Wheels should rotate the right direction/duration. **Calibrate `turn_90_duration` here:**

1. Mark a wheel position with tape.
2. Show `TURN_LEFT`. After 4 s, the wheel rotation should equal a 90° body turn in free space.
3. If it overshoots / undershoots, edit on the robot:

```bash
nano ~/par-a3/repo/src/qr_nav/config/qr_params.yaml
# adjust turn_90_duration up or down ~0.3 s, then:
cd ~/par-a3 && colcon build --packages-select qr_nav --symlink-install && source install/setup.bash
```

Repeat until 90° is repeatable. Then re-derive `turn_180_duration = 2 × turn_90_duration`.

---

## 6. Stage 4 — Floor run

**Pre-flight checklist:**

- Full battery (open-loop turns drift with battery sag)
- Clear floor, no fragile obstacles in the projected drive path
- `GO` card in your pocket (also arms the bot out of `IDLE` on launch, and unlocks `STOPPED`)
- Teleop terminal open as e-stop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Launch:

```bash
ros2 launch qr_nav qr_nav.launch.py
```

The robot boots in `IDLE` and **does not move** — `/nav_state` reports `IDLE`, `/cmd_vel` carries zero `twist`. Show a `GO` card to arm it at cruise speed; it then behaves exactly like Stage 2's `DRIVING` flow. Show `STOP` (or let it see a `STOP` card you've placed in its path) to halt; show `GO` to release from `STOPPED`.

Then escalate:

1. `GO` (arm from IDLE) → `STOP` → `GO` (recovery from STOPPED works)
2. `GO` → `TURN_LEFT` → `STOP` (turn integrated into a sequence)
3. Multi-card course of your design — keep cards ≥ 1 m apart so only one is in frame at a time
4. Final test: hold two different cards in one frame to validate the largest-bbox tiebreak in [`qr_detector_node.py`](qr_nav/qr_detector_node.py) `_detect_qr_codes`

### Live-run safety notes

- **`STOPPED` only exits via `GO`** — keep a `GO` card in your pocket.
- **`RECOVERING` is not a stop** — creeps at `min_speed` (0.05 m/s). With `recovery_timeout=3.0` it triggers fast if vision drops.
- **No obstacle avoidance in `DRIVING`** — if a `STOP` card is missed, the robot drives into whatever is past it. Hand on e-stop.
- **Open-loop turns drift with battery sag** — recheck `turn_90_duration` if battery dropped significantly between calibration and demo.
- **`/cmd_vel` shared with web UI / joystick** — if you accidentally grab the joystick or the web UI sends a Twist, it'll fight `qr_nav`'s output. Last message wins per packet.

---

## 7. Stage 5 — Adaptive lighting bring-up (optional, for dark-room operation)

The detector ships with an adaptive lighting monitor that classifies the scene as `BRIGHT` / `DIM` / `DARK` and, in `DARK`, switches to the OAK-D Pro's mono camera + IR floodlight. Switches only happen while the robot is stationary (`IDLE` / `STOPPED`); a switch requested mid-turn is deferred until the turn finishes, mid-drive triggers an immediate auto-stop, and resumption only happens after a valid QR is decoded on the new sensor.

This stage is **opt-in**. With `adaptive_lighting: true` (the default), the monitor *observes* on every launch but only changes anything when luma actually drops below the configured thresholds. In a normal lab it stays in `BRIGHT` forever and behaves identically to Stage 1–4.

> **Going deeper.** This section is the bring-up procedure (enable the feature, confirm it isn't completely broken). Once you're here, the **test procedures** for validating IR mode end-to-end — including the mid-turn deferred-switch edge case, the validation-timeout fail-safe, and what to capture for the report — live in [`TEST_IR_MODE.md`](TEST_IR_MODE.md).

### 7.1 One-time bot-side verification (do this once per OAK driver upgrade)

Husarion has renamed depthai topics + params across releases, so verify before relying on the defaults.

```bash
ssh husarion@192.168.1.150

ros2 topic list | grep oak                         # mono topic name
ros2 param list /oak 2>/dev/null | grep -iE 'flood|laser|ir'   # IR param names
ros2 topic info /oak/rgb/image_raw -v             # confirm RELIABLE / VOLATILE
```

Match these against the defaults in [`qr_params.yaml`](config/qr_params.yaml):

| YAML key                    | Default                            | What you're checking                                |
| --------------------------- | ---------------------------------- | --------------------------------------------------- |
| `mono_image_topic`          | `/oak/left/image_raw`              | `ros2 topic list` shows it under `/oak/`            |
| `depthai_node_name`         | `/oak`                             | The OAK camera node has this exact namespace        |
| `floodlight_param_name`     | `i_floodlight_brightness`          | `ros2 param list /oak` shows this param             |
| `dot_projector_param_name`  | `i_laser_dot_projector_current`    | Same. Older releases dropped the `i_` prefix.       |

If any name differs, edit the YAML — **no code change needed**. Then `colcon build --packages-select qr_nav --symlink-install` and continue.

If `AsyncParameterClient` isn't available on the robot's `rclpy` (pre-Iron), the detector will log a warning and skip IR control; lighting state is still observed and published, but you'll have to enable the floodlight manually with `ros2 param set /oak ...`.

### 7.2 Sanity-check the monitor (no motion)

Stationary detector, robot still on its box from Stage 3 or on the floor on `/cmd_vel_dummy`:

```bash
ros2 run qr_nav qr_detector \
  --ros-args --params-file ~/par-a3/install/qr_nav/share/qr_nav/config/qr_params.yaml
```

In a second session:

```bash
ros2 topic echo /qr_detector/lighting_state            # plain state name
ros2 topic echo /qr_detector/lighting_metrics          # JSON with luma_ema, decode_rate, etc.
```

You should see `BRIGHT` and a `luma_ema` value somewhere around 100–180 in normal indoor light. Now:

1. **Dim the lights** halfway. Wait `state_dwell_sec` (default 2 s). State should flip to `DIM`. `luma_ema` should drop into the 30–60 range.
2. **Lights off**. After another `state_dwell_sec`, state flips to `DARK`. If the IR controller succeeded, you'll see `IR applied: floodlight=1000 mA, projector=0 mA` in the detector log and the OAK-D's front face glows faintly red.
3. **Lights on again**. State recovers `DARK` → `DIM` → `BRIGHT` as you cross the upper hysteresis thresholds (`luma_dark + luma_bright_hysteresis`, etc.).

If transitions flicker rapidly at the boundary, increase `state_dwell_sec` or `luma_bright_hysteresis`. If transitions are too slow to be useful, lower them. Tune from the `lighting_metrics` log — don't guess.

### 7.3 Full handshake under motion

Wheels off the ground (Stage 3 setup) or on a `cmd_vel_topic:=/cmd_vel_dummy` floor (Stage 2 setup):

```bash
ros2 launch qr_nav qr_nav.launch.py cmd_vel_topic:=/cmd_vel_dummy
```

Watch all three in separate sessions:

```bash
ros2 topic echo /qr_detector/lighting_state
ros2 topic echo /qr_detector/switch_request
ros2 topic echo /qr_detector/switch_complete
ros2 topic echo /nav_state
```

Test sequence:

1. Show `GO` to arm. `nav_state` → `DRIVING`.
2. Kill the lights. Watch the sequence:
   - `lighting_state` flips to `DARK` after `state_dwell_sec`.
   - `switch_request: DARK` is published.
   - `nav_state` flips `DRIVING` → `STOPPED` (the detector saw `DRIVING` ⇒ immediate stop).
   - Detector log shows `Performing switch: subscribing to /oak/left/image_raw as MONO` and `IR applied: floodlight=1000 mA, projector=0 mA`.
3. Show **any** valid card to the camera (the IR-illuminated mono stream will see it).
   - `switch_complete: ok` is published.
   - `nav_state` flips `STOPPED` → `DRIVING`.
4. Now show `TURN_LEFT`, then *immediately* (mid-turn) lights on again. The `BRIGHT` transition's switch request should be **deferred** — `nav_state` stays `TURNING` until the turn completes, then goes `STOPPED` (not `DRIVING`). Then a valid QR on RGB resumes.

If step 4 transitions early (interrupts the turn), the pending-switch logic in [`command_interpreter_node.py`](qr_nav/command_interpreter_node.py) regressed — file a bug.

### 7.4 Common knobs (in [`qr_params.yaml`](config/qr_params.yaml))

| Knob                          | What it does                                                                    |
| ----------------------------- | ------------------------------------------------------------------------------- |
| `adaptive_lighting: false`    | Kill switch. Reverts to pre-adaptive behaviour exactly (RGB only, no IR).       |
| `luma_dim_threshold`          | Enter `DIM` below this 0–255 luma. Raise to be more aggressive about going dim. |
| `luma_dark_threshold`         | Enter `DARK` below this. Below ~20 the room is very dark for a phone camera.    |
| `luma_bright_hysteresis`      | How much luma has to *rise* before we leave a darker state. Stops flicker.      |
| `state_dwell_sec`             | Minimum stable time before a candidate state is promoted. Higher = lazier.      |
| `validation_timeout_sec`      | Max wait for first valid QR on the new sensor before giving up.                 |
| `ir_floodlight_dark_mA`       | OAK-D floodlight current in `DARK` (0–1500 mA). 1000 is a safe starting point. |

### 7.5 Disabling cleanly

If adaptive lighting causes a regression on demo day:

```bash
nano ~/par-a3/repo/src/qr_nav/config/qr_params.yaml
# set adaptive_lighting: false
cd ~/par-a3 && colcon build --packages-select qr_nav --symlink-install && source install/setup.bash
```

The detector returns to RGB-only operation with no IR poking. No code is removed; the monitor still computes luma silently but skips all transitions, sensor switches, and IR parameter calls.

---

## 8. Harvest the logs

Logs go to `/tmp/qr_nav_logs/qr_log_<timestamp>.csv` on the robot. `/tmp` wipes on reboot, so pull them off **before** powering down:

```bash
# from your laptop
mkdir -p logs
scp 'husarion@192.168.1.150:/tmp/qr_nav_logs/qr_log_*.csv' ./logs/
```

These are the raw data for the report's "Detection accuracy" and "Command execution accuracy" sections. Each row also includes the `lighting_state` published by the detector — useful for verifying that the adaptive monitor saw what you expected during the run. Organise by run, e.g. `logs/2026-05-09_calibration_1.csv`.

---

## Quick reference card

| Goal                                        | Command                                                                                                        |
| ------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Push code from laptop after edits           | `scp -r repo/src/qr_nav husarion@192.168.1.150:~/par-a3/repo/src/`                                             |
| Rebuild on robot after any change           | `cd ~/par-a3 && colcon build --packages-select qr_nav --symlink-install && source install/setup.bash`         |
| Confirm camera is publishing                | `ros2 topic hz /oak/rgb/image_raw`                                                                             |
| Confirm `/cmd_vel` type                     | `ros2 topic info /cmd_vel`                                                                                     |
| Watch decoded commands live                 | `ros2 topic echo /qr_command`                                                                                  |
| Watch FSM state live                        | `ros2 topic echo /nav_state`                                                                                   |
| Watch motor commands live                   | `ros2 topic echo /cmd_vel`                                                                                     |
| Watch lighting state live                   | `ros2 topic echo /qr_detector/lighting_state`                                                                  |
| Watch lighting metrics (luma, decode rate)  | `ros2 topic echo /qr_detector/lighting_metrics`                                                                |
| Watch adaptive-lighting switch handshake    | `ros2 topic echo /qr_detector/switch_request` (and `/qr_detector/switch_complete`)                             |
| Confirm OAK IR params exist                 | `ros2 param list /oak \| grep -iE 'flood\|laser\|ir'`                                                          |
| Manually force OAK floodlight on (test)     | `ros2 param set /oak i_floodlight_brightness 1000 && ros2 param set /oak i_laser_dot_projector_current 0`     |
| Disable adaptive lighting (kill switch)     | Edit [`qr_params.yaml`](config/qr_params.yaml) → `adaptive_lighting: false`, rebuild                            |
| Manual teleop (Jazzy stamped)               | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true`                              |
| Manual one-shot e-stop                      | `ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: base_link}}'`              |
| Full launch, motors live                    | `ros2 launch qr_nav qr_nav.launch.py`                                                                          |
| Full launch, motors diverted                | `ros2 launch qr_nav qr_nav.launch.py cmd_vel_topic:=/cmd_vel_dummy`                                            |
| Pull logs to laptop                         | `scp 'husarion@192.168.1.150:/tmp/qr_nav_logs/qr_log_*.csv' ./logs/`                                           |

---

## Failure-mode lookup

| Symptom                                                                       | Cause                                                                                       | Fix                                                                                                                       |
| ----------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `WARNING: topic [/camera/camera/color/image_raw] does not appear to be published yet` | Wrong topic in YAML                                                                          | Use `/oak/rgb/image_raw` — already the default in [`qr_params.yaml`](config/qr_params.yaml)                                |
| Detector runs, `/qr_command` never fires                                      | QoS mismatch — subscriber `BEST_EFFORT` against `RELIABLE` publisher                         | Already fixed; banner should say `qos: RELIABLE`                                                                          |
| Detector crashes with `Assertion failed: depth == CV_32F`                      | OpenCV 4.6 rejects `float64` from `.astype(float)`                                           | Already fixed; `np.float32` cast applied. Pull latest `qr_detector_node.py` if you see this.                              |
| `/cmd_vel` published but robot doesn't move                                   | Type mismatch — `Twist` vs `TwistStamped`                                                    | Already fixed; both `command_interpreter` and `data_logger` use `TwistStamped`                                            |
| Teleop keyboard doesn't move robot                                            | Same as above — teleop publishes `Twist` by default                                          | Use `--ros-args -p stamped:=true`                                                                                         |
| Robot turns ~45° instead of 90°                                               | `turn_90_duration` not re-derived after `turn_speed` change                                  | `duration = 1.5708 / turn_speed` — see comments in [`qr_params.yaml`](config/qr_params.yaml)                              |
| Robot stuck after a `STOP`                                                    | `STOPPED` only accepts `GO`                                                                  | Show `GO` card; or one-shot publish a `Twist` over teleop                                                                 |
| Robot keeps creeping after vision lost                                        | `RECOVERING` runs at `min_speed`, not zero                                                    | Send `STOP` card or use teleop e-stop                                                                                     |
| `IR partial/failed` warning, floodlight never turns on                        | `floodlight_param_name` / `dot_projector_param_name` don't match this depthai-ros release    | Run `ros2 param list /oak \| grep -iE 'flood\|laser\|ir'` and update the YAML to match. Rebuild.                          |
| `Could not build AsyncParameterClient ... IR control disabled`                | rclpy on the robot predates Iron, or `/oak` node is down                                     | If `/oak` is down: bring up the OAK driver first. If rclpy is old: set IR manually with `ros2 param set /oak ...`.        |
| Lighting state flickers between `BRIGHT` and `DIM` at boundary                | `state_dwell_sec` or `luma_bright_hysteresis` too low                                        | Raise both in [`qr_params.yaml`](config/qr_params.yaml); 2 s dwell + 20 hysteresis is the starting point.                 |
| Sensor switch requested but `nav_state` never reaches STOPPED                 | Interpreter not running, or `/qr_detector/switch_request` not connected                       | `ros2 topic info /qr_detector/switch_request` should show 1 subscriber. Restart `command_interpreter_node` if not.        |
| Sensor switch succeeds but `switch_complete: timeout`                         | No QR card in front of the new sensor inside `validation_timeout_sec`                         | Hold a QR card to the camera. Detector publishes `ok` on first valid decode, robot resumes DRIVING.                       |
| Adaptive lighting misfires on demo day                                         | Any of the above                                                                              | Kill switch: set `adaptive_lighting: false` in [`qr_params.yaml`](config/qr_params.yaml), rebuild. Stack reverts to RGB-only. |
| `colcon build` works but `ros2 run` shows old behaviour                        | Stale install: previous build was non-symlink                                                | `cd ~/par-a3 && rm -rf build install log && colcon build --packages-select qr_nav --symlink-install && source install/setup.bash` |
