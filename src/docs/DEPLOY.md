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
| (none, just launched)      | `linear.x=0.08, angular.z=0.0`                   | `DRIVING`                       |
| `STOP`                     | `0.0, 0.0`                                       | `STOPPED`                       |
| `TURN_LEFT` (while STOPPED)| (no change)                                      | (ignored — STOPPED only takes GO) |
| `GO`                       | `0.08, 0.0`                                      | `DRIVING`                       |
| `TURN_LEFT`                | `0.0, +0.4` for 4 s, then back to `0.08, 0.0`    | `TURNING` → `DRIVING`           |
| `TURN_RIGHT`               | `0.0, -0.4` for 4 s                              | `TURNING` → `DRIVING`           |
| `U_TURN`                   | `0.0, +0.4` for 8 s                              | `TURNING` → `DRIVING`           |
| `SPEED_UP`                 | `0.13, 0.0`                                      | `DRIVING`                       |
| `SPEED_DOWN`               | back to `0.08, 0.0`                              | `DRIVING`                       |

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
- `GO` card in your pocket (manual STOPPED-trap unlocker)
- Teleop terminal open as e-stop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Lay out **one** `STOP` card 1 m in front of the robot. Launch:

```bash
ros2 launch qr_nav qr_nav.launch.py
```

Robot drives forward at 0.08 m/s, sees `STOP`, halts, sits in `STOPPED`. Show `GO` to release.

Then escalate:

1. `STOP` → `GO` (recovery from STOPPED works)
2. `STOP` → `GO` → `TURN_LEFT` → `STOP` (turn integrated into a sequence)
3. Multi-card course of your design — keep cards ≥ 1 m apart so only one is in frame at a time
4. Final test: hold two different cards in one frame to validate the largest-bbox tiebreak in [`qr_detector_node.py`](qr_nav/qr_detector_node.py) `_detect_qr_codes`

### Live-run safety notes

- **`STOPPED` only exits via `GO`** — keep a `GO` card in your pocket.
- **`RECOVERING` is not a stop** — creeps at `min_speed` (0.05 m/s). With `recovery_timeout=3.0` it triggers fast if vision drops.
- **No obstacle avoidance in `DRIVING`** — if a `STOP` card is missed, the robot drives into whatever is past it. Hand on e-stop.
- **Open-loop turns drift with battery sag** — recheck `turn_90_duration` if battery dropped significantly between calibration and demo.
- **`/cmd_vel` shared with web UI / joystick** — if you accidentally grab the joystick or the web UI sends a Twist, it'll fight `qr_nav`'s output. Last message wins per packet.

---

## 7. Harvest the logs

Logs go to `/tmp/qr_nav_logs/qr_log_<timestamp>.csv` on the robot. `/tmp` wipes on reboot, so pull them off **before** powering down:

```bash
# from your laptop
mkdir -p logs
scp 'husarion@192.168.1.150:/tmp/qr_nav_logs/qr_log_*.csv' ./logs/
```

These are the raw data for the report's "Detection accuracy" and "Command execution accuracy" sections. Organise by run, e.g. `logs/2026-05-09_calibration_1.csv`.

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
| `colcon build` works but `ros2 run` shows old behaviour                        | Stale install: previous build was non-symlink                                                | `cd ~/par-a3 && rm -rf build install log && colcon build --packages-select qr_nav --symlink-install && source install/setup.bash` |
