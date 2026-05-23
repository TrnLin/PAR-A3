<div align="center">

# QR Code Command Navigation
### Visual Instruction Processing on ROSbot 3 PRO

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-yellow?logo=python&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-ROSbot_3_PRO-red?logo=linux&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Active-brightgreen)
 
**Deterministic. Machine-readable. Spatially Anchored.**

A real-time autonomous navigation system that enables the **Husarion ROSbot 3 PRO** to read, interpret, and execute visual QR code commands in dynamic indoor environments using a robust **Finite State Machine (FSM)**.

</div>

## 📌 Project Overview

This project simulates real-world logistics and warehouse robotics (e.g., Amazon Robotics fiducial marker navigation). Rather than relying on pre-programmed maps, the robot dynamically reads environmental cues (QR codes) to guide its behavior. The system must process the full perception-to-action pipeline:

- **Detect and decode** visual signals using onboard RGB cameras.
- **Map decoded instructions** to precise velocity and rotation commands.
- **Handle edge cases** (degraded codes, oblique angles, simultaneous detections).
- **Recover gracefully** if vision drops out or codes cannot be decoded.

## 🧰 Hardware

| Component | Specification |
|-----------|--------------|
| Robot Platform | Husarion ROSbot 3 PRO |
| Primary Sensor | OAK-D Pro RGB Camera |
| Compute | Onboard ROS 2 capable processor |
| Actuation | ROSbot Motor Controller |

<div align="center">
    <picture>
        <source media="(prefers-color-scheme: dark)" srcset="public/rosbot3_dark.png">
        <source media="(prefers-color-scheme: light)" srcset="public/rosbot3_light.png">
        <img alt="ROSbot3 component">
    </picture>
</div>

## 🧠 Navigation Policy (FSM)
A Finite State Machine ensures reliable transitions between behaviors and prevents conflicting commands. The FSM boots into `IDLE` (stationary) so the robot never moves on launch alone — the first valid QR card arms it.

| QR Code Content | Robot Behaviour | FSM Transition |
|-----------------|-----------------|----------------|
| `TURN_LEFT` | 90° left turn | `DRIVING` -> `TURNING` -> `DRIVING` |
| `TURN_RIGHT` | 90° right turn | `DRIVING` -> `TURNING` -> `DRIVING` |
| `U_TURN` | 180° turn | `DRIVING` -> `TURNING` -> `DRIVING` |
| `STOP` | Halt, wait for GO | any -> `STOPPED` |
| `GO` | Resume driving | `IDLE`\|`STOPPED` -> `DRIVING` |
| `SPEED_UP` | +0.05 m/s (max 0.4) | `DRIVING` -> `DRIVING` |
| `SPEED_DOWN` | -0.05 m/s (min 0.05) | `DRIVING` -> `DRIVING` |

### FSM Behaviours

The FSM boots into `IDLE` (stationary) so the robot never moves on launch alone - a `GO` command is strictly required to arm it.

| State | Behaviour | Exit Condition |
|-------|-----------|----------------|
| `IDLE` | Zero velocity on boot, strictly locked | `GO` command received |
|`DRIVING`|Forward at `cruise_speed`| QR command or `recovery_timeout` |
| `TURNING` | Timed rotation, ignores commands | Turn duration elapsed |
| `STOPPED` | Zero velocity (operator halt) | `GO` command received |
| `RECOVERING` | Slow to `min_speed` | Any valid QR command |

**Edge Case Handling**
- **Simultaneous Detections**: Spatial filtering selects the QR code with the largest bounding box area (closest to the camera).

- **Degraded Codes**: Fallback OpenCV adaptive thresholding pipeline activates if standard CLAHE enhancement fails.

- **Oblique Angles**: Contrast enhancement improves decoding at extreme viewing angles (up to 45°).

- **Signal Loss**: A configurable `recovery_timeout` drops the robot into a safe `RECOVERING` crawl state if visual cues are lost.

## 📦 Folder Structure
```text
repo/
├── qr_cards/                               # Generated printable QR cards
├── public/                                 # README images
├── logs/                                   # session_<TS>/... captured by qr_logging_env.sh
├── results/                                # CSV trials + analysis notebooks
├── src/                                    # ament_python ROS 2 package root (qr_nav)
│   ├── package.xml                         # ROS 2 manifest
│   ├── setup.py                            # entry points: qr_detector / command_interpreter / qr_logger
│   ├── config/
│   │   └── qr_params.yaml                  # Tunable speeds, thresholds & QoS
│   ├── launch/
│   │   └── qr_nav.launch.py
│   ├── qr_nav/
│   │   ├── qr_detector_node.py             # OAK-D perception & validation
│   │   ├── command_interpreter_node.py     # FSM & velocity publisher
│   │   └── data_logger_node.py             # CSV metric logging
│   ├── utils/
│   │   └── generate_qr_cards.py            # AST-parser to sync cards with detector
│   └── docs/                               # DEPLOY.md, ARCHITECTURE.md, lessons learned
├── tools/                                  # qr_install_and_run.sh + qr_*.sh stage scripts
├── README.md
└── requirements.txt
```

## ⚙️ Installation & Setup

> **Prerequisites:** ROS 2 **Jazzy** sourced in the current shell (`source /opt/ros/jazzy/setup.bash`), `python3`, `python3-colcon-common-extensions`, and on the robot `ros-jazzy-cv-bridge` + `python3-opencv`. For live runs you also need the ROSbot OAK-D driver publishing `/oak/rgb/image_raw` and `/cmd_vel` available as `geometry_msgs/msg/TwistStamped`.

### Quick start (one command)

[`tools/qr_install_and_run.sh`](tools/qr_install_and_run.sh) creates the Python venv, installs `requirements.txt`, builds the colcon workspace with `--symlink-install`, sources the overlay, and dispatches to the matching staged-deploy script under `tools/`. Run from the repo root:

```bash
git clone https://github.com/TrnLin/PAR-A3.git
cd PAR-A3/repo

./tools/qr_install_and_run.sh --mode stage2     # safe: /cmd_vel diverted to /cmd_vel_dummy
./tools/qr_install_and_run.sh --mode stage4     # live: real motion on /cmd_vel
```

Other supported modes (delegate to the existing stage scripts in [`tools/`](tools/)):

| `--mode` | Underlying script | Use for |
|----------|-------------------|---------|
| `preflight` | [`tools/qr_preflight.sh`](tools/qr_preflight.sh) | Camera + `/cmd_vel` checks, no motion |
| `stage1` | [`tools/qr_stage1_detect.sh`](tools/qr_stage1_detect.sh) | Detector only, robot stationary |
| `stage2` | [`tools/qr_stage2_diverted.sh`](tools/qr_stage2_diverted.sh) | Full launch, motors diverted (safe on floor) |
| `stage3` | [`tools/qr_stage3_wheels_off.sh`](tools/qr_stage3_wheels_off.sh) | Full launch, robot lifted (turn calibration) |
| `stage4` | [`tools/qr_stage4_floor.sh`](tools/qr_stage4_floor.sh) | Live floor run |

Useful flags: `--skip-build` reuses an existing `install/` overlay; `--no-venv` skips the local Python tooling install; everything after a literal `--` is forwarded as a launch argument, e.g. `./tools/qr_install_and_run.sh --mode stage2 -- session_id:=trial_a`.

### Manual setup (advanced)

If you prefer to drive each step yourself, the wrapper above runs the equivalent of:

```bash
git clone https://github.com/TrnLin/PAR-A3.git
cd PAR-A3/repo

python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
deactivate

python3 src/utils/generate_qr_cards.py          # (re)build qr_cards/all_commands.pdf; print at 100%

colcon build --packages-select qr_nav --symlink-install
source install/setup.bash

ros2 launch qr_nav qr_nav.launch.py             # add cmd_vel_topic:=/cmd_vel_dummy for safe runs
```

All velocity and timeout parameters are tunable without recompiling via [`src/config/qr_params.yaml`](src/config/qr_params.yaml). Operator-facing staged deployment, calibration, and troubleshooting notes live in [`src/docs/DEPLOY.md`](src/docs/DEPLOY.md).

## 📊 Evaluation Metrics
When launched through `tools/qr_install_and_run.sh` (or any of the `tools/qr_stage*.sh` scripts), the auto-logging hook in [`tools/qr_logging_env.sh`](tools/qr_logging_env.sh) captures each run under `logs/session_<TS>/<stage>_<TS>/`, including the `data_logger_node` CSV. The evaluation rubric requires analysis of $10+$ runs covering:

| Metric | Description |
|--------|-------------|
| Detection Accuracy | % of QR codes correctly detected and decoded. |
| Command Execution Accuracy | % of correct maneuvers performed in response to decoded strings. |
| Robustness | Performance under varying lighting, angles ($0^{\circ}$, $30^{\circ}$, $45^{\circ}$), and distances (0.3m, 0.6m, 1.0m). |
| Failure Analysis | Categorization of failure modes (e.g., missed detections vs. false positive states). |

## 👥 Contribution

| Full Name | Student ID | Link |
|------------|------|------|
| Hoang Minh Thang | s3999925 | <a href="https://github.com/ThangHoang54" target="_blank"><img src="https://skillicons.dev/icons?i=github" width="20px" /></a> |
| Truong Ba An | s3999568 | <a href="https://github.com/truongbaan" target="_blank"><img src="https://skillicons.dev/icons?i=github" width="20px" /></a> | 
| Tran Hoang Linh | s4043097 | <a href="https://github.com/TrnLin" target="_blank"><img src="https://skillicons.dev/icons?i=github" width="20px" /></a> | 

## 📜 License

This project is submitted as academic coursework at RMIT University. Code is shared under the [MIT License](LICENSE) for educational reference only.

## 📚 Resources

- [ROSbot 3 PRO Quick Start Guide](https://husarion.com/tutorials/howtostart/rosbot3-quick-start/)
- [ROSbot 3 PRO Hardware Manual](https://husarion.com/manuals/rosbot/)

## 🏛️ Acknowledgments

This project was developed as **Assignment 3** for **COSC3070: Programming Autonomous Robots** - Semester 2026A at **RMIT University**.

Assignment specification provided by **Dr. Ginel Dorleon**.