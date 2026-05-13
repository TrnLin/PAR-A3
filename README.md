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
- **Adapt to ambient lighting** — automatically switch from the RGB sensor to the OAK-D Pro's IR-illuminated mono camera when the room goes dark, with a stationary handshake so the swap never interrupts a turn.

## 🧰 Hardware

| Component | Specification |
|-----------|--------------|
| Robot Platform | Husarion ROSbot 3 PRO |
| Primary Sensor | OAK-D Pro — RGB (12 MP) + 2× IR-sensitive mono cameras + 850 nm IR floodlight |
| Compute | Onboard ROS 2 capable processor (Raspberry Pi 5) |
| Actuation | ROSbot Motor Controller |

<div align="center">
    <picture>
        <source media="(prefers-color-scheme: dark)" srcset="public/rosbot3_dark.png">
        <source media="(prefers-color-scheme: light)" srcset="public/rosbot3_light.png">
        <img alt="ROSbot3 component">
    </picture>
</div>

## 🏗️ System Architecture

The system is structured as three primary ROS 2 nodes communicating over standard topics. The detector switches between the OAK-D Pro's RGB stream and its IR-illuminated mono stream based on ambient lighting, coordinated with the interpreter via a small handshake so swaps only happen while the robot is stationary.

```text
┌────────────────────────────────────────────────────────────────┐
│                  ROSbot 3 PRO  ·  OAK-D Pro                    │
│   /oak/rgb/image_raw   (BRIGHT/DIM)                            │
│   /oak/left/image_raw  (DARK, mono + IR floodlight)            │
│   /oak  parameter node — i_floodlight_brightness, projector    │
└──────────────────────────────────┬─────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────┐
│              QR DETECTOR NODE (qr_detector)                    │
│  • OpenCV QRCodeDetector with detectAndDecodeMulti()           │
│  • Preprocessing: CLAHE + adaptive threshold fallback          │
│  • Validates against 7 known commands                          │
│  • Multi-QR: selects largest bbox (closest code)               │
│  • 1.0s cooldown per command (debounce)                        │
│  • Lighting monitor (BRIGHT/DIM/DARK) with hysteresis + dwell  │
│  • Switch handshake: gates on /nav_state == IDLE/STOPPED       │
│  • IR floodlight control via AsyncParameterClient('/oak')      │
│  Publishes: /qr_command, /qr_detections,                       │
│             /qr_detector/lighting_state, /lighting_metrics,    │
│             /qr_detector/switch_request, /switch_complete      │
└──────────────────────────────────┬─────────────────────────────┘
                                   │  /qr_command,
                                   │  switch_request / switch_complete
                                   ▼
┌────────────────────────────────────────────────────────────────┐
│       COMMAND INTERPRETER NODE (command_interpreter)           │
│  • FSM: IDLE, DRIVING, TURNING, STOPPED, RECOVERING            │
│  • 7 commands mapped to velocity behaviors                     │
│  • Timed turns (90°=2s, 180°=4s)                               │
│  • STOP waits for GO; RECOVERING on timeout                    │
│  • Switch handshake: pending_switch routes turn completion     │
│    to STOPPED; auto-resume on switch_complete:ok               │
│  Publishes: /cmd_vel (TwistStamped), /nav_state                │
└──────────────────────────────────┬─────────────────────────────┘
                                   │
                                   ▼
                       ROSbot Motors (/cmd_vel)

┌────────────────────────────────────────────────────────────────┐
│              DATA LOGGER NODE (qr_logger)                      │
│  • Logs commands, detections, FSM state, lighting state to CSV │
│  • Summary on shutdown: counts per command + state distributions│
└────────────────────────────────────────────────────────────────┘
```

## 🧠 Navigation Policy (FSM)
A Finite State Machine ensures reliable transitions between behaviors and prevents conflicting commands. The FSM boots into `IDLE` (stationary) so the robot never moves on launch alone — the first valid QR card arms it.

| QR Code Content | Robot Behaviour | FSM Transition |
|-----------------|-----------------|----------------|
| `TURN_LEFT` | 90° left turn | `IDLE`\|`DRIVING` -> `TURNING` -> `DRIVING` |
| `TURN_RIGHT` | 90° right turn | `IDLE`\|`DRIVING` -> `TURNING` -> `DRIVING` |
| `U_TURN` | 180° turn | `IDLE`\|`DRIVING` -> `TURNING` -> `DRIVING` |
| `STOP` | Halt, wait for GO | any -> `STOPPED` |
| `GO` | Resume driving | `IDLE`\|`STOPPED` -> `DRIVING` |
| `SPEED_UP` | +0.05 m/s (max 0.4) | `IDLE`\|`DRIVING` -> `DRIVING` |
| `SPEED_DOWN` | -0.05 m/s (min 0.05) | `IDLE`\|`DRIVING` -> `DRIVING` |

### FSM Behaviours

| State | Behaviour | Exit Condition |
|-------|-----------|----------------|
| `IDLE` | Zero velocity on boot, never auto-moves | Any valid QR command |
|`DRIVING`|Forward at `cruise_speed`| QR command or `recovery_timeout` |
| `TURNING` | Timed rotation, ignores commands | Turn duration elapsed |
| `STOPPED` | Zero velocity (operator halt) | `GO` command received |
| `RECOVERING` | Slow to `min_speed` | Any valid QR command |

**Edge Case Handling**
- **Simultaneous Detections**: Spatial filtering selects the QR code with the largest bounding box area (closest to the camera).

- **Degraded Codes**: Fallback OpenCV adaptive thresholding pipeline activates if standard CLAHE enhancement fails.

- **Oblique Angles**: Contrast enhancement improves decoding at extreme viewing angles (up to 45°).

- **Signal Loss**: A configurable `recovery_timeout` drops the robot into a safe `RECOVERING` crawl state if visual cues are lost.

- **Ambient Lighting**: An adaptive lighting monitor classifies the scene as `BRIGHT` / `DIM` / `DARK` from frame luma + decode-success rate, and switches the detector between the RGB and IR-illuminated mono cameras automatically. See the *Adaptive Lighting* section below.

## 🌗 Adaptive Lighting

The OAK-D Pro ships with two **IR-sensitive mono cameras** and an **850 nm IR floodlight** in addition to the RGB sensor. By default we use only RGB; the adaptive monitor extends this so the robot keeps working in dim or fully dark rooms.

### Three lighting states

| State | Sensor used | IR floodlight | Dot projector |
|-------|-------------|---------------|---------------|
| `BRIGHT` | `/oak/rgb/image_raw` (bgr8) | 0 mA | 0 mA |
| `DIM` | `/oak/rgb/image_raw` (bgr8) | ~300 mA | 0 mA |
| `DARK` | `/oak/left/image_raw` (mono8) | ~1000 mA | 0 mA |

The dot projector is **always** forced to 0 — its speckle pattern destroys QR decoding.

### Safe-switch handshake

Sensor swaps only happen while the robot is stationary (`IDLE` or `STOPPED`). The detector publishes `/qr_detector/switch_request`; the interpreter responds based on its FSM state:

| Current FSM state | Behaviour on switch request |
|-------------------|------------------------------|
| `IDLE` / `STOPPED` | Already safe — re-publish `/nav_state` so the detector proceeds immediately. |
| `DRIVING` / `RECOVERING` | Transition to `STOPPED` immediately (no in-flight action to finish). |
| `TURNING` | Set `pending_switch`; finish the turn, then route to `STOPPED` instead of `DRIVING`. Mid-turn interrupts are avoided. |

Once stationary, the detector resubscribes to the new sensor, enters a `VALIDATING` phase, and waits for the **first valid QR decode** on the new feed. On success it publishes `/qr_detector/switch_complete: ok` and the interpreter resumes `STOPPED → DRIVING` automatically. On timeout (`validation_timeout_sec`, default 10 s) the robot stays `STOPPED` until the operator shows a `GO` card.

### Kill switch

Set `adaptive_lighting: false` in [`src/config/qr_params.yaml`](src/config/qr_params.yaml) to revert to pre-adaptive behaviour (RGB only, no IR control, no handshake). The lighting monitor still observes silently but takes no action.

Full bring-up procedure for dark-room operation is in [`src/docs/DEPLOY.md`](src/docs/DEPLOY.md) (Stage 5). Step-by-step **test procedures** (decoding under IR, edge-case verification, what to capture for the report) are in [`src/docs/TEST_IR_MODE.md`](src/docs/TEST_IR_MODE.md). Deeper design rationale, the classifier internals, and the in-scope / out-of-scope boundary are documented in [`src/docs/qr_nav_explained.html`](src/docs/qr_nav_explained.html) (section 11).

## 📦 Folder Structure
```cmd
PAR-A3/
├── qr_cards/                                 # Generated printable QR cards
├── public/                                   # Static assets (figures)
├── src/
│   ├── config/
│   │   └── qr_params.yaml                    # Tunable speeds, thresholds, QoS,
│   │                                         # adaptive-lighting + IR knobs
│   ├── launch/
│   │   └── qr_nav.launch.py
│   ├── qr_nav/
│   │   ├── qr_detector_node.py               # OAK-D perception, lighting monitor,
│   │   │                                     # switch handshake, IR control
│   │   ├── command_interpreter_node.py       # FSM, velocity publisher,
│   │   │                                     # pending-switch handling
│   │   └── data_logger_node.py               # CSV metric logging incl. lighting_state
│   ├── utils/
│   │   └── generate_qr_cards.py              # AST-parser to sync cards with codebase
│   ├── docs/
│   │   ├── DEPLOY.md                         # End-to-end deployment guide
│   │   ├── TEST_IR_MODE.md                   # Hands-on IR / dark-room test procedures
│   │   ├── LESSONS_LEARNED.md                # Forensic record of bring-up issues
│   │   └── qr_nav_explained.html             # Long-form walkthrough of the pipeline
│   ├── package.xml
│   └── setup.py
├── README.md
└── requirements.txt
```

## ⚙️ Installation & Setup

### 1. Clone & Intall Dependencies
```bash
git clone https://github.com/TrnLin/PAR-A3.git
cd PAR-A3

# Create and activate virtual environment (for tools & linting)
python -m venv venv (`macOS`: python3 -m venv venv) # Create and activate a virtual environment
.\venv\Scripts\Activate.ps1 (`macOS`: source venv/bin/activate)
pip install -r .\requirements.txt (`macOS`: pip install -r requirements.txt)  # Install Python dependencies
deactivate # deactive venv when finished
```

### 2. Generate Printable QR Cards

To ensure printed QR codes are perfectly synchronized with the detector node's accepted commands, generate the test cards locally:

```bash
python src/utils/generate_qr_cards.py
```

(Print the resulting `qr_cards/all_commands.pdf` at 100% scale).

### 3. Build the ROS2 Workspace
```bash
colcon build --packages-select qr_nav
source install/setup.bash
```

## 🚀 Running the System
Ensure the ROSbot hardware drivers (OAK-D camera and motor controllers) are active, then launch the system with a single command:

```bash
ros2 launch qr_nav qr_nav.launch.py
```

All parameters (speeds, timeouts, bounding box limits) can be tuned without recompiling via `src/qr_nav/config/qr_params.yaml`.

## 📊 Evaluation Metrics
Data for evaluation is automatically logged to CSV files in `/tmp/qr_nav_logs` by the `data_logger_node`. The evaluation rubric requires analysis of $10+$ runs covering:

| Metric | Description |
|--------|-------------|
| Detection Accuracy | % of QR codes correctly detected and decoded. |
| Command Execution Accuracy | % of correct maneuvers performed in response to decoded strings. |
| Robustness | Performance under varying lighting, angles ($0^{\circ}$, $30^{\circ}$, $45^{\circ}$), and distances (0.3m, 0.6m, 1.0m). |
| Low-Light Operation | Adaptive-lighting transitions across `BRIGHT`/`DIM`/`DARK`, sensor switch success rate, validation-timeout rate, and detection accuracy under IR illumination. Logged per-run via the `lighting_state` CSV column and the `/qr_detector/lighting_metrics` topic. |
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