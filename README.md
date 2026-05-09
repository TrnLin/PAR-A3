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

## 🏗️ System Architecture

The system is structured as three primary ROS 2 nodes communicating over standard topics:

```text
┌──────────────────────────────────────────────────────┐
│              ROSbot 3 PRO Hardware                   │
│  OAK-D Pro RGB (/oak/rgb/image_raw)                  │
└────────────────────────┬─────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────┐
│              QR DETECTOR NODE (qr_detector)          │
│  • OpenCV QRCodeDetector with detectAndDecodeMulti() │
│  • Preprocessing: CLAHE + adaptive threshold fallback│
│  • Validates against 7 known commands                │
│  • Multi-QR: selects largest bbox (closest code)     │
│  • 1.0s cooldown per command (debounce)              │
│  Publishes: /qr_command, /qr_detections              │
└────────────────────────┬─────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────┐
│       COMMAND INTERPRETER NODE (command_interpreter) │
│  • FSM: DRIVING → TURNING → STOPPED → RECOVERING     │
│  • 7 commands mapped to velocity behaviors           │
│  • Timed turns (90°=2s, 180°=4s)                     │
│  • STOP waits for GO; RECOVERING on timeout          │
│  Publishes: /cmd_vel, /nav_state                     │
└────────────────────────┬─────────────────────────────┘
                         │
                         ▼
                  ROSbot Motors (/cmd_vel)

┌──────────────────────────────────────────────────────┐
│              DATA LOGGER NODE (qr_logger)            │
│  • Logs commands, detections, states to CSV          │
│  • Summary on shutdown: counts per command type      │
└──────────────────────────────────────────────────────┘
```

## 🧠 Navigation Policy (FSM)
A Finite State Machine ensures reliable transitions between behaviors and prevents conflicting commands:

| QR Code Content | Robot Behaviour | FSM Transition |
|-----------------|-----------------|----------------|
| `TURN_LEFT` | 90° left turn | `DRIVING` -> `TURNING` -> `DRIVING` |
| `TURN_RIGHT` | 90° right turn | `DRIVING` -> `TURNING` -> `DRIVING` |
| `U_TURN` | 180° turn | `DRIVING` -> `TURNING` -> `DRIVING` |
| `STOP` | Halt, wait for GO | any -> `STOPPED` |
| `GO` | Resume driving | `STOPPED` -> `DRIVING` |
| `SPEED_UP` | +0.05 m/s (max 0.4) | stay `DRIVING` |
| `SPEED_DOWN` | -0.05 m/s (min 0.05) | stay `DRIVING` |

### FSM Behaviours

| State | Behaviour | Exit Condition |
|-------|-----------|----------------|
|`DRIVING`|Forward at `cruise_speed`| QR command or `recovery_timeout` |
| `TURNING` | Timed rotation, ignores commands | Turn duration elapsed |
| `STOPPED` | Zero velocity | `GO` command received |
| `RECOVERING` | Slow to `min_speed` | Any valid QR command |

**Edge Case Handling**
- **Simultaneous Detections**: Spatial filtering selects the QR code with the largest bounding box area (closest to the camera).

- **Degraded Codes**: Fallback OpenCV adaptive thresholding pipeline activates if standard CLAHE enhancement fails.

- **Oblique Angles**: Contrast enhancement improves decoding at extreme viewing angles (up to 45°).

- **Signal Loss**: A configurable `recovery_timeout` drops the robot into a safe `RECOVERING` crawl state if visual cues are lost.

## 📦 Folder Structure
```cmd
PAR-A3/
├── qr_cards/                               # Generated printable QR cards
├── public/                               
├── src/
│   └── qr_nav/
│       ├── config/
│       │   └── qr_params.yaml              # Tunable speeds, thresholds & QoS
│       ├── launch/
│       │   └── qr_nav.launch.py
│       ├── qr_nav/
│       │   ├── qr_detector_node.py         # OAK-D perception & validation
│       │   ├── command_interpreter_node.py # FSM & velocity publisher
│       │   └── data_logger_node.py         # CSV metric logging
|       ├── utils/
|       |   └── generate_qr_cards.py        # AST-parser to sync cards with codebase
│       ├── package.xml
│       └── setup.py
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