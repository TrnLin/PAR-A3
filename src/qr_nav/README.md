# QR Code Command Navigation

**Project A — Programming Autonomous Robots (RMIT)**

An autonomous navigation system for the **Husarion ROSbot 3 PRO** that reads QR codes posted in the environment and executes the encoded navigation commands in real time, using a finite state machine for robust command-driven navigation.

## Quick Start (Single Command)

```bash
# 1. Build
cd ~/Documents/Coding/PAR-A3 && colcon build --packages-select qr_nav && source install/setup.bash

# 2. Run
ros2 launch qr_nav qr_nav.launch.py
```

**Prerequisites:** ROSbot 3 PRO hardware drivers must be running (camera, motors).

## Architecture

```
┌──────────────────────────────────────────────────────┐
│              ROSbot 3 PRO Hardware                    │
│  OAK-D Pro RGB (/camera/camera/color/image_raw)      │
└────────────────────────┬─────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────┐
│              QR DETECTOR NODE (qr_detector)           │
│  • OpenCV QRCodeDetector with detectAndDecodeMulti()  │
│  • Preprocessing: CLAHE + adaptive threshold fallback │
│  • Validates against 7 known commands                 │
│  • Multi-QR: selects largest bbox (closest code)      │
│  • 1.0s cooldown per command (debounce)               │
│  Publishes: /qr_command, /qr_detections              │
└────────────────────────┬─────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────┐
│       COMMAND INTERPRETER NODE (command_interpreter)  │
│  • FSM: DRIVING → TURNING → STOPPED → RECOVERING     │
│  • 7 commands mapped to velocity behaviors            │
│  • Timed turns (90°=2s, 180°=4s)                      │
│  • STOP waits for GO; RECOVERING on timeout           │
│  Publishes: /cmd_vel, /nav_state                      │
└────────────────────────┬─────────────────────────────┘
                         │
                         ▼
                  ROSbot Motors (/cmd_vel)

┌──────────────────────────────────────────────────────┐
│              DATA LOGGER NODE (qr_logger)             │
│  • Logs commands, detections, states to CSV           │
│  • Summary on shutdown: counts per command type       │
└──────────────────────────────────────────────────────┘
```

## Supported Commands

| QR Code Content | Robot Behaviour | FSM Transition |
|-----------------|-----------------|----------------|
| `TURN_LEFT` | 90° left turn | DRIVING → TURNING → DRIVING |
| `TURN_RIGHT` | 90° right turn | DRIVING → TURNING → DRIVING |
| `U_TURN` | 180° turn | DRIVING → TURNING → DRIVING |
| `STOP` | Halt, wait for GO | any → STOPPED |
| `GO` | Resume driving | STOPPED → DRIVING |
| `SPEED_UP` | +0.05 m/s (max 0.4) | stays DRIVING |
| `SPEED_DOWN` | -0.05 m/s (min 0.05) | stays DRIVING |

## FSM States

| State | Behaviour | Exit Condition |
|-------|-----------|----------------|
| `DRIVING` | Forward at cruise_speed | QR command or recovery_timeout |
| `TURNING` | Timed rotation, ignores commands | Turn duration elapsed |
| `STOPPED` | Zero velocity | GO command received |
| `RECOVERING` | Slow to min_speed | Any valid QR command |

## Edge Case Handling

- **Multiple simultaneous QR codes**: Selects the one with the largest bounding box area (closest to camera)
- **Degraded/partial codes**: Fallback preprocessing with adaptive thresholding
- **Oblique angles**: CLAHE contrast enhancement improves decode at angles up to ~45°
- **Rapid re-detection**: 1.0s cooldown prevents re-triggering the same command

## Configuration

All parameters in `config/qr_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cruise_speed` | 0.2 m/s | Base forward speed |
| `turn_speed` | 0.8 rad/s | Angular velocity for turns |
| `turn_90_duration` | 2.0 s | Time for 90° turn |
| `turn_180_duration` | 4.0 s | Time for U-turn |
| `recovery_timeout` | 5.0 s | Time before entering RECOVERING |
| `min_bbox_area` | 500 px | Ignore tiny QR detections |

## File Structure

```
src/qr_nav/
├── package.xml
├── setup.py / setup.cfg
├── config/
│   └── qr_params.yaml
├── launch/
│   └── qr_nav.launch.py
├── qr_nav/
│   ├── qr_detector_node.py           # QR detection + preprocessing
│   ├── command_interpreter_node.py    # FSM + velocity mapping
│   └── data_logger_node.py           # CSV logging
└── README.md
```
