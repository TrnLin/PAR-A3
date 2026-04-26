# Autonomous Traffic Light Obedience

**Project B — Programming Autonomous Robots (RMIT)**

An autonomous navigation system for the **Husarion ROSbot 3 PRO** that navigates a defined indoor corridor and responds correctly to colored signal cards (red/yellow/green) simulating traffic lights, using HSV color segmentation with depth-based spatial gating.

## Quick Start (Single Command)

```bash
# 1. Build
cd ~/Documents/Coding/PAR-A3 && colcon build --packages-select traffic_light_nav && source install/setup.bash

# 2. Run (full system with depth gating)
ros2 launch traffic_light_nav traffic_light_nav_full.launch.py

# 3. Run (without depth gating — for ablation study)
ros2 launch traffic_light_nav traffic_light_nav_no_depth.launch.py
```

**Prerequisites:** ROSbot 3 PRO hardware drivers must be running (camera, LIDAR, motors).

## Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                   ROSbot 3 PRO Hardware                        │
│  OAK-D Pro RGB + Depth  │  S2 LIDAR (/scan)                  │
└──────────┬──────────────┴──────────────┬──────────────────────┘
           │                              │
           ▼                              │
┌─────────────────────────────────────┐   │
│   SIGNAL DETECTOR NODE              │   │
│   (signal_detector)                 │   │
│                                     │   │
│   • HSV segmentation: R / Y / G     │   │
│   • Morphological filtering         │   │
│   • Contour area + aspect ratio     │   │
│   • Depth spatial gating:           │   │
│     - Distance < 1.5m               │   │
│     - Angle within ±20°             │   │
│   • Temporal confirmation (3 frames)│   │
│   • Priority: RED > YELLOW > GREEN  │   │
│                                     │   │
│   Publishes: /signal_state,         │   │
│              /signal_detections     │   │
└──────────────┬──────────────────────┘   │
               │                          │
               ▼                          ▼
┌─────────────────────────────────────────────────────────────┐
│          NAVIGATION CONTROLLER NODE                          │
│          (navigation_controller)                             │
│                                                              │
│   Signal FSM:                LIDAR Corridor Nav:             │
│   • GREEN  → cruise 0.25    • Wall centering                │
│   • YELLOW → cruise 0.125   • Front obstacle avoidance      │
│   • RED    → full stop      • Emergency stop < 0.2m         │
│   • UNKNOWN → hold last     • Proportional steering         │
│     (default YELLOW @3s)                                     │
│                                                              │
│   Publishes: /cmd_vel, /nav_state                            │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
                    ROSbot Motors

┌─────────────────────────────────────────────────────────────┐
│              DATA LOGGER NODE (traffic_logger)               │
│  • Logs signal states, detections, velocities, odom to CSV  │
│  • Tracks response latency per signal transition            │
│  • Summary: per-color counts, state distribution, latency   │
└─────────────────────────────────────────────────────────────┘
```

## Signal Detection Pipeline

1. **HSV Segmentation**: Convert RGB to HSV, apply per-color masks
   - Red: H[0-10] + H[170-180], S[100-255], V[80-255]
   - Yellow: H[20-35], S[100-255], V[100-255]
   - Green: H[40-80], S[80-255], V[80-255]
2. **Morphological Filtering**: Open + Close to remove noise
3. **Contour Analysis**: Filter by area (>800px) and aspect ratio (0.5-2.0)
4. **Depth Spatial Gating**: Only accept if centroid depth <1.5m AND within ±20° of heading
5. **Temporal Confirmation**: 3 consecutive frame detections required
6. **Priority**: RED > YELLOW > GREEN when multiple colors present

## Signal Responses

| Signal | Robot Behaviour | Speed |
|--------|-----------------|-------|
| Green | Resume/maintain full speed | 0.25 m/s |
| Yellow | Reduce speed, prepare to stop | 0.125 m/s |
| Red | Complete stop immediately | 0.0 m/s |
| Unknown | Hold last known state; default to Yellow after 3s | varies |

## False Positive Suppression

1. **Minimum contour area** (800px) eliminates small colored patches
2. **Aspect ratio filter** (0.5-2.0) rejects non-rectangular shapes
3. **Depth gating** ignores signals >1.5m away or >20° off-heading
4. **Temporal confirmation** requires 3 consecutive frames

## Ablation Study: Depth Gating

```bash
# With depth gating — logs to /tmp/traffic_light_logs/
ros2 launch traffic_light_nav traffic_light_nav_full.launch.py

# Without depth gating — logs to /tmp/traffic_light_logs/no_depth_gating/
ros2 launch traffic_light_nav traffic_light_nav_no_depth.launch.py
```

Compare CSV logs for:
- Detection accuracy (TP/FP/FN per color)
- False positive rate from non-signal colored objects
- Response latency differences

## Configuration

All parameters in `config/traffic_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cruise_speed` | 0.25 m/s | Full GREEN speed |
| `max_signal_distance` | 1.5 m | Depth gating threshold |
| `gate_angle_deg` | 20° | Angular gating half-width |
| `confirm_frames` | 3 | Frames to confirm detection |
| `min_signal_area` | 800 px | Min contour area |
| `unknown_timeout` | 3.0 s | UNKNOWN → default YELLOW |
| `use_depth_gating` | true | Toggle for ablation |

## File Structure

```
src/traffic_light_nav/
├── package.xml
├── setup.py / setup.cfg
├── config/
│   └── traffic_params.yaml
├── launch/
│   ├── traffic_light_nav_full.launch.py        # With depth gating
│   └── traffic_light_nav_no_depth.launch.py    # Without (ablation)
├── traffic_light_nav/
│   ├── signal_detector_node.py                 # HSV + depth gating
│   ├── navigation_controller_node.py           # Signal FSM + LIDAR nav
│   └── data_logger_node.py                     # CSV logging
└── README.md
```
