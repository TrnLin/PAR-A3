# Reactive Autonomous Navigation with Obstacle Avoidance

**Project C — Programming Autonomous Robots (RMIT)**

A purely reactive navigation system for the **Husarion ROSbot 3 PRO** that roams freely through unknown indoor environments, detecting and avoiding obstacles in real time with no prior map, no goal, and no memory.

## Quick Start (Single Command)

```bash
# 1. Build (from workspace root, i.e. the PAR-A3 directory)
cd ~/Documents/Coding/PAR-A3 && colcon build --packages-select reactive_nav && source install/setup.bash

# 2. Run full system (LIDAR + depth + ToF)
ros2 launch reactive_nav reactive_nav_full.launch.py

# 3. Run LIDAR-only (for sensor ablation study)
ros2 launch reactive_nav reactive_nav_lidar_only.launch.py
```

**Prerequisites:** The ROSbot 3 PRO hardware drivers must already be running (LIDAR, camera, motors). These are typically started via the Husarion snaps (`rosbot`, `husarion-depthai`, `husarion-rplidar`).

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROSbot 3 PRO Hardware                       │
│  S2 LIDAR (/scan)  │  OAK-D Pro (/camera/.../depth)  │  ToF x4 │
└────────┬───────────┴──────────┬────────────────────┴────┬───────┘
         │                      │                          │
         ▼                      ▼                          ▼
┌────────────────────────────────────────────────────────────────┐
│              SENSOR FUSION NODE (sensor_fusion)                │
│  • LIDAR → 360° sector mapping                                │
│  • Depth → front-sector mapping (~72° FOV)                    │
│  • ToF → point sector mapping                                 │
│  • Per-sector minimum across all sensors                      │
│  • Dynamic obstacle detection (temporal differencing)         │
│  Publishes: /obstacle_sectors, /dynamic_obstacles             │
└──────────────────────────┬────────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────────┐
│          REACTIVE NAVIGATOR NODE (reactive_navigator)          │
│  • Virtual Force Field (VFF) algorithm                        │
│  • Behavioural state machine:                                 │
│    FREE_ROAM → CAREFUL → NARROW_PASSAGE                       │
│         ↓          ↓                                          │
│    AVOIDING ← DEAD_END_RECOVERY                               │
│  • Random exploration bias for coverage                       │
│  Publishes: /nav_cmd_vel, /nav_state                          │
└──────────────────────────┬────────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────────┐
│            SAFETY MONITOR NODE (safety_monitor)                │
│  • Subscribes to front ToF sensors                            │
│  • HARD emergency stop if ANY ToF < 0.10m                     │
│  • Hysteresis: resume when ALL ToF > 0.25m                    │
│  • Forwards /nav_cmd_vel → /cmd_vel when safe                 │
│  Publishes: /cmd_vel, /safety_status                          │
└──────────────────────────┬────────────────────────────────────┘
                           │
                           ▼
                    ROSbot Motors

┌────────────────────────────────────────────────────────────────┐
│              DATA LOGGER NODE (data_logger)                    │
│  • Logs odometry, sectors, states, collisions to CSV          │
│  • Periodic summary (collisions/min, distance covered)        │
│  • Output: /tmp/reactive_nav_logs/nav_log_YYYYMMDD_HHMMSS.csv │
└────────────────────────────────────────────────────────────────┘
```

## Sensor Fusion Strategy

The environment is divided into **12 angular sectors** of 30° each, centred on the robot:

```
        Sector 2    Sector 1    Sector 0    Sector 11   Sector 10
        (LEFT-FWD)  (FWD-LEFT)  (FRONT)     (FWD-RIGHT) (RIGHT-FWD)
             \          |          |          |          /
              \         |          |          |         /
               \        |          |          |        /
  Sector 3 ----\--------+----------+----------+------/---- Sector 9
  (LEFT)        \       |          |          |      /      (RIGHT)
                 \      |        ROBOT        |     /
  Sector 4 ------+------+----------+----------+---+------ Sector 8
  (LEFT-REAR)     \     |          |          |   /       (RIGHT-REAR)
                   \    |          |          |  /
        Sector 5    Sector 6    Sector 7
        (REAR-LEFT) (REAR)      (REAR-RIGHT)
```

Each sensor contributes to its covered sectors:
- **S2 LIDAR**: All 12 sectors (360° coverage)
- **OAK-D Pro Depth**: Sectors 10, 11, 0, 1, 2 (~72° forward FOV)
- **VL53L0X ToF**: Sectors corresponding to mounting positions (FL→1, FR→11, RL→5, RR→7)

The fused distance per sector is the **minimum** across all contributing sensors.

## Navigation Policy

### Virtual Force Field (VFF)
Each obstacle sector exerts a **repulsive force** inversely proportional to distance. A constant **forward driving force** provides the attraction. The resultant vector determines the robot's heading and speed.

### Behavioural State Machine

| State | Trigger | Behaviour |
|-------|---------|-----------|
| `FREE_ROAM` | All sectors > 1.2m | Full speed, random exploration bias |
| `CAREFUL` | Any sector < 0.8m | VFF active steering |
| `NARROW_PASSAGE` | Sides < 0.5m, front clear | Reduced speed, centre between walls |
| `AVOIDING` | Front < 0.35m | Hard turn away from nearest obstacle |
| `DEAD_END_RECOVERY` | Front+left+right < 0.5m | Back up 2s → rotate 2.5s toward open side |
| `EMERGENCY_STOP` | ToF < 0.10m | Hard stop (safety monitor) |

### Dynamic Obstacle Handling
Temporal differencing on sector distances detects approaching objects. Dynamic obstacles receive **1.5x boosted repulsive force** for faster avoidance response.

## Sensor Ablation Study

To compare LIDAR-only vs. LIDAR + depth:

```bash
# Run 1: Full sensors — log to /tmp/reactive_nav_logs/
ros2 launch reactive_nav reactive_nav_full.launch.py

# Run 2: LIDAR only — log to /tmp/reactive_nav_logs/lidar_only/
ros2 launch reactive_nav reactive_nav_lidar_only.launch.py
```

Compare the CSV logs for:
- Collision rate (contacts/min)
- Dynamic obstacle response latency
- State distribution differences

## Configuration

All parameters are in `config/nav_params.yaml`. Key tunable values:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_speed` | 0.3 m/s | Full cruising speed |
| `emergency_stop_distance` | 0.10 m | ToF hard-stop threshold |
| `dead_end_threshold` | 0.5 m | Distance to trigger dead-end recovery |
| `use_depth_camera` | true | Toggle depth camera for ablation |
| `repulsive_gain` | 0.8 | VFF repulsive force strength |
| `influence_distance` | 1.5 m | Max range for VFF obstacle influence |

## Demo Scenarios

The system handles all four required scenarios:

1. **Static obstacles** — VFF steers around chairs, boxes, walls, corners
2. **Narrow passage** — Detects tight corridors, reduces speed, centres between walls
3. **Moving obstacle** — Dynamic detection triggers boosted avoidance response
4. **Dead end** — Detects 3-sided blockage, backs up, rotates toward open space

## File Structure

```
src/reactive_nav/
├── package.xml                              # ROS 2 package manifest
├── setup.py                                 # Python package setup
├── setup.cfg
├── config/
│   └── nav_params.yaml                      # All tunable parameters
├── launch/
│   ├── reactive_nav_full.launch.py          # Full system launch
│   └── reactive_nav_lidar_only.launch.py    # LIDAR-only ablation launch
├── reactive_nav/
│   ├── __init__.py
│   ├── sensor_fusion_node.py                # Sensor fusion (LIDAR+depth+ToF)
│   ├── reactive_navigator_node.py           # VFF + state machine
│   ├── safety_monitor_node.py               # ToF emergency stop gate
│   └── data_logger_node.py                  # CSV logging for evaluation
└── README.md
```
