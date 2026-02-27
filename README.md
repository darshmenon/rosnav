# ROS 2 Navigation and SLAM with Nav2 and Gazebo Harmonic

📖 **Related Blog Post**:
👉 [*Mastering ROS 2 Navigation: From SLAM Mapping to Autonomous Obstacle Avoidance*](https://medium.com/@darshmenon02/mastering-ros-2-navigation-from-slam-mapping-to-autonomous-obstacle-avoidance-7446e4ff049a)

## Overview

Autonomous robot navigation using **Nav2**, **SLAM Toolbox**, and **Gazebo Harmonic**. Includes frontier-based exploration, waypoint following, multi-robot support, and 2D LiDAR. Works on **ROS 2 Humble and Jazzy** — distro is detected automatically at launch.

![alt text](images/nav2spedup-ezgif.com-video-to-gif-converter.gif)
![alt text](images/map_demo.png)

---

## ⚠️ Humble vs Jazzy — Automatic Params Selection

Nav2 plugin naming differs between distros. **The launch files detect `$ROS_DISTRO` automatically** and pick the right config:

| Distro | Params file used | Plugin format |
|---|---|---|
| **Humble** (default) | `config/nav2_params.yaml` | `nav2_behaviors/Spin` |
| **Jazzy** | `config/nav2_params_jazzy.yaml` | `nav2_behaviors::Spin` |

> No manual changes needed — just `source /opt/ros/<distro>/setup.bash` before launching.
> If `$ROS_DISTRO` is missing, launch files fall back to **Humble** params.

---

## Features

- **SLAM live mapping** — SLAM Toolbox builds map while navigating
- **Frontier exploration** — robot autonomously explores unknown areas
- **Nav2 full stack** — planner, controller, recovery, behaviours
- **Multi-robot** — two robots sharing one map
- **Waypoint following** — navigate a sequence of poses
- **Complex worlds** — obstacles world and maze world included

---

## Requirements

| | Humble | Jazzy |
|---|---|---|
| OS | Ubuntu 22.04 | Ubuntu 24.04 |
| Gazebo | Harmonic | Harmonic |

---

## Installation

```bash
# Replace 'humble' with 'jazzy' on Ubuntu 24.04
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-bridge \
  ros-humble-xacro ros-humble-joint-state-publisher \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-teleop-twist-keyboard

mkdir -p ~/rosnav/src && cd ~/rosnav/src
git clone https://github.com/darshmenon/rosnav.git
cd ~/rosnav && colcon build --symlink-install
source ~/rosnav/install/setup.bash
```

---

## Running

### Mode 1 — Saved map + AMCL localisation
```bash
ros2 launch diff_drive_robot robot.launch.py
# If map:= is omitted, launch auto-picks:
#   ~/rosnav/<world_name>_map.yaml (if present), else ~/rosnav/my_map.yaml
# Example:
#   ros2 launch diff_drive_robot robot.launch.py world:=.../maze.world
#   -> tries ~/rosnav/maze_map.yaml first
```

### Mode 2 — SLAM live mapping + Nav2 (no map needed)
```bash
ros2 launch diff_drive_robot slam_nav.launch.py
# Gazebo (maze world) + SLAM Toolbox + Nav2 + RViz — all-in-one
```

### Mode 3 — Autonomous frontier exploration
```bash
# Terminal 1
ros2 launch diff_drive_robot slam_nav.launch.py

# Terminal 2 (after ~10s)
ros2 run diff_drive_robot frontier_explorer.py

# Save map when complete (world-aware names)
ros2 run nav2_map_server map_saver_cli -f ~/rosnav/maze_map
# or
ros2 run nav2_map_server map_saver_cli -f ~/rosnav/obstacles_map
```

### Custom world
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world:=/full/path/to/world.world
# Optional explicit map save prefix:
ros2 launch diff_drive_robot slam_nav.launch.py world:=/full/path/to/world.world map_prefix:=~/rosnav/custom_map
```

---

## Keyboard Control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `FATAL: plugin X does not exist` | Wrong distro params — check `$ROS_DISTRO` is sourced correctly |
| Map not publishing | Check `map` arg path in launch, or config in `mapper_params_online_async.yaml` |
| Robot not moving | Run `ros2 topic hz /cmd_vel` — if 0, Nav2 lifecycle failed; check node list |
| RViz GLSL errors | Cosmetic only, can be ignored |

---

## Contributing

Issues and pull requests welcome.
