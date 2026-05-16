<div align="center">

# ROS 2 Autonomous Navigation Stack

### SLAM · Nav2 · Multi-Robot · Frontier Exploration · Fleet Management

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)
[![Blog](https://img.shields.io/badge/Blog-Medium-black?logo=medium)](https://medium.com/@darshmenon02/mastering-ros-2-navigation-from-slam-mapping-to-autonomous-obstacle-avoidance-7446e4ff049a)

![nav2 demo](images/nav2spedup-ezgif.com-video-to-gif-converter.gif)

</div>

---

## What is this?

A full autonomous robot navigation stack built on **Nav2**, **SLAM Toolbox**, and **Gazebo Harmonic**. Single command launches everything — Gazebo, SLAM, Nav2, RViz, frontier exploration. Scale from one robot to a fleet by editing a single list.

**Distro detection is automatic.** Source your ROS install and launch — no config changes needed between Humble and Jazzy.

---

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [All Launch Modes](#all-launch-modes)
- [Fleet Management](#fleet-management)
- [Worlds](#worlds)
- [Troubleshooting](#troubleshooting)

---

## Features

<table>
<tr>
<td width="50%">

**Core Navigation**
- SLAM Toolbox live mapping
- Nav2 full stack (MPPI controller, Smac planner, recovery BT)
- Frontier-based autonomous exploration
- Waypoint following
- Custom Behavior Tree (backup→spin→clear→wait)

</td>
<td width="50%">

**Multi-Robot Fleet**
- N robots, one SLAM-built shared map
- Coordinated frontier assignment — no duplicate effort
- Namespaced TF per robot (`robot1/odom`, `robot1/base_link`)
- Headless mode for SSH / CI
- Hungarian task allocation across idle robots

</td>
</tr>
<tr>
<td>

**Safety Stack**
- Collision Monitor — stop/slowdown zones from live scan
- Priority collision avoidance between robots
- Deadlock detection and automatic recovery
- Dynamic obstacle tracker with MarkerArray output

</td>
<td>

**Tooling**
- Fleet CLI (`fleet_manager.py`) — list, teleop, goto, savemap, health
- Fleet GUI (Tkinter) — click-to-navigate, velocity sliders
- Multi-robot keyboard teleop with robot switcher
- Coverage path planner (boustrophedon sweep)
- Fleet health monitor at 1 Hz on `/fleet/health`

</td>
</tr>
</table>

---

## Requirements

| | Humble | Jazzy |
|---|---|---|
| OS | Ubuntu 22.04 | Ubuntu 24.04 |
| Gazebo | Harmonic | Harmonic |

> Nav2 plugin syntax differs between distros. Launch files detect `$ROS_DISTRO` automatically and pick the right params — no manual changes needed.

---

## Installation

```bash
# Replace humble with jazzy on Ubuntu 24.04
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-bridge \
  ros-humble-xacro ros-humble-joint-state-publisher \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-teleop-twist-keyboard

mkdir -p ~/rosnav/src && cd ~/rosnav/src
git clone https://github.com/darshmenon/rosnav.git
cd ~/rosnav
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start

```bash
# Explore the maze — SLAM + Nav2 + frontier explorer in one command
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true

# Multi-robot fleet (2 robots, coordinated exploration)
ros2 launch diff_drive_robot multi_robot.launch.py

# Keyboard control (any terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Maps auto-save to `src/diff_drive_robot-main/maps/map_<world>.yaml` every 15 s during exploration.

---

## All Launch Modes

### Single Robot

#### Mode 1 — Autonomous SLAM + Frontier Exploration
Gazebo + SLAM + Nav2 + RViz + frontier explorer. Robot maps the world on its own.
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true
```

#### Mode 2 — Manual SLAM
Drive the robot yourself to build the map.
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze
# Run frontier explorer later if needed:
ros2 run diff_drive_robot frontier_explorer.py
```

#### Mode 3 — Pre-built Map + AMCL Localisation
Load a saved map and navigate in localisation-only mode.
```bash
ros2 launch diff_drive_robot robot.launch.py world:=/full/path/to/maze.world
# Force a specific map:
ros2 launch diff_drive_robot robot.launch.py map:=/full/path/to/my_map.yaml
```

#### Mode 4 — Coverage Sweep
After mapping — boustrophedon lawnmower sweep over the full free space.
```bash
ros2 run diff_drive_robot coverage_planner.py
# Tighter rows for warehouse:
ros2 run diff_drive_robot coverage_planner.py --ros-args -p sweep_spacing:=0.4
```

#### Mode 5 — 3-Tier Autonomy (Mission + Nav + Safety)
```
Mission Layer  ←  mission_server.py   patrol / sequence / goto
Nav Layer      ←  Nav2 BT + MPPI      path planning + control
Safety Layer   ←  collision_monitor   stop / slowdown from scan
```
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze safety:=true

# Separate terminal — start mission server
ros2 run diff_drive_robot mission_server.py

# Send missions
ros2 run diff_drive_robot mission_server.py patrol robot1 1,2,0 3,4,90 0,0,180
ros2 run diff_drive_robot mission_server.py goto robot1 3.0 -1.0 45
ros2 run diff_drive_robot mission_server.py status
ros2 run diff_drive_robot mission_server.py cancel
```

#### Mode 6 — LLM Voice Navigation
Speak or type plain-English commands; Whisper transcribes, ollama parses, Nav2 executes.

```
Mic → Whisper STT → ollama LLM → NavigateToPose → Nav2
```
```bash
# Start nav stack first
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze

# Separate terminal — start LLM navigator
ros2 run diff_drive_robot llm_nav.py

# Press Enter to speak, or type directly:
# > go to room_b
# > go to 2.5 1.0
# > stop

# Text-only (no mic):
ros2 topic pub /llm_nav/command std_msgs/msg/String "data: 'go to room_a'" --once
```

Named locations are defined in `config/locations.yaml` (origin, room_a–c, hallway, charging_dock).
The node retries for up to 60 s if Nav2 is still starting up.

**Override defaults:**
```bash
ros2 run diff_drive_robot llm_nav.py --ros-args \
    -p whisper_model:=small \
    -p ollama_model:=llama2 \
    -p record_seconds:=6.0
```

**Requirements:** `ollama serve` running with a model pulled (`ollama pull llama2`).

---

### Multi-Robot

![Multi-robot navigation and exploration](images/multi_robot_navigation_and_exploration.gif)

`robot1` runs SLAM and shares the map. All other robots use AMCL on that map. A single `frontier_coordinator` assigns unique frontiers — no two robots explore the same area.

```bash
# SLAM + coordinated exploration (default)
ros2 launch diff_drive_robot multi_robot.launch.py

# Pre-built map mode
ros2 launch diff_drive_robot multi_robot.launch.py explore:=false

# Different world
ros2 launch diff_drive_robot multi_robot.launch.py world:=warehouse

# Headless (SSH / CI)
ros2 launch diff_drive_robot multi_robot.launch.py headless:=true

# Full fleet management stack
ros2 launch diff_drive_robot multi_robot.launch.py fleet_mgmt:=true
```

#### Adding robots
Edit only the `ROBOTS` list in `multi_robot.launch.py`:
```python
ROBOTS = [
    {'name': 'robot1', 'x': '-2.0', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
    {'name': 'robot2', 'x': '-0.8', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
    {'name': 'robot3', 'x':  '0.5', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
]
```
Everything else — TF, Nav2 params, frontier coordinator — picks up the new robot automatically.

#### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `world` | `maze` | World name or full `.world` path |
| `explore` | `true` | `true` = SLAM + frontier; `false` = pre-built map + AMCL |
| `headless` | `false` | No Gazebo GUI or RViz |
| `fleet_mgmt` | `false` | Start mission server, task allocator, health monitor, collision avoidance, deadlock recovery |
| `rviz` | `true` | Skip RViz |
| `map` | *(auto)* | Path to map YAML — only used when `explore:=false` |

#### Shared map building

![Multi-robot mapping](images/multi_robot_mapping.png)

#### Verify multi-robot
```bash
# Topics per robot
ros2 topic list | grep -E "/robot1|/robot2"

# Send goals
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.5, y: -0.5}, orientation: {w: 1.0}}}}"

ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: -0.5}, orientation: {w: 1.0}}}}"

# Watch odom
ros2 topic echo /robot1/odom --once
```

---

## Fleet Management

### CLI

```bash
ros2 run diff_drive_robot fleet_manager.py list               # list active robots
ros2 run diff_drive_robot fleet_manager.py status             # SLAM / Nav2 / map state
ros2 run diff_drive_robot fleet_manager.py add robot3 1.0 2.0 # spawn robot at (1,2)
ros2 run diff_drive_robot fleet_manager.py teleop robot1      # keyboard drive
ros2 run diff_drive_robot fleet_manager.py goto robot2 3.0 -1.0
ros2 run diff_drive_robot fleet_manager.py explore robot2
ros2 run diff_drive_robot fleet_manager.py savemap src/diff_drive_robot-main/maps/map_maze
ros2 run diff_drive_robot fleet_manager.py health             # per-robot health report

# Missions (mission_server must be running)
ros2 run diff_drive_robot fleet_manager.py mission robot1 patrol 1,2,0 3,4,90 0,0,180
ros2 run diff_drive_robot fleet_manager.py mission robot1 status
ros2 run diff_drive_robot fleet_manager.py mission robot1 cancel

# Task queue
ros2 run diff_drive_robot fleet_manager.py tasks add 2.0 1.5 0 pickup_A
ros2 run diff_drive_robot fleet_manager.py tasks add 4.0 -1.0 90 dock_B
ros2 run diff_drive_robot fleet_manager.py tasks status
ros2 run diff_drive_robot fleet_manager.py tasks clear
```

### GUI
```bash
ros2 run diff_drive_robot fleet_gui.py
```
Click on the map to send goals, use sliders for teleop, spawn robots, save the SLAM map — all in one window.

### Multi-robot teleop
```bash
ros2 run diff_drive_robot multi_teleop.py
# WASD to drive, R to switch robot, N to spawn new
```

### Dynamic obstacle tracker
```bash
ros2 run diff_drive_robot obstacle_tracker.py
# Visualise in RViz: MarkerArray on /obstacle_tracker/markers
ros2 topic echo /obstacle_tracker/state
```

### Fleet health monitor
```bash
ros2 run diff_drive_robot fleet_health.py
ros2 topic echo /fleet/health
```
Tracks odom/scan Hz, Nav2 node presence, collision state, and mission state per robot. Reports `OK` / `WARN` / `ERROR` at 1 Hz.

### Task allocator
```bash
ros2 run diff_drive_robot task_allocator.py  # or fleet_mgmt:=true in multi_robot
ros2 run diff_drive_robot fleet_manager.py tasks add 2.0 1.5 0 pickup_A
```
Hungarian assignment across idle robots. Pure-Python — no scipy needed.

---

## 3D LiDAR (optional)

The URDF supports both 2D and 3D LiDAR. Default is 2D (`lidar.xacro`).

To switch to 3D:
1. Edit `urdf/robot.urdf.xacro` — replace `lidar.xacro` with `lidar3d.xacro`
2. Convert PointCloud2 → LaserScan for Nav2:

```bash
sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args -r cloud_in:=/points -r scan:=/scan \
    -p min_height:=0.1 -p max_height:=1.0 \
    -p angle_min:=-1.57 -p angle_max:=1.57
```

---

## Worlds

| World | Size | Description |
|---|---|---|
| `maze` | — | Enclosed maze, ideal for exploration |
| `obstacles` | — | Open field with barrel obstacles |
| `warehouse` | 24×20 m | 5 shelf rows, loading dock, staging zone, pillars, pallet stacks |
| `house` | 16×12 m | Living room, kitchen, hallway, 2 bedrooms, bathroom, furniture |
| `corridor` | — | Narrow corridor with branching rooms |

All worlds use SDF primitives only — no external model downloads, instant load.

```bash
# Single robot, any world
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=warehouse explore:=true

# Multi-robot, any world
ros2 launch diff_drive_robot multi_robot.launch.py world:=warehouse
ros2 launch diff_drive_robot multi_robot.launch.py world:=house
ros2 launch diff_drive_robot multi_robot.launch.py world:=corridor explore:=false
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `FATAL: plugin X does not exist` | Check `$ROS_DISTRO` is sourced correctly — wrong distro params loaded |
| `SmacPlannerHybrid` not found | `sudo apt install ros-$ROS_DISTRO-nav2-smac-planner` |
| Map not saving | Confirm `explore:=true`; maps write to `src/diff_drive_robot-main/maps/` |
| `No frontiers` in explorer logs | Check for `TF_OLD_DATA` / dropped scans; kill stale Gazebo/ROS processes |
| Robot not moving | `ros2 topic hz /cmd_vel` — if 0, Nav2 lifecycle failed; check node list |
| Robots not visible in Gazebo | Rebuild: `colcon build --symlink-install` then `source install/setup.bash` |
| `goal rejected` immediately | Nav2 still starting — coordinator retries every 2 s automatically |
| All robots go to same area | Old per-robot `frontier_explorer` nodes running — kill them; only `frontier_coordinator` should run |
| Multi-robot TF errors | Run `ros2 run tf2_tools view_frames` to inspect the tree; confirm `rsp.launch.py` frame_prefix fix is applied |
| RViz GLSL errors | Cosmetic — safe to ignore |

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Fleet Management                    │
│  mission_server · task_allocator · fleet_health      │
│  priority_collision_avoidance · deadlock_recovery    │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│                  Per-Robot Stack                     │
│  Nav2 (MPPI + Smac + BT)  ·  AMCL / SLAM Toolbox    │
│  velocity_smoother  ·  collision_monitor             │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│                   Simulation                         │
│          Gazebo Harmonic  ·  ros-gz-bridge           │
│          LaserScan  ·  Odometry  ·  TF               │
└─────────────────────────────────────────────────────┘
```

---

<div align="center">

Made by [@darshmenon](https://github.com/darshmenon) · [Blog post](https://medium.com/@darshmenon02/mastering-ros-2-navigation-from-slam-mapping-to-autonomous-obstacle-avoidance-7446e4ff049a)

</div>
