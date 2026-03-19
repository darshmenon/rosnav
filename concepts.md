# ROS 2 Navigation Concepts

A reference guide covering every concept used in this project.

---

## 1. ROS 2 Basics

### Nodes
A **node** is a single executable process in ROS 2. Each node does one thing (e.g., read LiDAR, compute path, drive motors). Nodes communicate via topics, services, and actions.

### Topics
**Topics** are named data channels. A node **publishes** data; other nodes **subscribe** to it.
- `/scan` — LiDAR distance readings
- `/odom` — robot wheel odometry
- `/cmd_vel` — velocity commands sent to the robot

### TF (Transform Tree)
TF tracks the **position of every frame** (coordinate system) relative to every other:
- `map → odom → base_link → laser_frame`
- Lets Nav2 know where the robot is in the world at any moment.

### Actions
**Actions** are long-running tasks with feedback (e.g., "navigate to pose").
Nav2 exposes `navigate_to_pose` and `follow_waypoints` as action servers.

---

## 2. Gazebo Harmonic

A physics simulator that models the robot's body, wheels, sensors, and environment.
The **gz_bridge** translates Gazebo topics to ROS 2 topics and back:
- `/scan` (Gazebo) → `/scan` (ROS 2 LaserScan)
- `/cmd_vel` (ROS 2 Twist) → `/cmd_vel` (Gazebo motors)
- `/points` (Gazebo) → `/points` (ROS 2 PointCloud2) — when using 3D LiDAR

---

## 3. SLAM (Simultaneous Localisation and Mapping)

**SLAM Toolbox** builds a 2D occupancy grid map while simultaneously tracking where the robot is inside it, using only LiDAR data.

- **Mapping mode** (`mode: mapping`) — build a new map from scratch.
- **Localization mode** (`mode: localization`) — load a saved map and locate the robot within it (no new map built).

The output is a `/map` topic (OccupancyGrid) used by Nav2.

```
Run to build the map:
  ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze

Run to auto-explore and progressively save the map:
  ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true

Maps will automatically be saved to `src/diff_drive_robot-main/maps/map_maze` or `map_obstacles`.
```

---

## 4. Nav2 Stack

Nav2 is the ROS 2 navigation framework. It is a collection of nodes managed by **lifecycle managers**.

### Components

| Component | Role |
|---|---|
| **map_server** | Loads a saved map yaml and publishes it on `/map` |
| **AMCL** | Particle-filter localisation — figures out where the robot is on the loaded map using LiDAR |
| **planner_server** | Global path planner (NavFn/A*) — finds a route from start to goal |
| **controller_server** | Local controller (MPPI) — follows the global path while avoiding nearby obstacles |
| **behavior_server** | Recovery behaviours — spin, backup, wait when the robot gets stuck |
| **bt_navigator** | Behaviour Tree — orchestrates all the above components for a navigation goal |
| **velocity_smoother** | Smooths velocity commands to prevent jerky motion |
| **collision_monitor** | Emergency brake if an obstacle enters the safety zone |

### Launch Modes

- **`bringup_launch.py`** — full stack (map_server + AMCL + navigation). Use for autonomous navigation with a saved map.
- **`navigation_launch.py`** — navigation stack only (no map_server). Use when SLAM is running separately.

---

## 5. Nav2 Plugin Naming — Humble vs Jazzy

> **This is a common gotcha when switching between ROS distros.**

Nav2 plugin names changed format between Humble and Jazzy:

| Plugin | Humble | Jazzy |
|---|---|---|
| Behaviors (Spin, BackUp…) | `nav2_behaviors/Spin` | `nav2_behaviors::Spin` |
| NavFn planner | `nav2_navfn_planner/NavfnPlanner` | `nav2_navfn_planner::NavfnPlanner` |
| Costmap layers | `nav2_costmap_2d::StaticLayer` | `nav2_costmap_2d::StaticLayer` |
| MPPI controller | `nav2_mppi_controller::MPPIController` | `nav2_mppi_controller::MPPIController` |

This project ships **two config files** and auto-selects at launch via `$ROS_DISTRO`:
- `config/nav2_params.yaml` — Humble (default)
- `config/nav2_params_jazzy.yaml` — Jazzy

The launch files contain:
```python
_NAV2_PARAMS = 'nav2_params_jazzy.yaml' if ROS_DISTRO == 'jazzy' else 'nav2_params.yaml'
```

---

## 6. Costmaps

Costmaps are grids that encode how dangerous each cell is.

- **Global costmap** — full map, used by the path planner. Uses the `/map` static layer + obstacle layer (live LiDAR).
- **Local costmap** — small rolling window around the robot, used by the controller to avoid close-range obstacles.

**Inflation layer** — expands obstacle cells outward by `inflation_radius` so the robot steers away from walls.

---

## 7. MPPI Controller

**Model Predictive Path Integral** — the local controller used in this project.
It samples thousands of random velocity trajectories in parallel, scores them against a cost function (stay on path, avoid obstacles, prefer forward motion), and executes the lowest-cost one.

Configured under `controller_server → FollowPath` in `nav2_params.yaml`.

---

## 8. Waypoint Following

`scripts/waypoint_nav.py` uses Nav2's **FollowWaypoints** action:
1. You define a list of (x, y, yaw) poses.
2. The node sends all poses at once to the action server.
3. Nav2 navigates to each in sequence, reporting feedback after each one.

```bash
ros2 run diff_drive_robot waypoint_nav.py
```

Edit `WAYPOINTS` at the top of the script to change the route.

---

## 9. Frontier Exploration

`scripts/frontier_explorer.py` implements autonomous map exploration:

1. Subscribe to `/map` (OccupancyGrid from SLAM).
2. Find **frontier cells** — free cells (value 0) adjacent to unknown cells (value -1).
3. Cluster frontiers using in-node connected-component labelling (no SciPy required).
4. Pick the nearest cluster centroid as the next navigation goal (uses TF robot pose).
5. Send the goal to Nav2 via `NavigateToPose`.
6. Repeat until no frontiers remain.

```bash
# Single command — SLAM + Nav2 + frontier explorer + auto-save
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true
```

Recent reliability fixes in `frontier_explorer.py`:
- Removed SciPy runtime dependency (frontier adjacency + clustering implemented with NumPy + BFS).
- Added configurable `base_frame` and `goal_frame` TF lookup (`map -> base_link` by default).
- Added `min_goal_distance` filter so very near centroids are skipped (prevents no-op goals).
- If TF is not ready yet, the node waits instead of using a fake `(0, 0)` position.
- Added `map_save_path` parameter so completed exploration auto-saves map via `map_saver_cli`.

---

## 10. Multi-Robot Navigation + Map Sharing

`launch/multi_robot.launch.py` runs N robots simultaneously with a **scalable** design.

### Architecture — SLAM + Frontier mode (`explore:=true`, default)
```
SLAM Toolbox ─── /robot1/scan ──► /map (shared, progressive)
                                     │
                  ┌──────────────────┴────────────────┐
               robot1/                             robot2/
               (SLAM provides map→odom TF)         amcl (localises on /map)
               planner / controller / bt_nav       planner / controller / bt_nav
               frontier_explorer                   frontier_explorer
```

### Architecture — Pre-built map mode (`explore:=false`)
```
map_server ──► /map (shared, static)
                │
     ┌──────────┴──────────┐
  robot1/               robot2/
  amcl                  amcl
  planner               planner
  controller            controller
```

### Scalability — adding more robots
The fleet is driven by the **ROBOTS list** at the top of `multi_robot.launch.py`.
Nav2 params use a single **template file** (`nav2_multirobot_params.yaml`) — the
placeholder `ROBOT_NS` is substituted at launch time. No per-robot YAML files needed.

```python
# multi_robot.launch.py — add one line per robot
ROBOTS = [
    {'name': 'robot1', 'x': '-2.0', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
    {'name': 'robot2', 'x': '-0.8', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
    {'name': 'robot3', 'x':  '0.5', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
    # ... up to N robots
]
```

### TF frame naming (critical for multi-robot)
Each robot uses `frame_prefix: <ns>/` in its RSP, so TF frames are unique:
- `robot1/base_link`, `robot1/odom`, `robot1/laser_frame`
- `robot2/base_link`, `robot2/odom`, `robot2/laser_frame`

Nav2 params (`amcl.base_frame_id`, `bt_navigator.robot_base_frame`, etc.) must
match these prefixed names. The template file handles this automatically.

```bash
# SLAM + frontier exploration in maze (default)
ros2 launch diff_drive_robot multi_robot.launch.py

# Pre-built map mode
ros2 launch diff_drive_robot multi_robot.launch.py explore:=false

# Different world
ros2 launch diff_drive_robot multi_robot.launch.py world:=obstacles explore:=false

# Send goal to robot1
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}}}}"

# Send goal to robot2
ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: 2.0}}}}"
```

### Key files
| File | Role |
|---|---|
| `launch/multi_robot.launch.py` | Main launch — edit ROBOTS list to scale fleet |
| `config/nav2_multirobot_params.yaml` | Template params (ROBOT_NS placeholder) |
| `config/mapper_params_multirobot.yaml` | SLAM params for robot1 in explore mode |

### Bugs fixed
- `rsp.launch.py` now declares and passes `frame_prefix` to RSP → TF frames are correctly namespaced per robot (was causing robot not visible in Gazebo)
- Costmap local layer changed from `voxel_layer` (3D only) to `obstacle_layer` for 2D LaserScan
- AMCL per-robot added (was missing from `navigation_launch.py` which does not include AMCL)
- All frame IDs prefixed with robot namespace (was causing TF conflicts between robots)

---

## 11. 2D vs 3D LiDAR

| | 2D LiDAR (`lidar.xacro`) | 3D LiDAR (`lidar3d.xacro`) |
|---|---|---|
| Channels | 1 horizontal ring | 16 vertical channels |
| Output topic | `/scan` (LaserScan) | `/points` (PointCloud2) |
| Nav2 compatibility | Native | Needs `pointcloud_to_laserscan` node |
| Typical use | Navigation, SLAM | Object detection, 3D mapping |

To enable 3D LiDAR, edit `robot.urdf.xacro`:
```xml
<!-- Replace -->
<xacro:include filename="lidar.xacro" />
<!-- With -->
<xacro:include filename="lidar3d.xacro" />
```

Then for Nav2 to work you need to convert PointCloud2 → LaserScan:
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r cloud_in:=/points -r scan:=/scan
```

---

## 12. Adding a Camera Sensor (RGB / Depth)

Cameras are added as a URDF xacro file, similar to `lidar.xacro`.

### RGB camera (`camera.xacro`) — example snippet
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <topic>image_raw</topic>
    <gz_frame_id>camera_link</gz_frame_id>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip><near>0.1</near><far>100</far></clip>
    </camera>
  </sensor>
</gazebo>
```

Include it in `robot.urdf.xacro`:
```xml
<xacro:include filename="camera.xacro" />
```

Bridge to ROS 2 (add to `gz_bridge.yaml`):
```yaml
- ros_topic_name: "/image_raw"
  gz_topic_name: "/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
- ros_topic_name: "/camera_info"
  gz_topic_name: "/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
```

### Depth camera (RGBD)
Use `sensor type="depth_camera"` in Gazebo and bridge `sensor_msgs/msg/PointCloud2` or `sensor_msgs/msg/Image` (depth). Can replace or supplement 2D LiDAR for richer obstacle data.

### Mobile robot / arched camera mount
Mount the camera link at any offset from `chassis`:
```xml
<joint name="camera_joint" type="fixed">
  <parent link="chassis"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0 0.20" rpy="0 0 0"/>  <!-- front, raised -->
</joint>
```

---

## 13. New Tool Scripts

### `fleet_manager.py` — Product-like fleet CLI
```bash
ros2 run diff_drive_robot fleet_manager.py list          # list active robots
ros2 run diff_drive_robot fleet_manager.py status        # map/SLAM/Nav2 status
ros2 run diff_drive_robot fleet_manager.py add robot3 1.0 2.0   # dynamic spawn
ros2 run diff_drive_robot fleet_manager.py teleop robot1 # keyboard control
ros2 run diff_drive_robot fleet_manager.py goto robot2 3.0 -1.0 # nav goal
ros2 run diff_drive_robot fleet_manager.py explore robot2        # frontier
ros2 run diff_drive_robot fleet_manager.py savemap /tmp/my_map   # save SLAM map
ros2 run diff_drive_robot fleet_manager.py stop robot1           # cancel nav
```

### `multi_teleop.py` — Interactive multi-robot keyboard teleop
```bash
ros2 run diff_drive_robot multi_teleop.py
# → shows robot list, select one, drive it, switch with R, spawn new with N
```

---

## 14. Custom Obstacle Avoidance (`navigation.py`)

A simpler alternative to Nav2 — a 4-state finite state machine:

```
GOAL_SEEK ──obstacle──► FIND_CLEAR ──aligned──► MOVE_CLEAR ──moved──► REALIGN
    ▲                                                                      │
    └──────────────────────────────────────────────────────────────────────┘
```

Uses only `/scan` (LiDAR) and `/odom`, no map required. Good for unstructured environments but does not do global path planning.

---

## 13. Repository Organization (Recommended)

For this repo, a cleaner layout helps debugging and repeatability:

- Keep ROS package source under `src/diff_drive_robot-main/` only.
- Move generated/runtime artifacts out of root into:
  - `src/diff_drive_robot-main/maps/` for generated `map_*.yaml` and `map_*.pgm`
  - `logs/` for launch or benchmark logs
- Add `scripts/` at repo root only for workflow wrappers (build/test/run), not ROS nodes.
- Keep docs in `docs/` (`README.md` for quickstart, `concepts.md` for deep reference).
- Add `Makefile` or `justfile` commands for common flows (`build`, `slam-nav`, `frontier`, `save-map`).


## 14. A* Path Planner (`path_planning.py`)

Standalone global planner using the **A\* algorithm**:
- Converts the world to a discrete grid.
- Uses Euclidean distance as the heuristic.
- Supports 8-direction movement (including diagonals).
- Currently uses hardcoded obstacles — future: subscribe to `/map` OccupancyGrid.

---

## Quick Reference — Launch Files

| Launch file | What it does | Key args |
|---|---|---|
| `robot.launch.py` | Gazebo + robot + Nav2 full bringup with saved map | `map`, `world`, `robot_name`, `spawn_x/y/z/yaw`, `rviz`, `use_sim_time` |
| `slam_nav.launch.py` | Gazebo + robot + SLAM Toolbox + Nav2 (+ optional auto frontier) | `world_name`, `world`, `explore`, `map_prefix`, `rviz`, `robot_name`, `spawn_x/y/z/yaw` |
| `slam.launch.py` | Gazebo + robot + SLAM Toolbox mapping mode | `use_sim_time` |
| `multi_robot.launch.py` | Two robots + shared map server + Nav2 per robot | `map`, `world`, `rviz` |
| `nav2.launch.py` | Nav2 only (attach to running Gazebo) | `map`, `world`, `use_sim_time` |

## Quick Reference — Scripts

| Script | What it does | Key ROS params |
|---|---|---|
| `navigation.py` | Custom obstacle-avoidance FSM (no Nav2 needed) | `goal_x`, `goal_y`, `base_speed`, `obstacle_threshold` |
| `path_planning.py` | Standalone A* path planner | `grid_size_x/y`, `resolution`, `safety_margin` |
| `waypoint_nav.py` | Navigate through a sequence of waypoints via Nav2 | `waypoints_file`, `frame_id` |
| `frontier_explorer.py` | Autonomous map exploration via frontier detection + optional auto-save | `min_frontier_size`, `revisit_radius`, `poll_period`, `map_save_path` |
| `check_odometry.py` | Debug odometry data | — |
| `reset_pose.py` | Reset robot pose in simulation | `world_name`, `robot_name`, `reset_x/y/z/yaw` |

---

## How to Source and Run

```bash
# Every new terminal needs this
source /opt/ros/humble/setup.bash           # or jazzy
source ~/rosnav/install/setup.bash

# Build after any changes
cd ~/rosnav
colcon build --symlink-install --packages-select diff_drive_robot
```

Map selection behavior:
- If `map:=` is provided, that exact map is used.
- If `map:=` is empty, launch tries `<package_share>/maps/map_<world_name>.yaml` first.
- Legacy fallbacks still work (`~/rosnav/maps` and old root-level map files).

---

## RViz Checklist — What to Verify

After launching any launch file, open RViz and add these displays:

| Display | Topic | What it confirms |
|---|---|---|
| Map | `/map` | Map is loaded and being published |
| RobotModel | — | URDF loaded, TF tree working |
| LaserScan | `/scan` | LiDAR data flowing from Gazebo |
| Pose | `/amcl_pose` | AMCL is localising the robot (only in `robot.launch.py`) |
| Path | `/plan` | Nav2 planner computed a path |
| MarkerArray | `/local_costmap/costmap` | Local obstacle avoidance active |

Set **Fixed Frame** to `map` in RViz Global Options.

---

## Checking SLAM / Localization from Terminal

```bash
# Is the map being published?
ros2 topic echo /map --once | head -5

# Is AMCL running and localising?
ros2 topic echo /amcl_pose

# Is the TF tree complete? (map → odom → base_link → laser_frame)
ros2 run tf2_tools view_frames   # saves frames.pdf

# Is Nav2 active?
ros2 node list | grep -E "amcl|planner|controller|bt_navigator"

# Is the robot moving? (should show non-zero during navigation)
ros2 topic hz /cmd_vel

# Save map after SLAM mapping (world-aware naming)
ros2 run nav2_map_server map_saver_cli -f src/diff_drive_robot-main/maps/map_maze
ros2 run nav2_map_server map_saver_cli -f src/diff_drive_robot-main/maps/map_obstacles

# Load custom waypoints
ros2 run diff_drive_robot waypoint_nav.py --ros-args \
    -p waypoints_file:=~/rosnav/waypoints.yaml

# Run frontier exploration (slam_nav.launch.py must be active)
ros2 run diff_drive_robot frontier_explorer.py

# Reset robot to origin
ros2 run diff_drive_robot reset_pose.py --ros-args \
    -p world_name:=obstacles -p robot_name:=diff_drive
```

---

## 16. 3-Tier Autonomy Stack

The full stack is structured as three independent layers:

```
┌─────────────────────────────────────────┐
│  Mission Layer  — mission_server.py     │  High-level goals (patrol, goto, sequence)
│                                         │  Breaks goals into NavigateToPose calls
├─────────────────────────────────────────┤
│  Navigation Layer  — Nav2 BT + MPPI     │  Path planning, local control, recovery
│                                         │  Costmaps, planner, controller, BT
├─────────────────────────────────────────┤
│  Safety Layer  — collision_monitor.py   │  Independent scan watchdog
│                                         │  Overrides cmd_vel on obstacle detection
└─────────────────────────────────────────┘
```

Each layer is independent — the safety layer can stop the robot regardless of what the navigation or mission layer is doing.

---

## 17. Collision Monitor

`scripts/collision_monitor.py` is a standalone safety watchdog.

**How it works:**
1. Subscribes to `/scan` (LaserScan).
2. Checks the minimum range in a configurable forward FOV (default ±30°).
3. When `min_range < stop_distance` (default 0.30 m): publishes `Twist(0,0)` to `/cmd_vel` at 20 Hz, overriding Nav2.
4. When `min_range < slowdown_distance` (default 0.70 m): state = SLOWDOWN (relay mode only).
5. Publishes JSON state to `/collision_monitor/state`.

**Modes:**
- `watchdog` (default): publishes zero-vel override during STOP. Simple — no pipeline changes needed.
- `relay`: subscribes to `cmd_vel_nav`, scales or zeroes, publishes to `cmd_vel`. Requires controller remapping.

**Parameters:**

| Parameter | Default | Meaning |
|---|---|---|
| `robot_ns` | `''` | Namespace prefix (e.g. `robot1`) |
| `stop_distance` | `0.30` | m — publish zero vel |
| `slowdown_distance` | `0.70` | m — scale vel (relay mode) |
| `slowdown_factor` | `0.40` | Scale factor in slowdown zone |
| `front_angle_deg` | `60` | Total forward FOV to monitor |
| `watch_all_around` | `false` | Use 360° instead of forward FOV |
| `relay_mode` | `false` | Enable relay pipeline |

```bash
# Launch with slam_nav (safety:=true is default):
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze safety:=true

# Or run standalone:
ros2 run diff_drive_robot collision_monitor.py --ros-args \
    -p stop_distance:=0.35 -p watch_all_around:=true

# Monitor state:
ros2 topic echo /collision_monitor/state
```

---

## 18. Mission Server

`scripts/mission_server.py` is the top-level mission execution daemon.

**How it works:**
1. Runs as a persistent ROS 2 node.
2. Subscribes to `/mission/execute` (std_msgs/String JSON).
3. On receipt of a mission, sends `NavigateToPose` goals to the target robot's Nav2 stack.
4. Publishes current state to `/mission/state` (std_msgs/String JSON) at 1 Hz.

**Mission types:**

| Type | Behaviour |
|---|---|
| `patrol` | Loop through waypoints indefinitely |
| `sequence` | Visit waypoints once in order, then DONE |
| `goto` | Navigate to a single pose, then DONE |

**State machine:** `IDLE → NAVIGATING → DONE / FAILED`
Cancel with `action: cancel` → returns to `IDLE`.

```bash
# Start daemon:
ros2 run diff_drive_robot mission_server.py

# Patrol robot1 through 3 waypoints:
ros2 run diff_drive_robot mission_server.py patrol robot1 1,2,0 3,4,90 0,0,180

# Single goal:
ros2 run diff_drive_robot mission_server.py goto robot1 3.0 -1.0 45

# Check state:
ros2 run diff_drive_robot mission_server.py status

# Cancel:
ros2 run diff_drive_robot mission_server.py cancel

# Via fleet_manager:
ros2 run diff_drive_robot fleet_manager.py mission robot1 patrol 1,2,0 3,4,90
ros2 run diff_drive_robot fleet_manager.py mission robot1 status
ros2 run diff_drive_robot fleet_manager.py collision robot1
```
