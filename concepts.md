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
Run to build a map:
  ros2 launch diff_drive_robot slam.launch.py

Save the map after exploring (name it after the world):
  ros2 run nav2_map_server map_saver_cli -f ~/rosnav/map_maze
  # or: ros2 run nav2_map_server map_saver_cli -f ~/rosnav/map_obstacles
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
# Terminal 1 — SLAM + Nav2 (all-in-one)
ros2 launch diff_drive_robot slam_nav.launch.py

# Terminal 2 — frontier explorer
ros2 run diff_drive_robot frontier_explorer.py
```

Recent reliability fixes in `frontier_explorer.py`:
- Removed SciPy runtime dependency (frontier adjacency + clustering implemented with NumPy + BFS).
- Added configurable `base_frame` and `goal_frame` TF lookup (`map -> base_link` by default).
- Added `min_goal_distance` filter so very near centroids are skipped (prevents no-op goals).
- If TF is not ready yet, the node waits instead of using a fake `(0, 0)` position.

---

## 10. Multi-Robot Navigation + Map Sharing

`launch/multi_robot.launch.py` runs two robots simultaneously:

### Architecture
```
/map  ←── map_server (shared, one instance)
            │
     ┌──────┴──────┐
  robot1/        robot2/
  amcl           amcl
  planner        planner
  controller     controller
```

- Each robot has its own **namespace** (`robot1/`, `robot2/`) so topics don't collide.
- A **single map_server** publishes `/map` — both robots localise against the same map.
- Each robot's AMCL independently estimates its own pose on that shared map.
- Each robot gets its own Nav2 stack with separate costmaps.

```bash
ros2 launch diff_drive_robot multi_robot.launch.py

# Send goal to robot1
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}}}}"

# Send goal to robot2
ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: 2.0}}}}"
```

> **Nav2 Jazzy note:** `use_namespace` was removed — only `namespace` is needed now.
> Topics are now relative paths (`scan` not `/scan`) within each namespace.

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

## 12. Custom Obstacle Avoidance (`navigation.py`)

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
  - `maps/` for `*.yaml`, `*.pgm`, `*.data`, `*.posegraph`
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
| `slam_nav.launch.py` | Gazebo + robot + SLAM Toolbox + Nav2 + RViz — **no map needed** | `world`, `map_prefix`, `rviz`, `robot_name`, `spawn_x/y/z/yaw` |
| `slam.launch.py` | Gazebo + robot + SLAM Toolbox mapping mode | `use_sim_time` |
| `multi_robot.launch.py` | Two robots + shared map server + Nav2 per robot | `map`, `world`, `rviz` |
| `nav2.launch.py` | Nav2 only (attach to running Gazebo) | `map`, `world`, `use_sim_time` |

## Quick Reference — Scripts

| Script | What it does | Key ROS params |
|---|---|---|
| `navigation.py` | Custom obstacle-avoidance FSM (no Nav2 needed) | `goal_x`, `goal_y`, `base_speed`, `obstacle_threshold` |
| `path_planning.py` | Standalone A* path planner | `grid_size_x/y`, `resolution`, `safety_margin` |
| `waypoint_nav.py` | Navigate through a sequence of waypoints via Nav2 | `waypoints_file`, `frame_id` |
| `frontier_explorer.py` | Autonomous map exploration via frontier detection | `min_frontier_size`, `revisit_radius`, `poll_period` |
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
- If `map:=` is empty, launch tries `~/rosnav/<world_name>_map.yaml` first.
- If that file does not exist, it falls back to `~/rosnav/my_map.yaml`.

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
ros2 run nav2_map_server map_saver_cli -f ~/rosnav/maze_map
ros2 run nav2_map_server map_saver_cli -f ~/rosnav/obstacles_map

# Load custom waypoints
ros2 run diff_drive_robot waypoint_nav.py --ros-args \
    -p waypoints_file:=~/rosnav/waypoints.yaml

# Run frontier exploration (slam_nav.launch.py must be active)
ros2 run diff_drive_robot frontier_explorer.py

# Reset robot to origin
ros2 run diff_drive_robot reset_pose.py --ros-args \
    -p world_name:=obstacles -p robot_name:=diff_bot
```
