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
  ros2 launch diff_drive_robot slam_nav.launch.py world_name:=hospital

Run to auto-explore and progressively save the map:
  ros2 launch diff_drive_robot slam_nav.launch.py world_name:=hospital explore:=true

Maps will automatically be saved to `src/diff_drive_robot-main/maps/map_hospital` or `map_obstacles`.
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
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=hospital explore:=true
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
No file editing needed. The fleet is generated from `robot_count` + `robot_layout`
(`line`/`grid`/`circle`), or from an exact `robots_json` list for custom poses.
Spawn validation (on by default) relocates any robot whose generated/given pose
falls inside a wall, other robot, or outside a `zone_*` spawn area.

```bash
ros2 launch diff_drive_robot multi_robot.launch.py robot_count:=4 robot_layout:=grid
ros2 launch diff_drive_robot multi_robot.launch.py \
  robots_json:='[{"name":"robot1","x":-2,"y":-1},{"name":"robot2","x":-0.8,"y":-1}]'
```

Nav2 params for `drive_type:=diff` (default) come from a single hand-tuned
**template file** (`nav2_multirobot_params.yaml`) — the placeholder `ROBOT_NS` is
substituted at launch time. For `drive_type:=mecanum`/`ackermann` there is no
separate fleet-tuned template; instead the same single-robot
`nav2_params_mecanum.yaml`/`nav2_params_ackermann.yaml` used by
`slam_nav.launch.py` is namespaced programmatically at launch time (see
"Fleet-wide drive type" below) — one source of truth per drive type, either way.

### TF frame naming (critical for multi-robot)
Each robot uses `frame_prefix: <ns>/` in its RSP, so TF frames are unique:
- `robot1/base_link`, `robot1/odom`, `robot1/laser_frame`
- `robot2/base_link`, `robot2/odom`, `robot2/laser_frame`

Nav2 params (`amcl.base_frame_id`, `bt_navigator.robot_base_frame`, etc.) must
match these prefixed names. For `drive_type:=diff` the hand-tuned template file
handles this; for `mecanum`/`ackermann`, `_common.namespace_nav2_params()`
rewrites the single-robot params dict at launch time instead (same rule: prefix
frame names with `<ns>/`, leave the shared `map` frame alone, absolute per-robot
topics get `/<ns>` inserted).

```bash
# SLAM + frontier exploration in hospital (default)
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
| `launch/multi_robot.launch.py` | Main launch — `robot_count`/`robot_layout`/`robots_json` scale the fleet, no editing needed |
| `launch/_common.py` | Shared helper module (also used by `slam_nav.launch.py`): world resolution, Gazebo/RSP/laser-filter builders, nav2 params filename selection, and `namespace_nav2_params()` for fleet-wide drive types |
| `config/nav2_multirobot_params.yaml` / `_jazzy.yaml` | Hand-tuned diff-drive fleet template (ROBOT_NS placeholder) |
| `config/nav2_params_mecanum.yaml`, `nav2_params_ackermann.yaml`, etc. | Single-robot params, reused + auto-namespaced for `drive_type:=mecanum`/`ackermann` fleets |
| `config/mapper_params_multirobot.yaml` | SLAM params for robot1 in explore mode |

### Fleet-wide drive type
`drive_type`/`controller` — the same choice single-robot mode offers (§18, §18b) — apply to the whole fleet:
```bash
ros2 launch diff_drive_robot multi_robot.launch.py drive_type:=mecanum
ros2 launch diff_drive_robot multi_robot.launch.py drive_type:=ackermann
```
`diff` (default) is unchanged — it still uses the hand-tuned `nav2_multirobot_params.yaml`.
`mecanum`/`ackermann` have no hand-tuned fleet template, so `_make_robot_params()`
in `multi_robot.launch.py` detects that the selected file has no literal `ROBOT_NS`
placeholder and falls back to `_common.namespace_nav2_params()`, which rewrites
frame IDs/topics on the loaded dict and drops nodes that aren't part of the
per-robot fleet bringup (`map_server`, `docking_server`, `loopback_simulator`).
Footprint/controller tuning (`robot_radius`, inflation, critics, …) is whatever
the single-robot file already has — not re-tuned for tighter fleet spacing the
way the diff-drive template was, so treat those as a starting point.

### Bugs fixed
- `rsp.launch.py` now declares and passes `frame_prefix` to RSP → TF frames are correctly namespaced per robot (was causing robot not visible in Gazebo)
- Costmap local layer changed from `voxel_layer` (3D only) to `obstacle_layer` for 2D LaserScan
- AMCL per-robot added (was missing from `navigation_launch.py` which does not include AMCL)
- All frame IDs prefixed with robot namespace (was causing TF conflicts between robots)
- Gazebo plugin topics are namespaced per robot, so odom/scan/camera data no longer collapses onto shared root topics
- `robot_state_publisher` remaps `tf` and `tf_static` into each robot namespace, which avoids cross-robot static TF collisions

---

## 11. Fleet Management Stack

The optional fleet-management layer is enabled with:

```bash
ros2 launch diff_drive_robot multi_robot.launch.py fleet_mgmt:=true
```

It starts five cooperating nodes:

| Node | Role |
|---|---|
| `mission_server.py` | Accepts high-level per-robot missions and drives Nav2 actions |
| `task_allocator.py` | Assigns pending tasks to idle robots using Hungarian matching |
| `fleet_health.py` | Publishes fleet health telemetry to `/fleet/health` |
| `priority_collision_avoidance.py` | Makes lower-priority robots yield in predicted close conflicts |
| `deadlock_recovery.py` | Detects robots stuck mid-mission and triggers recovery maneuvers |

### Mission Server

`mission_server.py` sits above Nav2. Clients publish JSON to `/mission/execute`, and the server converts it into `NavigateToPose` goals.

Supported mission types:
- `goto`
- `sequence`
- `patrol`

Important behavior:
- Mission execution is concurrent per robot, not single-global.
- State is published on `/mission/state` as one message per robot.
- Each state message includes robot namespace, mission type, waypoint progress, and current target pose.

### Task Allocation

`task_allocator.py` keeps a shared task queue and watches `/mission/state` plus per-robot odometry.

Allocation flow:
1. Detect idle robots.
2. Build a robot-task distance matrix.
3. Solve the optimal assignment using the Hungarian algorithm.
4. Publish per-robot `goto` missions to `/mission/execute`.

Task lifecycle:
- `pending`
- `assigned`
- `done`
- `failed`

If a mission is cancelled, the task returns to `pending`.
If a mission fails, the task is retried up to the configured retry limit.

### Collision Avoidance vs CBS

This repo does not implement Conflict-Based Search (CBS) or another centralized multi-agent path planner.

Instead, it uses:
- independent Nav2 planners per robot
- a shared global map
- priority-based yielding for near conflicts
- deadlock recovery when a robot stops making progress

That means coordination here is reactive rather than globally optimal.

### Behavior Trees in Multi-Robot Mode

Each robot runs its own namespaced Nav2 BT navigator:
- `/robot1/bt_navigator`
- `/robot2/bt_navigator`

The custom tree in `config/bt/navigate_w_recovery.xml` is still a single-robot recovery tree:
- compute path
- follow path
- on failure: backup, spin, clear costmaps, wait

The fleet-management layer sits outside the BT. It coordinates missions and recovery around Nav2; it does not replace Nav2 with a centralized multi-robot behavior tree.

---

## 11b. Open-RMF Traffic Scheduling (experimental)

Section 11's `priority_collision_avoidance.py` / `deadlock_recovery.py` are reactive — robots yield to each other only once a conflict is imminent, with no global schedule. `rmf_fleet.launch.py` adds real Open-RMF traffic scheduling on top of the same Nav2 stacks instead: `rmf_traffic_schedule` negotiates conflict-free itineraries across the whole fleet *before* robots move, and `rmf_task_dispatcher` assigns submitted tasks (patrol loops, for now) to whichever registered robot can do them.

Open-RMF ships as system packages (`ros-humble-rmf-*`, apt-installed) — it is not a dependency of the rest of this repo, only of this optional integration.

Pieces:

| Piece | Package | Role |
|---|---|---|
| `rmf_traffic_schedule` | `rmf_traffic_ros2` | Traffic scheduler — negotiates conflict-free paths across the fleet |
| `rmf_task_dispatcher` | `rmf_task_ros2` | Assigns submitted tasks to registered robots |
| `rmf_fleet_adapter.py` | this package | Registers `robot1..robotN` as one RMF fleet; bridges RMF path commands to each robot's existing `/{ns}/navigate_to_pose` |
| `rmf_submit_task.py` | this package | CLI to submit a patrol task over `/task_api_requests` |

Architecture:
- The nav graph is built at startup from `config/locations.yaml` — one RMF waypoint per named location, connected in a star topology through `hallway` (`WAYPOINT_LANES` in `rmf_fleet_adapter.py`). Edit that table (or `locations.yaml`) if a world's rooms/corridors differ.
- `Nav2RobotCommand` (one per robot namespace) turns RMF's `follow_new_path` waypoint sequence into ordinary `NavigateToPose` action calls against that robot's Nav2 stack, and reports live position back to RMF from `/{ns}/amcl_pose`.
- Because the graph is anchored to fixed map-frame coordinates, this only works with `multi_robot.launch.py explore:=false` (static map + AMCL) — SLAM/explore mode has no fixed map for the graph to sit on.
- Battery/task-planner parameters are placeholders (`BATTERY_*` constants) — there is no real battery telemetry in this sim yet, so `account_for_battery_drain` is off.
- `dock()` hands off to the configured docking plugin (`aruco` by default → `aruco_dock.py`, or `noop` for bring-up). Per-dock marker/staging/charge settings live in `config/docks.yaml`. Visual docking retries with soft reverse and Nav2 restage; `stop()` / interrupt cancel the dock subprocess.

Run:

```bash
# 1. Static-map fleet (RMF's graph needs fixed map-frame coordinates)
ros2 launch diff_drive_robot multi_robot.launch.py explore:=false robot_count:=2

# 2. RMF traffic scheduling + task dispatch + fleet adapter
ros2 launch diff_drive_robot rmf_fleet.launch.py robot_count:=2

# 3. Submit a patrol task and watch the fleet negotiate shared corridor space
ros2 run diff_drive_robot rmf_submit_task.py patrol room_a room_b --rounds 2
```

Known gap: the exact JSON shape `rmf_task_dispatcher` expects on `/task_api_requests` follows the `rmf_api_msgs` task-request schema, which isn't vendored as a local file in this ROS distro's apt packages — `rmf_submit_task.py`'s payload matches what RMF's own demos publish, but is the one part of this integration not yet confirmed against a live dispatcher. If it's rejected, `ros2 topic echo /task_api_responses` while resubmitting will show why.

---

## 12. 2D vs 3D LiDAR

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

## 13. Adding a Camera Sensor (RGB / Depth)

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

## 14. New Tool Scripts

### `fleet_manager.py` — Product-like fleet CLI
```bash
ros2 run diff_drive_robot fleet_manager.py list          # list active robots
ros2 run diff_drive_robot fleet_manager.py status        # map/SLAM/Nav2 status
ros2 run diff_drive_robot fleet_manager.py add robot3 1.0 2.0   # dynamic spawn
ros2 run diff_drive_robot fleet_manager.py teleop robot1 # keyboard control
ros2 run diff_drive_robot fleet_manager.py goto robot2 3.0 -1.0 # nav goal
ros2 run diff_drive_robot fleet_manager.py dock robot1           # navigate to charging_dock
ros2 run diff_drive_robot fleet_manager.py undock robot1         # back away 0.5 m from the dock
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

## 15. Custom Obstacle Avoidance (`navigation.py`)

A simpler alternative to Nav2 — a 4-state finite state machine:

```
GOAL_SEEK ──obstacle──► FIND_CLEAR ──aligned──► MOVE_CLEAR ──moved──► REALIGN
    ▲                                                                      │
    └──────────────────────────────────────────────────────────────────────┘
```

Uses only `/scan` (LiDAR) and `/odom`, no map required. Good for unstructured environments but does not do global path planning.

---

## 16. Repository Organization (Recommended)

For this repo, a cleaner layout helps debugging and repeatability:

- Keep ROS package source under `src/diff_drive_robot-main/` only.
- Move generated/runtime artifacts out of root into:
  - `src/diff_drive_robot-main/maps/` for generated `map_*.yaml` and `map_*.pgm`
  - `logs/` for launch or benchmark logs
- Add `scripts/` at repo root only for workflow wrappers (build/test/run), not ROS nodes.
- Keep docs in `docs/` (`README.md` for quickstart, `concepts.md` for deep reference).
- Add `Makefile` or `justfile` commands for common flows (`build`, `slam-nav`, `frontier`, `save-map`).


## 17. A* Path Planner (`path_planning.py`)

Standalone global planner using the **A\* algorithm**:
- Converts the world to a discrete grid.
- Uses Euclidean distance as the heuristic.
- Supports 8-direction movement (including diagonals).
- Currently uses hardcoded obstacles — future: subscribe to `/map` OccupancyGrid.

---

## 18. Mecanum (Holonomic) Drive

`robot.launch.py drive_type:=mecanum` swaps the standard 2-wheel differential
base for a 4-wheel holonomic base that can strafe sideways and move diagonally
without rotating first.

### What changes vs diff drive
| Piece | Diff drive | Mecanum |
|---|---|---|
| URDF | `robot.urdf.xacro` → `robot_core.xacro` (2 driven rear wheels, 2 fixed low-friction front casters) | `robot_mecanum.urdf.xacro` → `robot_core_mecanum.xacro` (all 4 wheels driven, `continuous` joints) |
| Gazebo plugin | `gazebo_control.xacro`, `gz::sim::systems::DiffDrive` | `gazebo_control_mecanum.xacro`, `gz::sim::systems::MecanumDrive` (native to gz-sim7/8, no custom plugin needed) |
| Nav2 controller (Humble/DWB) | `nav2_params.yaml` — `max_vel_y: 0.0`, `vy_samples: 1` (no lateral motion), includes `RotateToGoal` critic | `nav2_params_mecanum.yaml` — `max_vel_y`/`min_vel_y` unlocked to ±0.5, `vy_samples: 10`, `RotateToGoal` dropped (holonomic robots don't need to pre-rotate) |
| Nav2 controller (Jazzy/MPPI) | `nav2_params_jazzy.yaml` — `motion_model: "DiffDrive"` | `nav2_params_mecanum_jazzy.yaml` — `motion_model: "Omni"` |

The `MecanumDrive` plugin takes `vx`/`vy`/`wz` from `cmd_vel` (via the same
`cmd_vel_safe` bridge topic diff drive uses) and computes wheel spin
kinematically. It doesn't simulate actual roller geometry, so wheel friction
in `robot_core_mecanum.xacro` is tuned low-but-uniform (`mu1`/`mu2` = 0.3) as
a compromise — enough grip to drive forward, permissive enough to strafe.
True anisotropic roller friction (`fdir1`, full grip along the roller,
frictionless across it) is a native-SDF technique used in Gazebo's own
`mecanum_drive.sdf` demo world, but isn't reliably expressible through the
`<gazebo reference="...">` URDF extension tags this repo's xacro pipeline
uses.

### Try it
```bash
ros2 launch diff_drive_robot robot.launch.py drive_type:=mecanum headless:=true rviz:=false

# strafe sideways with no rotation:
ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 18b. Ackermann (Car-Like) Drive

`robot.launch.py drive_type:=ackermann` swaps in a 4-wheel car-like base:
two fixed rear drive wheels plus two front wheels that steer through a small
knuckle link (`front_left_steering_link` / `front_right_steering_link`),
each yawing about Z relative to the chassis before spinning about its own axle.

| Piece | Diff drive | Ackermann |
|---|---|---|
| URDF | `robot.urdf.xacro` → `robot_core.xacro` | `robot_ackermann.urdf.xacro` → `robot_core_ackermann.xacro` (rear wheels: `continuous` joints on `base_link`; front wheels: `revolute` steering joint → steering link → `continuous` wheel joint) |
| Gazebo plugin | `gazebo_control.xacro`, `gz::sim::systems::DiffDrive` | `gazebo_control_ackermann.xacro`, `gz::sim::systems::AckermannSteering` (native gz-sim system — same one used in gz-sim's own `ackermann_steering.sdf` demo world) |
| Nav2 controller | DWB (Humble default) | Always MPPI — `nav2_params_ackermann.yaml`, `motion_model: "Ackermann"`, `AckermannConstraints.min_turning_r: 0.66` (computed from `wheel_base / tan(steering_limit)` = 0.45 / tan(0.6rad)). DWB has no car-like turning-radius constraint, so there's no DWB variant for this drive type. |

The `AckermannSteering` plugin takes the same `cmd_vel` (linear.x, angular.z)
input as `DiffDrive`/`MecanumDrive` and internally solves the steering angle
and per-wheel speeds — Nav2 doesn't need to know about steering joints at all,
it just needs `min_turning_r` so MPPI never plans a path tighter than the
vehicle can actually steer.

### Try it
```bash
ros2 launch diff_drive_robot robot.launch.py drive_type:=ackermann headless:=true rviz:=false

# Mapping by manually driving the car-like base:
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=hospital drive_type:=ackermann

# drive forward while turning — steering angle emerges from angular.z:
ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

---

## Quick Reference — Launch Files

| Launch file | What it does | Key args |
|---|---|---|
| `robot.launch.py` | Gazebo + robot + Nav2 full bringup with saved map | `map`, `world`, `robot_name`, `spawn_x/y/z/yaw`, `rviz`, `use_sim_time`, `drive_type` (`diff`\|`mecanum`\|`ackermann`) |
| `slam_nav.launch.py` | Gazebo + robot + SLAM Toolbox + Nav2 (+ optional auto frontier) | `world_name`, `world`, `explore`, `map_prefix`, `rviz`, `robot_name`, `spawn_x/y/z/yaw`, `drive_type` (`diff`\|`mecanum`\|`ackermann`), `controller` (`dwb`\|`mppi`) |
| `slam.launch.py` | Gazebo + robot + SLAM Toolbox mapping mode | `use_sim_time` |
| `multi_robot.launch.py` | Scalable N-robot fleet: SLAM+frontier or shared map + Nav2 per robot | `world`, `map`, `explore`, `slam_mode`, `robot_count`, `robot_layout`, `robots_json`, `drive_type` (`diff`\|`mecanum`\|`ackermann`), `controller` (`dwb`\|`mppi`), `fleet_mgmt`, `rviz`, `headless` |
| `nav2.launch.py` | Nav2 only (attach to running Gazebo) | `map`, `world`, `use_sim_time` |
| `rmf_fleet.launch.py` | Open-RMF traffic scheduling on top of an already-running static-map fleet (see §11b) | `robot_count`, `robots`, `fleet_name`, `map_name`, `adapter_delay` |
| `_common.py` | Not a launch file — shared helper module (world resolution, Gazebo/RSP/laser-filter builders, nav2 params selection + multi-robot namespacing) imported by `slam_nav.launch.py` and `multi_robot.launch.py` | — |

## Quick Reference — Scripts

| Script | What it does | Key ROS params |
|---|---|---|
| `navigation.py` | Custom obstacle-avoidance FSM (no Nav2 needed) | `goal_x`, `goal_y`, `base_speed`, `obstacle_threshold` |
| `path_planning.py` | Standalone A* path planner | `grid_size_x/y`, `resolution`, `safety_margin` |
| `waypoint_nav.py` | Navigate through a sequence of waypoints via Nav2 | `waypoints_file`, `frame_id` |
| `frontier_explorer.py` | Autonomous map exploration via frontier detection + optional auto-save | `min_frontier_size`, `revisit_radius`, `poll_period`, `map_save_path` |
| `check_odometry.py` | Debug odometry data | — |
| `reset_pose.py` | Reset robot pose in simulation | `world_name`, `robot_name`, `reset_x/y/z/yaw` |
| `rmf_fleet_adapter.py` | Registers the fleet with Open-RMF, bridges RMF path commands to Nav2 (see §11b) | `--fleet-name`, `--map-name`, `--robots` |
| `rmf_submit_task.py` | Submits a patrol task to `rmf_task_dispatcher` to exercise traffic scheduling | `category`, `places`, `--rounds`, `--wait` |

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
ros2 run nav2_map_server map_saver_cli -f src/diff_drive_robot-main/maps/map_hospital
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

## 18. 3-Tier Autonomy Stack

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

## 19. Collision Monitor

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
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=hospital safety:=true

# Or run standalone:
ros2 run diff_drive_robot collision_monitor.py --ros-args \
    -p stop_distance:=0.35 -p watch_all_around:=true

# Monitor state:
ros2 topic echo /collision_monitor/state
```

---

## 20. Mission Server

`scripts/mission_server.py` is the top-level mission execution daemon.

**How it works:**
1. Runs as a persistent ROS 2 node.
2. Subscribes to `/mission/execute` (std_msgs/String JSON).
3. Runs missions concurrently per robot instead of using one global active mission.
4. Sends `NavigateToPose` goals to the target robot's Nav2 stack.
5. Publishes current state to `/mission/state` (std_msgs/String JSON) at 1 Hz, one message per robot.

**Mission types:**

| Type | Behaviour |
|---|---|
| `patrol` | Loop through waypoints indefinitely |
| `sequence` | Visit waypoints once in order, then DONE |
| `goto` | Navigate to a single pose, then DONE |

**State machine:** `IDLE → NAVIGATING → DONE / FAILED`, with `RECOVERING` used for patrol retries.
Cancel with `action: cancel`:
- with `robot` set: cancels only that robot's mission
- with empty `robot`: cancels all tracked missions

```bash
# Start daemon:
ros2 run diff_drive_robot mission_server.py

# Patrol robot1 through 3 waypoints:
ros2 run diff_drive_robot mission_server.py patrol robot1 1,2,0 3,4,90 0,0,180

# Single goal:
ros2 run diff_drive_robot mission_server.py goto robot1 3.0 -1.0 45

# Check all robots:
ros2 run diff_drive_robot mission_server.py status

# Check just one robot:
ros2 run diff_drive_robot mission_server.py status robot1

# Cancel:
ros2 run diff_drive_robot mission_server.py cancel

# Via fleet_manager:
ros2 run diff_drive_robot fleet_manager.py mission robot1 patrol 1,2,0 3,4,90
ros2 run diff_drive_robot fleet_manager.py mission robot1 status
ros2 run diff_drive_robot fleet_manager.py collision robot1
```

---

## 21. Velocity Smoother

`nav2_velocity_smoother` is a Nav2 lifecycle node that applies jerk-limiting to `cmd_vel`.

**Why it matters:**  
The MPPI controller outputs velocity commands that can change abruptly between ticks (10–20 Hz). Without smoothing, the robot's drivetrain receives step changes in velocity that cause:
- Wheel slip
- Mechanical stress
- Oscillation at high speeds

**How it works:**
1. Subscribes to `/cmd_vel` (raw controller output).
2. Applies configurable max acceleration (`max_accel`) and deceleration (`max_decel`) limits.
3. Publishes filtered velocity to `/cmd_vel_smoothed`.
4. `gz_bridge.yaml` also bridges `cmd_vel_smoothed → Gazebo /cmd_vel`, so the robot receives the smooth stream.

**Pipeline:**
```
MPPI Controller → /cmd_vel → velocity_smoother → /cmd_vel_smoothed → gz_bridge → Gazebo
```

**Key parameters** (in `nav2_params.yaml` under `velocity_smoother`):

| Parameter | Default | Meaning |
|---|---|---|
| `smoothing_frequency` | 20 Hz | Output rate |
| `max_velocity` | [1.0, 0.0, 2.5] | [vx, vy, wz] max m/s or rad/s |
| `max_accel` | [2.5, 0.0, 3.2] | [vx, vy, wz] max acceleration |
| `max_decel` | [-2.5, 0.0, -3.2] | [vx, vy, wz] max deceleration |
| `feedback` | OPEN_LOOP | OPEN_LOOP or CLOSED_LOOP (uses odom) |

Started automatically by `slam_nav.launch.py` 10 seconds after Nav2.

---

## 22. Custom Behavior Tree

`config/bt/navigate_w_recovery.xml` is a custom Nav2 Behavior Tree that replaces the default navigation BT.

**What a Behavior Tree is:**  
A BT is a tree of nodes that ticks top-down every cycle. Each node returns SUCCESS, FAILURE, or RUNNING. Nav2's bt_navigator runs your BT on every navigation goal.

**Node types used:**

| Node | Type | What it does |
|---|---|---|
| `RecoveryNode` | Decorator | Retries child N times before failing |
| `PipelineSequence` | Control | Runs children in parallel pipeline — stops all if one fails |
| `RateController` | Decorator | Throttles child to run at a fixed Hz |
| `ReactiveFallback` | Control | Re-ticks all children every tick; succeeds on first success |
| `RoundRobin` | Control | Tries next child each time it's called |
| `ComputePathToPose` | Action | Calls global planner |
| `FollowPath` | Action | Calls local controller (MPPI) |
| `ClearEntireCostmap` | Action | Service call to clear global or local costmap |
| `BackUp` | Action | Drives backward |
| `Spin` | Action | Rotates in place |
| `Wait` | Action | Pauses for N seconds |
| `GoalUpdated` | Condition | Succeeds if goal changed since last tick |

**Recovery sequence (this custom BT):**
```
NavigateToPose goal received
  └─ Retry up to 6 times:
       ├─ [Try] Plan + Follow path (replanning at 1 Hz)
       └─ [Recover] RoundRobin:
            1. BackUp 0.20m        — escape contact
            2. Spin 90°            — fresh scan data
            3. Clear both costmaps — force full replan
            4. Wait 3s             — let dynamic obstacles clear
```

Registered in `nav2_params.yaml`:
```yaml
bt_navigator:
  default_nav_to_pose_bt_xml: "<pkg_share>/config/bt/navigate_w_recovery.xml"
```

To revert to Nav2's built-in BT, set that value to `""`.

---

## 23. Coverage Path Planner

`scripts/coverage_planner.py` computes a **boustrophedon** (lawnmower) sweep path over the free space of the current map and executes it via Nav2's `FollowWaypoints`.

**Algorithm:**
1. Receive `/map` (OccupancyGrid from SLAM or map_server).
2. Identify FREE cells (value = 0).
3. Erode free space by `robot_radius` to guarantee wall clearance.
4. Find bounding box of navigable region.
5. Sweep horizontal scan lines separated by `sweep_spacing` metres, alternating direction each line.
6. Emit one waypoint per `sweep_spacing` interval on each navigable line.
7. Optionally sort waypoints to start from the robot's current TF position.
8. Send all waypoints to `FollowWaypoints` action server.

**Pattern:**
```
→ → → → → → → → →
                 ↓
← ← ← ← ← ← ← ←
↓
→ → → → → → → → →
```

**Parameters:**

| Parameter | Default | Meaning |
|---|---|---|
| `sweep_spacing` | 0.5 m | Distance between scan lines |
| `robot_radius` | 0.25 m | Clearance from walls |
| `start_from_robot` | true | Start nearest waypoint to robot pose |
| `robot_ns` | `''` | Namespace (multi-robot) |

```bash
# Single robot coverage after mapping:
ros2 run diff_drive_robot coverage_planner.py

# Tighter sweep (warehouse):
ros2 run diff_drive_robot coverage_planner.py --ros-args -p sweep_spacing:=0.4

# Multi-robot:
ros2 run diff_drive_robot coverage_planner.py --ros-args -p robot_ns:=robot2
```

---

## 24. Multi-Robot Task Allocator

`scripts/task_allocator.py` implements a Hungarian-assignment task allocator that sits above `mission_server.py`.

**How it works:**
1. Maintains a shared task queue — a list of `(x, y, yaw)` poses with IDs.
2. Discovers active robots from `/*/cmd_vel` topics (same as fleet_manager).
3. Subscribes to `/mission/state` to track which robots are IDLE/DONE.
4. Subscribes to `/<ns>/odom` to know each robot's current position.
5. Every 0.5 s: build a robot-task distance matrix and solve the minimum-total-cost assignment.
6. Sends `goto` missions to `/mission/execute` (consumed by mission_server daemon).
7. When mission_server reports DONE, marks the task complete.
8. When mission_server reports FAILED or IDLE, re-queues or fails the task based on retry count.

**Topics:**

| Topic | Direction | Content |
|---|---|---|
| `/task_queue/add` | in | JSON `{x, y, yaw, label}` — add task |
| `/task_queue/clear` | in | Any JSON — remove pending tasks |
| `/task_queue/state` | out | JSON queue + robot states at 2 Hz |
| `/mission/state` | in | Robot mission states (from mission_server) |
| `/mission/execute` | out | goto commands to mission_server |
| `/<ns>/odom` | in | Robot position for distance calculation |

**Task states:** `pending → assigned → done` or `failed`

```bash
# Start daemons (or launch `multi_robot.launch.py fleet_mgmt:=true`):
ros2 run diff_drive_robot task_allocator.py

# Add tasks:
ros2 run diff_drive_robot task_allocator.py add 2.0 1.5 0 pickup_A
ros2 run diff_drive_robot task_allocator.py add 4.0 -1.0 90 dock_B
ros2 run diff_drive_robot task_allocator.py add 0.0 3.0 180

# Monitor:
ros2 run diff_drive_robot task_allocator.py status

# Or via fleet_manager:
ros2 run diff_drive_robot fleet_manager.py tasks add 2.0 1.5 0 pickup_A
ros2 run diff_drive_robot fleet_manager.py tasks status
ros2 run diff_drive_robot fleet_manager.py tasks clear
```
