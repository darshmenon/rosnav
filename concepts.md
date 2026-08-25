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
- `map → odom → base_link → laser_frame` (lidar)
- `base_link → imu_link` (IMU, used by Cartographer)
- `base_link → camera_link` (RGB / RGB-D when enabled)
- Lets Nav2 know where the robot is in the world at any moment.

### Actions
**Actions** are long-running tasks with feedback (e.g., "navigate to pose").
Nav2 exposes `navigate_to_pose` and `follow_waypoints` as action servers.

---

## 2. Gazebo Harmonic

A physics simulator that models the robot's body, wheels, sensors, and environment.
The **gz_bridge** translates Gazebo topics to ROS 2 topics and back.
Config is selected by `_common.gz_bridge_yaml(lidar_type, enable_rgbd)`:

| Config | When |
|---|---|
| `gz_bridge.yaml` | Default 2D lidar (+ optional RGB) |
| `gz_bridge_3d.yaml` | `lidar_type:=3d` (no empty `/scan` bridge) |
| `gz_bridge_rgbd.yaml` | `enable_rgbd:=true` / `slam_algo:=vslam`\|`multisensor` |
| `gz_bridge_3d_rgbd.yaml` | 3D lidar + RGB-D (`multisensor` with `lidar_type:=3d`, etc.) |

Typical ROS topics after bridging:
- `/scan_raw` ← Gazebo `/scan` (2D lidar only)
- `/points` ← Gazebo `/points/points` (3D lidar PointCloudPacked)
- `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/depth/points` (RGB-D)
- `/cmd_vel_safe` (ROS) → Gazebo `/cmd_vel` (motors)
- `/odom`, `/tf`, `/clock`, `/imu`

---

## 3. SLAM (Simultaneous Localisation and Mapping)

SLAM builds a map while tracking the robot inside it. Every backend here still
publishes a 2D `nav_msgs/OccupancyGrid` on `/map` (plus `map→odom` TF) so Nav2
is unchanged — only the mapper behind `/map` swaps.

### Mapping vs navigating on a saved map

| Mode | Arg | What runs |
|---|---|---|
| Live mapping (default) | `slam:=true` | Chosen `slam_algo` builds `/map`; Nav2 follows |
| Nav on saved map | `slam:=false` | `map_server` + **AMCL** (or RTAB localize if `slam_algo:=vslam` + `rtabmap_db`) |
| Explore while mapping | `explore:=true` (needs `slam:=true`) | Frontier explorer sends Nav2 goals into unknown |
| SLAM backend | `slam_algo:=…` | See matrix below |

```bash
# Build the map (SLAM on)
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital

# Auto-explore and progressively save
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital explore:=true

# Navigate on the saved map (SLAM off → AMCL)
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital slam:=false
# Maps autosave under src/rosnav_bot/maps/map_<world> when explore:=true
```

Fleet equivalent: `multi_robot.launch.py` uses `explore:=true/false` and optional
`slam_mode:=single|multi` (see §10).

### Algorithm matrix (`slam_algo`)

| `slam_algo` | Package | Sensors | When to use |
|---|---|---|---|
| `2d` (default) | **slam_toolbox** | `/scan` + `/odom` | Indoor 2D mapping; default / most tested |
| `cartographer` | **cartographer_ros** | `/scan` + `/odom` + **`/imu`** | Strong loop closure; apt: `ros-$ROS_DISTRO-cartographer-ros` |
| `3d` | **rtabmap_slam** | `/points` + RGB + `/odom` | Needs `lidar_type:=3d`; ICP + RGB place recognition |
| `vslam` | **rtabmap_slam** | **RGB-D** + `/odom` | Textured worlds; can localize from a `.db` (no AMCL) |
| `multisensor` | **rtabmap_slam** | **RGB-D + lidar** + `/odom` | Lidar ICP + depth occupancy + visual cues |
| `cslam` | **cslam** + RTAB | `/points` (fleet) | Swarm-SLAM; `link_third_party.sh --cslam` + `lidar_type:=3d` |
| `orbslam3` | **ORB-SLAM3** (sidecar) + `orb_slam3_occupancy_bridge.py` | RGB-D | Feature-based VSLAM; tracker runs in a separate Docker/bare-metal process — see below |

Missing optional packages fall back to `2d` (or `3d` for `cslam`) with a launch warning.

### ORB-SLAM3 (`slam_algo:=orbslam3`)

The ORB-SLAM3 tracker itself runs outside this launch file's process tree —
in `docker/orb_slam3/` (Docker) or via `docker/orb_slam3/build_bare_metal.sh`
(no Docker) — because it has no ROS distro apt package (from-source OpenCV 4.4
+ Pangolin + native build, ~30-60 min) and is GPLv3 vs. `rtabmap_slam`'s BSD.
Recipe re-derived from
[suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker)
(a working reference build), with rosnav_bot's own camera topics, frames, and
computed intrinsics swapped in — see `docker/orb_slam3/README.md`.

`slam_algo:=orbslam3` starts `scripts/orb_slam3_occupancy_bridge.py`, which
turns the sidecar's `map->odom` TF + the depth camera into a real
`nav_msgs/OccupancyGrid` on `/map` (Bresenham ray-traced free space, height-
band occupied endpoints) — ORB-SLAM3 itself only publishes a sparse
feature-point map, not an occupancy grid, so Nav2/the frontier explorer would
have nothing to consume without this bridge. `slam:=` is ignored for this
backend (the sidecar owns its own mapping-vs-localizing state via
`System.SaveAtlasToFile`/`LoadAtlasFromFile`); `/map` stays all-unknown until
the sidecar starts publishing:

```bash
ros2 launch rosnav_bot slam_nav.launch.py slam_algo:=orbslam3 world_name:=cafe explore:=true
docker compose up orb_slam3          # separate terminal — see docker/orb_slam3/README.md
```

**vs. `slam_algo:=vslam` (RTAB-Map RGB-D):**

| | RTAB-Map (`vslam`) | ORB-SLAM3 (`orbslam3`) |
|---|---|---|
| Registration | Depth ICP + visual bag-of-words loop closure | Pure ORB feature tracking + bundle adjustment |
| Process | In-process node (`slam_nav.launch.py`) | Separate container/process + this repo's occupancy bridge |
| Integration | Native colcon package, apt-installable | Separate Docker/bare-metal build |
| License | BSD | GPLv3 |
| Occupancy grid for Nav2 | Built into RTAB-Map (`Grid/Sensor=1`) | `orb_slam3_occupancy_bridge.py`'s own ray-traced grid — simpler than RTAB-Map's, good enough for costmaps/exploration |
| Multi-session map reuse | `rtabmap_db` (SQLite) | `System.SaveAtlasToFile`/`LoadAtlasFromFile` |
| Best fit here | Default VSLAM choice — one process, native build | Tracking-robustness comparison against RTAB-Map, or when you specifically want ORB-SLAM3's tracker |

**Verified working end-to-end** (2026-08-25, bare-metal build): with
`slam_algo:=orbslam3` running and the ORB-SLAM3 sidecar tracking against the
sim's camera, `/map` filled in with real occupied/free cells (not just
unknown) and Nav2's `global_costmap` matched it exactly — confirming the
whole chain (tracker → `map->odom` TF → `orb_slam3_occupancy_bridge.py` →
`/map` → Nav2) actually works, not just that the pieces start without
crashing.

**Bugs found and fixed getting there** — none of these are hypothetical,
each one broke a real run:

1. **Sophus header missing at colcon-build time.** ORB_SLAM3's own
   `build.sh` runs `make install` for its vendored Sophus with no
   `CMAKE_INSTALL_PREFIX`, defaulting to `/usr/local`. In the reference
   Docker image (root user) this silently succeeds, and `/usr/local/include`
   is on GCC's default search path anyway — so it "just works" there with no
   `find_package` plumbing. As a non-root bare-metal user it fails
   permission-denied, *silently* (no `set -e` in their `build.sh`), leaving
   `sophus/se3.hpp` uninstalled anywhere → `orb_slam3_ros2_wrapper`'s colcon
   build fails with `fatal error: sophus/se3.hpp: No such file or
   directory`. Fix (`build_bare_metal.sh`): explicitly
   `cmake --install Thirdparty/Sophus/build --prefix "$PREFIX"` after
   `build.sh` runs — no rebuild needed, just installs from the
   already-built tree.
2. **Hardcoded Docker paths in CMake and every sensor-mode launch file.**
   `orb_slam3_ros2_wrapper/CMakeModules/FindORB_SLAM3.cmake` hardcodes
   `set(ORB_SLAM3_ROOT_DIR "/home/orb/ORB_SLAM3")` — a plain `set()`, not an
   environment-variable read, despite its own comment claiming otherwise.
   `rgbd.launch.py` (and `rgbd_imu`/`mono`/`mono_imu`/`stereo`/`stereo_imu`)
   separately hardcode `/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt` and
   `/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/...` for the vocabulary
   and settings-file arguments passed to the node. Both match the Docker
   image's fixed layout exactly, so nothing breaks there — but bare-metal
   clones to `$HOME/orb_slam3_src`/`$HOME/orb_slam3_ws`, so the node dies
   immediately with `Failed to open settings file at: /root/colcon_ws/...`.
   Fix: `build_bare_metal.sh` `sed`-patches both the CMake file and every
   launch file's hardcoded strings in the local clone after cloning (not
   upstream).
3. **`visualization: true` crashes on startup.** ORB_SLAM3's `Viewer::Run()`
   (started when `System`'s `bUseViewer` — this repo's `visualization` ROS
   param — is true) bundles the Pangolin 3D map view together with a plain
   `cv::namedWindow`/`imshow` "Current Frame" debug window in the same
   function. The latter needs GTK-enabled OpenCV; this repo's OpenCV 4.4
   build deliberately skips GTK/GStreamer dev headers (ORB-SLAM3 doesn't use
   OpenCV's own video I/O) → `terminate called after throwing... OpenCV...
   Rebuild the library with ... GTK+ ... support`. Since the flag bundles
   both windows together, keeping the Pangolin view without rebuilding
   OpenCV isn't possible without patching `Viewer.cc` — not done here. Fix:
   `docker/orb_slam3/params/rosnav_rgbd_ros_params.yaml` defaults
   `visualization: false`; flip it only after rebuilding OpenCV with
   `libgtk-3-dev` present before the cmake configure step.
4. **`colcon build` ignored `BUILD_JOBS`.** `cmake --build ... -j"$BUILD_JOBS"`
   controls the manual OpenCV/Pangolin build steps, but colcon's own
   `make` invocation for the ROS packages runs at full `nproc` regardless —
   confirmed via `ps aux` showing `-j22` even with `BUILD_JOBS=1` requested.
   Fix: `export MAKEFLAGS="-j${BUILD_JOBS}"` before the `colcon build` call,
   which `make` (the underlying build tool for `ament_cmake` packages)
   reads directly.
5. **`orb_slam3_occupancy_bridge.py`'s own bug**: `Time(msg=msg.header.stamp)`
   isn't valid rclpy API (`TypeError: Time.__init__() got an unexpected
   keyword argument 'msg'`) — crashed the bridge node on the very first
   depth frame, so `/map` never appeared even though everything upstream
   was healthy. Fix: `Time.from_msg(msg.header.stamp)`.

### Sensors & TF

```
map → odom → base_link → laser_frame | imu_link | camera_link
```

- **2D lidar** — filtered `/scan` feeds slam_toolbox / Cartographer / multisensor.
- **3D lidar** — `lidar_type:=3d` → `/points`; `pointcloud_to_scan.py` still feeds `/scan` for Nav2.
- **IMU** — `urdf/imu.xacro` on every chassis; Cartographer tracks `imu_link`.
- **RGB / RGB-D** — forced for `3d` (RGB), `vslam`, and `multisensor`.

Wheel odom from Gazebo remains the sole `odom→base_link` publisher for every
backend (no separate ICP/visual odometry node) — avoids a two-parent TF conflict.

### slam_toolbox (`slam_algo:=2d`)

Default. Params: `config/mapper_params_online_async.yaml` (and multi-robot variants).

```bash
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital explore:=true
```

### Cartographer (`slam_algo:=cartographer`)

Google Cartographer 2D submaps + pose-graph. Launch starts `cartographer_node` +
`cartographer_occupancy_grid_node` (`/map`). Lua: `config/cartographer/rosnav_2d.lua`
(IMU) and `rosnav_2d_no_imu.lua`. Upstream includes are merged at runtime from
`cartographer_ros/configuration_files`.

```bash
sudo apt install ros-$ROS_DISTRO-cartographer-ros
ros2 launch rosnav_bot slam_nav.launch.py slam_algo:=cartographer explore:=true
```

### RTAB-Map lidar (`slam_algo:=3d`)

Requires `lidar_type:=3d`. ICP on `/points`; RGB (no depth) for bag-of-words loop
closure only. Always projects 2D `/map` for Nav2; `octomap:=true` also fills
`/octomap_binary` / `/octomap_full`.

```bash
ros2 launch rosnav_bot slam_nav.launch.py lidar_type:=3d slam_algo:=3d explore:=true
ros2 launch rosnav_bot slam_nav.launch.py lidar_type:=3d slam_algo:=3d octomap:=true
```

### RTAB-Map visual (`slam_algo:=vslam`)

Forces `enable_rgbd:=true`. Visual registration (`Reg/Strategy=0`), occupancy from
depth (`Grid/Sensor=1`). Prefer textured Fuel worlds (`cafe`, …). Details also in §13.

```bash
ros2 launch rosnav_bot slam_nav.launch.py slam_algo:=vslam world_name:=cafe \
  rtabmap_db:=src/rosnav_bot/maps/rtabmap_cafe.db explore:=true
ros2 launch rosnav_bot slam_nav.launch.py slam:=false slam_algo:=vslam \
  rtabmap_db:=src/rosnav_bot/maps/rtabmap_cafe.db   # RTAB replaces AMCL
```

### Multi-sensor RTAB-Map (`slam_algo:=multisensor`)

One RTAB-Map node: RGB-D **and** lidar (`Grid/Sensor=2`, ICP `Reg/Strategy=1`).
Default 2D `/scan`, or `lidar_type:=3d` for `/points`.

```bash
ros2 launch rosnav_bot slam_nav.launch.py slam_algo:=multisensor explore:=true
ros2 launch rosnav_bot slam_nav.launch.py lidar_type:=3d slam_algo:=multisensor explore:=true
```

### Swarm-SLAM (`slam_algo:=cslam`)

Multi-robot only (`multi_robot.launch.py slam_mode:=multi lidar_type:=3d`). Needs
the optional `cslam` package.

```bash
bash src/rosnav_bot/scripts/link_third_party.sh --cslam
colcon build --packages-up-to cslam
ros2 launch rosnav_bot multi_robot.launch.py slam_mode:=multi lidar_type:=3d slam_algo:=cslam
```

### Benchmarking algorithms

`scripts/benchmark_slam.sh` runs each algo headless with the same world +
`explore:=true` budget, then `benchmark.py` (`mode:=slam`) logs coverage % and
time-to-converge → `/tmp/rosnav_slam_bench/<stamp>/comparison.md`. This compares
*algorithms* (`slam_algo:=2d|cartographer|...`, §3 above); to compare
slam_toolbox's own run modes instead, use `slam_toolbox_mode:=online_async|
online_sync|lifelong` on `slam_nav.launch.py` (`online_async`, the default, drops
scans under load; `online_sync` blocks and processes every scan; `lifelong` adds
long-term memory/eviction for revisited areas — falls back to `online_async` with
a log line if `lifelong_slam_toolbox_node` isn't installed).

```bash
./src/rosnav_bot/scripts/benchmark_slam.sh
WORLD=maze DURATION_S=90 ALGOS="2d cartographer multisensor" ./src/rosnav_bot/scripts/benchmark_slam.sh

# Manual (stack already running)
ros2 run rosnav_bot benchmark.py --ros-args \
  -p mode:=slam -p label:=carto -p duration_sec:=120 -p out_dir:=~/rosnav_benchmarks

# Diff N JSON reports offline — also writes comparison PNG charts + a single
# self-contained HTML dashboard (comparison.html) next to the first input,
# unless matplotlib isn't installed (table-only fallback) or -p charts:=false.
ros2 run rosnav_bot benchmark.py --ros-args -p mode:=report \
  -p inputs:="['~/rosnav_benchmarks/2d_slam.json','~/rosnav_benchmarks/carto_slam.json']"
```

`benchmark.py` also has `mode:=nav` (goal timing/speed/efficiency/recoveries —
compare `controller:=dwb|mppi|rpp`, §7 below), `mode:=localization` (AMCL
covariance trace), and `mode:=accuracy` (drift, + RMSE/max error if a
ground-truth topic is bridged — compare `localization_filter:=ekf|ukf`, §6),
not only mappers (see §6b for EKF vs UKF). Typical controller comparison:

```bash
# Run once per controller (each launched separately, same world/goals)
ros2 launch rosnav_bot slam_nav.launch.py world_name:=maze slam:=false controller:=dwb &
ros2 run rosnav_bot benchmark.py --ros-args -p mode:=nav -p label:=maze_dwb \
  -p goals_file:=src/rosnav_bot/config/waypoints.yaml
# ...repeat with controller:=mppi -> label:=maze_mppi, controller:=rpp -> label:=maze_rpp

# Then compare all three
ros2 run rosnav_bot benchmark.py --ros-args -p mode:=report \
  -p inputs:="['~/rosnav_benchmarks/maze_dwb_nav.json','~/rosnav_benchmarks/maze_mppi_nav.json','~/rosnav_benchmarks/maze_rpp_nav.json']"
```

EKF vs UKF is the same pattern with `mode:=accuracy` and
`localization_filter:=ekf|ukf` instead of `controller:=...`.

---

## 6b. Localization Filter — EKF vs UKF

`slam_nav.launch.py localization_filter:=<ekf|ukf>` (default `ekf`). This is the
`robot_localization` filter that fuses wheel odom (`/odom`) + IMU gyro (`/imu`)
into the `odom -> base_link` TF — it's the *only* thing publishing that TF (the
Gazebo drive plugin's own TF is routed off `/tf`, see `_common.localization_filter_node`),
so it's always on, not a toggle. Both filters share the exact same fusion config
(`config/ekf.yaml` / `config/ukf.yaml` — same `odom0_config`/`imu0_config`),
so `ukf.yaml` only adds the UKF-specific sigma-point tuning (`alpha`/`kappa`/`beta`).

- **EKF** — linearizes the motion/measurement model around the current estimate
  (first-order Taylor expansion) each step. Cheaper; the standard choice, and
  usually fine for this robot's near-linear diff-drive dynamics at low speed.
- **UKF** — propagates a small set of deterministically-chosen "sigma points"
  through the *actual* nonlinear model instead of linearizing it, then
  reconstructs the mean/covariance from the transformed points. More accurate
  through sharp turns (where the linearization error EKF accepts is largest), at
  a small extra CPU cost from the sigma-point sampling.

RViz: `slam_explore.rviz` has "Wheel Odom (raw)" (`/odom`, disabled by default)
and "Fused Odom (EKF/UKF)" (`/odometry/filtered`, enabled, with its position
covariance ellipse on) — toggle the raw one on to see the filter's correction
visually, or watch the fused ellipse alone to see EKF/UKF confidence over time.

Compare numerically with `benchmark.py mode:=accuracy` (drift-only, no setup
needed; add `ground_truth_topic:=...` for absolute RMSE if a ground-truth pose
is bridged) — see §3's Benchmarking algorithms section for the report/chart flow.

---

## 4. Nav2 Stack

Nav2 is the ROS 2 navigation framework. It is a collection of nodes managed by **lifecycle managers**.

### Components

| Component | Role |
|---|---|
| **map_server** | Loads a saved map yaml and publishes it on `/map` |
| **AMCL** | Particle-filter localisation — figures out where the robot is on the loaded map using LiDAR |
| **planner_server** | Global path planner (NavFn/A*) — finds a route from start to goal |
| **controller_server** | Local controller (DWB/MPPI/RPP, see §7) — follows the global path while avoiding nearby obstacles |
| **behavior_server** | Recovery behaviours — spin, backup, wait when the robot gets stuck |
| **bt_navigator** | Behaviour Tree — orchestrates all the above components for a navigation goal |
| **velocity_smoother** | Smooths velocity commands to prevent jerky motion |
| **collision_monitor** | Emergency brake if an obstacle enters the safety zone |

### Launch Modes

- **`slam_nav.launch.py`** — primary single-robot entry. Toggle mapping vs localisation with `slam:=true|false` (see §3); Nav2 always comes up.
- **`bringup_launch.py`** — full stack (map_server + AMCL + navigation). Use for autonomous navigation with a saved map.
- **`navigation_launch.py`** — navigation stack only (no map_server). Use when SLAM is running separately.
- **`robot.launch.py`** — Gazebo + robot + Nav2 on a saved `map:=…` (nav-on-map path; good for drive-type demos).

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

**Obstacle layer plugin** — both costmaps use `nav2_costmap_2d::VoxelLayer`, not the 2D-only `ObstacleLayer` (all `nav2_params*.yaml` / multirobot templates). Observation sources are listed unconditionally:

| Source | Topic | Active when |
|---|---|---|
| `scan` | `/scan` | Always (2D lidar, or projected from `/points` when `lidar_type:=3d`) |
| `points` | `/points` | `lidar_type:=3d` (empty otherwise — harmless) |
| `depth` | `/camera/depth/points` | `enable_rgbd:=true` / `slam_algo:=vslam` (empty otherwise) |

With the default 2D lidar, VoxelLayer behaves like ObstacleLayer (single-slice marking from `/scan`). With 3D lidar it marks a real voxel grid from `/points` so overhangs (shelves, tables) show up. Depth adds close-range front obstacles from the RGB-D cloud. `z_voxels` is capped at 16 — `voxel_grid` packs each column into a 16-bit mask, so more silently errors every cycle ("can only support up to 16 z values"); `z_resolution: 0.125` × `z_voxels: 16` = 2.0m, matching `max_obstacle_height`.

**OctoMap** — RTAB-Map (`slam_algo:=3d` or `vslam`) always projects its map to the 2D `/map` OccupancyGrid Nav2 uses; `octomap:=true` additionally sets `Grid/3D:=true` so `/octomap_binary`/`/octomap_full` (and `/octomap_occupied_space`) populate a real 3D voxel map for visualization — Nav2 itself still consumes `/map` + the VoxelLayer above, not the octomap directly.

---

## 7. Local Controllers — DWB vs MPPI vs RPP

`slam_nav.launch.py controller:=<dwb|mppi|rpp>` (Humble, diff-drive; ignored on
Jazzy which defaults to MPPI). All three are configured under
`controller_server → FollowPath`, in `nav2_params.yaml` / `nav2_params_mppi.yaml` /
`nav2_params_rpp.yaml` respectively — same costmaps, same max speed (0.15 m/s),
so a `benchmark.py mode:=nav` run across all three is apples-to-apples.

- **DWB** (`dwb_core::DWBLocalPlanner`, default) — trajectory rollout: samples a
  grid of (vx, vtheta) velocity pairs, forward-simulates each for `sim_time`, and
  scores them with a set of critics (`PathAlign`, `GoalDist`, `Oscillation`, ...).
- **MPPI** (`nav2_mppi_controller::MPPIController`, `controller:=mppi`) — **Model
  Predictive Path Integral**: samples thousands of random velocity trajectories in
  parallel each control cycle, scores them against the same kind of critics, and
  executes a cost-weighted average of the low-cost ones (not just the single best,
  unlike DWB). `visualize: true` in `nav2_params_mppi.yaml` publishes every sampled
  candidate rollout on `/trajectories` (MarkerArray, colored by cost) — add it as
  a MarkerArray display in RViz to actually watch the velocity-sample generation;
  it's wired into `slam_explore.rviz` already (disabled by default, enable when
  `controller:=mppi`).
- **RPP** (`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`,
  `controller:=rpp`) — carrot-chasing pursuit controller: picks a lookahead point
  on the path and steers a circular arc toward it, regulating speed down on tight
  curvature/near obstacles. No sampling — cheapest of the three, but no explicit
  obstacle cost in the arc choice (relies on `use_collision_detection` instead).

All three publish their selected path on `/local_plan` (Path display in
`slam_explore.rviz`, "Local Plan").

---

## 8. Waypoint Following

`scripts/waypoint_nav.py` uses Nav2's **FollowWaypoints** action:
1. You define a list of (x, y, yaw) poses.
2. The node sends all poses at once to the action server.
3. Nav2 navigates to each in sequence, reporting feedback after each one.

```bash
ros2 run rosnav_bot waypoint_nav.py
```

Edit `WAYPOINTS` at the top of the script to change the route.

---

## 9. Frontier Exploration

`scripts/frontier_explorer.py` implements autonomous map exploration:

1. Subscribe to `/map` (OccupancyGrid from SLAM).
2. Find **frontier cells** — free cells (value 0) adjacent to unknown cells (value -1) — via a pluggable `frontier_detector`: `wfd` (default, reachable-space flood fill), `classic` (full-grid scan), or `rrt` (sampling-based, see below).
3. Cluster frontiers using in-node connected-component labelling (no SciPy required).
4. Score clusters via a pluggable `frontier_scorer` (`utility` default: size/distance tradeoff, `weighted`, or `nearest`) and pick the best as the next navigation goal (uses TF robot pose).
5. Send the goal to Nav2 via `NavigateToPose`.
6. Repeat until no frontiers remain.

Interactive step-through of this exact pipeline (naive frontier mask → WFD reachability
filter → clustering → safe goal placement → utility scoring), computed live against a small
illustrative grid: https://claude.ai/code/artifact/bcce2ab9-74e1-4472-9eab-50d33daacfda
(private Claude artifact — share it from the page's share menu if this doc ever needs to be
readable by someone without access to your account).

```bash
# Single command — SLAM + Nav2 + frontier explorer + auto-save
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital explore:=true

# Sampling-based (RRT) frontier detector instead of the default wfd flood fill
ros2 launch rosnav_bot slam_nav.launch.py world_name:=maze explore:=true frontier_detector:=rrt
```

**`rrt` detector** — genuinely different discovery mechanism from `wfd`/`classic` (both of which flood-fill the full occupancy grid). Grows a random tree (RRT) through known-free space from the robot's pose; any sampled step that lands on a frontier cell becomes a cluster seed instead of being extended further. Seeds are flood-filled into full clusters using the exact same code as `wfd`/`classic`, so scoring, goal-safety pullback, and BT/Nav2 handoff are unchanged — only *how frontiers are found* differs. Cheaper than `wfd` on large maps (no full-grid connected-component scan), at the cost of being probabilistic — it can miss a frontier on a given poll cycle and pick it up on the next. Tunable via `rrt_iterations` (default 300) and `rrt_step_size` (default 0.5m). Verified in sim on `maze` (`slam_nav.launch.py`): drove multiple successful goal hops and grew the map from ~1.1k to ~38k free cells with no stalls.

Recent reliability fixes in `frontier_explorer.py`:
- Removed SciPy runtime dependency (frontier adjacency + clustering implemented with NumPy + BFS).
- Added configurable `base_frame` and `goal_frame` TF lookup (`map -> base_link` by default).
- Added `min_goal_distance` filter so very near centroids are skipped (prevents no-op goals).
- If TF is not ready yet, the node waits instead of using a fake `(0, 0)` position.
- Added `map_save_path` parameter so completed exploration auto-saves map via `map_saver_cli`.
- Auto-save passes `map_saver_cli -t <map_topic>` so multi-SLAM (`/map_merged`) and scan-fusion (`/map_fused`) maps save correctly — not only `/map`.

**Exploration boundary + session resume (2026-08-24)** — added after surveying
`suchetanrs/roadmap-explorer` for ideas (concepts only, no code copied; see
`~/inspiration/roadmap-explorer` and the `project_exploration_inspiration` memory):
- `exploration_boundary` param: flat `"x1,y1,x2,y2,..."` polygon (map frame) via
  `slam_nav.launch.py`'s `exploration_boundary:=` arg. Frontier goals outside the polygon
  are rejected before scoring (ray-casting point-in-polygon test, `_in_boundary()`). Empty
  (default) = unbounded, unchanged behavior. **rclpy gotcha hit while wiring this up**: an
  *empty* `double_array` parameter — as either the node's own declared default or a launch
  override — can't be type-inferred and raises `InvalidParameterTypeException` (collapses to
  `BYTE_ARRAY` instead). Fixed by declaring a non-empty `[0.0, 0.0]` default (which
  `_parse_boundary()` already treats as "disabled", needing ≥3 points) and only including the
  `exploration_boundary` key in the launch `parameters=[...]` dict when a real boundary was
  configured, otherwise omitting it entirely so the node's own default applies.
- Session checkpoint (`_save_session_checkpoint`/`_load_session`): the visited-frontier list
  is written as JSON to `<map_prefix>_session.json` every 10 goals and on finish (same cadence
  as the existing progressive map autosave — this file is written by default whenever
  `map_prefix` resolves, same as the map itself already was). `resume_session:=true` loads it
  at startup instead of starting fresh — useful for continuing exploration after a restart
  without re-walking already-covered ground. Doesn't persist `_failed_frontiers` (time-based
  cooldown state isn't meaningful across a restart).
- Both features + `_parse_boundary`/`_in_boundary`/session save-load are covered by
  `test/test_frontier_explorer.py` (pytest, wired via `ament_add_pytest_test` — `colcon test
  --packages-select rosnav_bot`). First real Python test coverage in this package.
- **`_save_session_checkpoint()` bug**: it was defined but never actually called from
  `_explore()`/`_finish_exploration()` on first pass — the unit test only exercised the method
  directly, so it passed despite the dead wiring. Caught by a live headless run deliberately
  run long enough to cross the `iteration % 10 == 0` threshold (checkpoint file only appears
  every 10 goals or on finish). Fixed by adding the missing call sites. Both save and resume
  verified live afterward (`resume_session:=true` logged `Resumed session from ...: N
  previously-visited frontier(s) loaded.`).

**FOV-aware information gain (2026-08-24, `info_gain_mode:=fov`)** — a second idea adapted
from roadmap-explorer's `CountBasedGain` plugin (own implementation, not copied). Default
stays `info_gain_mode:=ring` (unchanged fixed-radius unknown-cell count around the whole
cluster) so the recorded `explorer:=` backend comparison numbers in this section stay valid.
`fov` mode (`_info_gain_fov_cast()`) instead casts a depth/angle-limited sensor cone
(`info_gain_fov` radians wide, default ~60°; `info_gain_max_depth` meters, default 2.0) from
the candidate goal cell along the heading the robot would actually approach from (robot→goal),
stopping each ray at the first occupied cell (occlusion) — a more physically grounded estimate
of what a camera/lidar would reveal from that vantage point than an omnidirectional ring,
particularly useful with `frontier_scorer:=weighted` (the only scorer that uses `info_gain`
directly in its score formula). Verified live (`house.world`, `frontier_scorer:=weighted
info_gain_mode:=fov`): non-zero `info=0.85m²`/`info=0.59m²` values correctly feeding the
weighted score, no crashes, normal exploration progress. Covered by 4 pytest cases (forward
cell counted, behind-heading cell excluded, occlusion blocks a ray, heading direction respected).

### Exploration backend choice (`explorer:=`)

`slam_nav.launch.py` can run one of 4 exploration backends via `explorer:=` (`_common.py`
`resolve_explorer`/`explorer_nodes`): `builtin` (rosnav's own `frontier_explorer.py`
above), `explore_lite` (m-explore-ros2), `frontier` (frontier_exploration_ros2, MRTSP),
`rrt` (rrt_explore). **Default is `explore_lite`** (changed 2026-08-23 from `builtin`).

Headless single-robot comparison on `house.world` (2026-08-23, `explorer:=<backend>`,
identical spawn point):

| backend | coverage | outcome |
|---|---|---|
| `builtin` | 58% | robot got wedged on a recovery-behavior collision loop (`Pose Goes Off Grid` on both `backup` and `spin`); needed a manual map save + kill |
| **`explore_lite`** | 22% (house), 36.5% (office) | **finished cleanly on its own both times, no intervention** |
| `frontier` (MRTSP) | 29% | got stuck retrying an unreachable frontier cluster indefinitely — no blacklist/give-up logic like `builtin`'s |
| `rrt` (rrt_explore) | 0% | **does not run at all in single-robot mode** — `rrt.cpp`'s `get_ros_parameters()` tries to parse a robot ID out of the node namespace assuming a `robotN/` prefix; with no namespace (single robot) this throws, the function returns `false`, and the constructor bails before setting up any timers/subscriptions (`rrt.cpp:45`). The node stays alive but never explores. Only usable multi-robot, namespaced. |

`explore_lite` was picked as the default because it's the only backend that reliably
finishes an unattended/headless run — it explores less per run than `builtin`, but doesn't
require a human to notice and rescue a wedged robot. `multi_robot.launch.py`'s `explorer:=`
default is unchanged (`builtin`) since this comparison only covered single-robot mode.

**No backend wins on every world.** On `warehouse.world`, `explore_lite` hit a different
failure mode than on house/office: it picked a frontier within `nav2_params.yaml`'s
`xy_goal_tolerance` (0.25m) of the robot, `controller_server` declared the goal reached
instantly with zero actual movement, and it looped on an equally-close next goal forever
(1480+ iterations, 0% progress). `builtin` on the same world got to 46.3% before wedging near
a shelf — clearly better there. If a new world stalls immediately under the default
`explore_lite`, try `explorer:=builtin` before assuming the world itself is broken.

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

### Architecture — Multi-SLAM mode (`explore:=true`, `slam_mode:=multi`)
Experimental. Every robot runs its own namespaced `slam_toolbox` (`/{ns}/map`).
`map_merge_known.py` stitches those grids into `/map_merged` using **known spawn
offsets** (not unknown-pose map matching). Nav2 + `frontier_coordinator` consume
`/map_merged`. There is no shared `/map` topic in this mode.

Independent `slam_toolbox` instances drift apart over a long run, so trusting the
static spawn offset for the whole run eventually ghosts/doubles-up overlapping
regions of `/map_merged`. `collab_loop_closure.py` (default on, `collab_loop_closure:=false`
to disable) corrects this: for every robot pair with overlapping map bounding
boxes, it brute-force searches a small (dx, dy, dtheta) window and scores each
candidate by occupied-cell agreement between the two grids — the same family of
algorithm Cartographer uses for loop closure, applied natively to the
`OccupancyGrid`s `slam_toolbox` already publishes. Accepted corrections (enough
overlap + high enough match ratio) are EMA-smoothed and published as JSON on
`/collab/poses`; `map_merge_known` prefers those over its static `init_poses_json`
once available. robot1 is always the fixed global reference, never corrected.

This is a lightweight, 2D-native equivalent of the idea behind two open-source
collaborative-SLAM frameworks — [Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM)'s
sparse inter-robot loop closure and [Multi-Robot-Graph-SLAM](https://github.com/aserbremen/Multi-Robot-Graph-SLAM)'s
graph sharing between per-robot SLAM instances — not a vendored port of either:
both are 3D point-cloud/PCL/GICP pipelines built for Velodyne-class lidars. With
`slam_mode:=multi` + default `slam_algo:=2d`, each robot still runs `slam_toolbox` on
2D `LaserScan` — and when `lidar_type:=3d`, that scan is the projected `/scan` from
`pointcloud_to_scan`, not the raw cloud. For true 3D multi-SLAM use
`slam_mode:=multi lidar_type:=3d slam_algo:=3d` (per-robot RTAB-Map on `/points`).

```
robot1 slam_toolbox ──► /robot1/map ─┬────────────────────────────┐
robot2 slam_toolbox ──► /robot2/map ─┼─► collab_loop_closure ─►/collab/poses
robotN slam_toolbox ──► /robotN/map ─┘         │                   │
                                                ▼                   ▼
                                          map_merge_known ──► /map_merged
                                                │
                                                ▼
                               Nav2 + frontier_coordinator
```

```bash
ros2 launch rosnav_bot multi_robot.launch.py slam_mode:=multi
# Disable inter-robot loop closure, static known-pose merge only:
ros2 launch rosnav_bot multi_robot.launch.py slam_mode:=multi collab_loop_closure:=false
# Save the merged map (frontier auto-save already uses -t /map_merged):
ros2 run nav2_map_server map_saver_cli -t /map_merged -f src/rosnav_bot/maps/map_hospital
ros2 run rosnav_bot fleet_manager.py savemap src/rosnav_bot/maps/map_hospital
```

### Scalability — adding more robots
No file editing needed. The fleet is generated from `robot_count` + `robot_layout`
(`line`/`grid`/`circle`), or from an exact `robots_json` list for custom poses.
Spawn validation (on by default) relocates any robot whose generated/given pose
falls inside a wall, other robot, or outside a `zone_*` spawn area.

```bash
ros2 launch rosnav_bot multi_robot.launch.py robot_count:=4 robot_layout:=grid
ros2 launch rosnav_bot multi_robot.launch.py \
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
# SLAM + frontier exploration in cafe (default)
ros2 launch rosnav_bot multi_robot.launch.py

# Pre-built map mode
ros2 launch rosnav_bot multi_robot.launch.py explore:=false

# Different world
ros2 launch rosnav_bot multi_robot.launch.py world:=obstacles explore:=false

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
ros2 launch rosnav_bot multi_robot.launch.py drive_type:=mecanum
ros2 launch rosnav_bot multi_robot.launch.py drive_type:=ackermann
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
ros2 launch rosnav_bot multi_robot.launch.py fleet_mgmt:=true
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
| `rmf_fleet_adapter.py` | this package | Thin entry point → `rosnav_bot.rmf.adapter.main()`. Registers `robot1..robotN` as one RMF fleet; bridges RMF path commands to each robot's existing `/{ns}/navigate_to_pose` |
| `rmf_submit_task.py` | this package | CLI to submit a patrol task over `/task_api_requests` |

The adapter logic lives in the `rosnav_bot/rmf/` Python package, not in the script itself:

| Module | Role |
|---|---|
| `config.py` | Loads `config/rmf_fleet.yaml` (+ the `locations.yaml` it references) into an `RmfFleetConfig` dataclass |
| `graph_builder.py` | Builds the RMF nav graph from `locations` + `lanes` in the config — no hardcoded waypoint table |
| `robot_command.py` | `Nav2RobotCommand` — the `RobotCommandHandle` implementation bridging RMF to Nav2 |
| `docking.py` | Pluggable `DockingPlugin` interface (`aruco`, `noop`, or `module.path:ClassName`) |
| `adapter.py` | Assembles the above and runs the adapter (`run_adapter()` / `main()`) |

Architecture:
- Everything customizable lives in `config/rmf_fleet.yaml` (+ `config/locations.yaml`) — no Python edits needed for a new map's rooms/corridors, lane topology, vehicle limits, battery placeholders, or task-acceptance policy.
- The nav graph is built at startup from `config/locations.yaml` (one RMF waypoint per named location) connected per the `lanes:` list in `rmf_fleet.yaml` (star topology through `hallway` by default).
- `Nav2RobotCommand` (one per robot namespace) turns RMF's `follow_new_path` waypoint sequence into ordinary `NavigateToPose` action calls against that robot's Nav2 stack, and reports live position back to RMF from `/{ns}/amcl_pose`.
- Because the graph is anchored to fixed map-frame coordinates, this only works with `multi_robot.launch.py explore:=false` (static map + AMCL) — SLAM/explore mode has no fixed map for the graph to sit on.
- Battery/task-planner parameters come from `rmf_fleet.yaml`'s `battery:` section — there is no real battery telemetry in this sim yet, so `account_for_drain` is off by default.
- `dock()` hands off to the configured `docking.plugin` (`aruco` by default → `aruco_dock.py`, or `noop` for bring-up; can be overridden with `--docking` or a custom `module.path:ClassName`). Per-dock marker/staging/charge settings live in `config/docks.yaml`. Visual docking retries with soft reverse and Nav2 restage; `stop()` / interrupt cancel the dock subprocess.
- `rmf_fleet.launch.py` accepts `rmf_config:=/path/to/my_rmf_fleet.yaml` and `docking:=noop|aruco|module:Class` to override the config file's own settings without editing it.
- For `drive_type:=diff` (the default), `multi_robot.launch.py` also brings up Nav2's own `opennav_docking` `docking_server` per robot (its own small `lifecycle_manager_docking`), configured via the `docking_server:` section in `nav2_multirobot_params.yaml`. Its `SimpleChargingDock` plugin consumes the same `/{ns}/detected_dock_pose` that `aruco_dock.py` publishes — the two docking paths (RMF's `ArucoDockingPlugin` subprocess flow and Nav2's native `docking_server` action) share one visual-detection source but are otherwise independent. mecanum/ackermann have no tuned `docking_server:` section, so it's skipped for those drive types.

Run:

```bash
# 1. Static-map fleet (RMF's graph needs fixed map-frame coordinates)
ros2 launch rosnav_bot multi_robot.launch.py explore:=false robot_count:=2

# 2. RMF traffic scheduling + task dispatch + fleet adapter
ros2 launch rosnav_bot rmf_fleet.launch.py robot_count:=2

# 3. Submit a patrol task and watch the fleet negotiate shared corridor space
ros2 run rosnav_bot rmf_submit_task.py patrol room_a room_b --rounds 2
```

`rmf_submit_task.py`'s payload shape (envelope + `task_request` fields) is verified against the upstream `open-rmf/rmf_api_msgs` JSON schemas and `open-rmf/rmf_demos`'s `dispatch_patrol.py` reference implementation — not yet exercised against a live `rmf_task_dispatcher` in this repo, but the shape itself is confirmed correct. If a live dispatcher still rejects it, `ros2 topic echo /task_api_responses` while resubmitting will show why.

---

## 12. 2D vs 3D LiDAR

| | 2D LiDAR (`lidar.xacro`) | 3D LiDAR (`lidar3d.xacro`) |
|---|---|---|
| Channels | 1 horizontal ring | 16 vertical channels |
| Bridge topic | `/scan_raw` (LaserScan) | `/points` (PointCloud2) |
| Nav2 / SLAM input | Native after filter chain → `/scan` | `pointcloud_to_scan` → `/scan_raw`, then same filter chain |
| Typical use | Navigation, SLAM | 3D mapping + projected `/scan` for Nav2 |

In this repo you do **not** hand-edit the URDF for 3D — pass `lidar_type:=3d` to
`slam_nav.launch.py` / `multi_robot.launch.py` (diff-drive only). That swaps in
`lidar3d.xacro`, the matching `gz_bridge_3d*.yaml`, and
`scripts/pointcloud_to_scan.py` (or `pointcloud_to_laserscan` if installed).

```bash
ros2 launch rosnav_bot slam_nav.launch.py lidar_type:=3d
ros2 launch rosnav_bot slam_nav.launch.py lidar_type:=3d slam_algo:=3d octomap:=true
ros2 launch rosnav_bot multi_robot.launch.py slam_mode:=multi lidar_type:=3d slam_algo:=3d
# Optional mount / FOV: lidar3d_height:=0.25 lidar3d_vfov_deg:=10
```

### Scan cleaning chain (before SLAM / Nav2)

Everything that consumes lidar for mapping or costmaps sees **`/scan`**, never the
raw Gazebo feed. Pipeline in `slam_nav.launch.py`:

```
/points (3D only) ──► pointcloud_to_scan ──► /scan_raw
                                              │
/scan_raw (2D bridge) ────────────────────────┤
                                              ▼
                                    laser_filters
                                    (range + shadows)
                                              │
                         scan_gate:=true (default)
                                              ▼
                                    /scan_pre ──► scan_quality_gate ──► /scan
                         scan_gate:=false
                                              ▼
                                            /scan
```

| Stage | Config / script | What it drops |
|---|---|---|
| `laser_filters` | `config/laser_filters.yaml` | Near-range band, beyond max range, shadow/veiling points at corners |
| `scan_quality_gate` | `scripts/scan_quality_gate.py` | **Malformed** frames: empty `frame_id`, zero/backwards stamp, broken angle/range metadata, beam-count vs angles mismatch, sudden beam-count jump, too many NaNs, too few finite in-range hits |

`laser_filters` cleans *content*; the gate rejects *structurally* bad messages so
`slam_toolbox` / Cartographer / RTAB-Map never integrate them. Disable with
`scan_gate:=false`. Monitor-only (no remapping):

```bash
ros2 run rosnav_bot scan_quality_gate.py --ros-args -p use_sim_time:=true
# logs: scans ok=… rejected=… (reasons: …)
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

### Depth camera (RGBD) / VSLAM

`rgbd_camera.xacro` mounts an RGB-D sensor on the same `camera_link` as the RGB camera
(mutually exclusive with RGB-only `camera.xacro`).

**Enable:**
- `enable_rgbd:=true` — depth for Nav2 costmaps / perception (independent of SLAM algo)
- `slam_algo:=vslam` — forces RGB-D + RTAB-Map visual SLAM (`Reg/Strategy=0`, `Grid/Sensor=1`)

Bridge configs (`gz_bridge_rgbd.yaml` / `gz_bridge_3d_rgbd.yaml`) publish:
- `/camera/image_raw` — RGB
- `/camera/depth/image_raw` — depth image (`32FC1`)
- `/camera/camera_info`
- `/camera/depth/points` — depth PointCloud2 → VoxelLayer source `depth`

RTAB-Map camera QoS is set to **Reliable** so it matches `ros_gz_bridge` publishers
(BEST_EFFORT would never match).

```bash
# Depth-aware Nav2 (keep 2D slam_toolbox) — costmaps use /camera/depth/points
ros2 launch rosnav_bot slam_nav.launch.py enable_rgbd:=true

# 3D lidar + depth camera together
ros2 launch rosnav_bot slam_nav.launch.py lidar_type:=3d slam_algo:=3d enable_rgbd:=true octomap:=true

# Visual SLAM (RGB-D registration)
ros2 launch rosnav_bot slam_nav.launch.py slam_algo:=vslam world_name:=cafe
# Localization + Nav2 (no AMCL):
ros2 launch rosnav_bot slam_nav.launch.py slam:=false slam_algo:=vslam rtabmap_db:=/path/to.db
```

Prefer textured Fuel worlds (`cafe`, `lake_house`, …) for VSLAM — see README §10b.
`slam_algo:=3d` is separate: lidar ICP + RGB bag-of-words only (RGB-only camera unless `enable_rgbd:=true`).

Full SLAM backend matrix (including **Cartographer**, **multisensor**, **cslam**, and
the headless **benchmark**) → **§3**.

Both `camera.xacro` and `rgbd_camera.xacro` render at **1280×960** (bumped from
640×480 — 4:3 kept so the tuned 90° horizontal FOV / ArUco framing doesn't
shift). `gs_capture.py`'s `image_width`/`image_height` params default to the
same values so its intrinsics stay correct.

### Overview camera (`world_monitor_cam`)

Every world file has a fixed, static third-party camera named
`world_monitor_cam`, bridged to `/world_monitor/image_raw`
(`gz_bridge.yaml`/`gz_bridge_rgbd.yaml`) and shown in `bot.rviz`'s "World
Monitor" Image display — useful for watching the robot from outside its own
sensors without opening the Gazebo GUI (works headless). `hospital.world`'s
is hand-tuned to frame the ArUco dock marker; the rest use a generic
spawn-relative placement (elevated, looking back down at the robot's start
point) that hasn't been individually verified per world — retune the
`<pose>`/`<clip><far>` in the world file if it clips geometry or misses the
interesting area.

### Mobile robot / arched camera mount
Mount the camera link at any offset from `chassis`:
```xml
<joint name="camera_joint" type="fixed">
  <parent link="chassis"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0 0.20" rpy="0 0 0"/>  <!-- front, raised -->
</joint>
```

### YOLO object detection (`yolo_detector.py`) — pluggable, off by default
Optional add-on node — nothing else in the stack depends on it, and it's disabled
unless explicitly turned on.

```bash
pip install ultralytics   # not a rosdep, install once
ros2 launch rosnav_bot slam_nav.launch.py enable_yolo:=true
# tune it:
ros2 launch rosnav_bot slam_nav.launch.py enable_yolo:=true \
  yolo_model:=yolov8n.pt yolo_confidence:=0.6 yolo_classes:=person,chair
```
Subscribes to `camera/image_raw`, runs Ultralytics YOLO inference at a fixed,
throttled rate (`max_rate_hz` param, decoupled from the camera's publish rate
so slow inference never backs up the image queue), and publishes:
- `yolo/detections` (`vision_msgs/Detection2DArray`)
- `yolo/image_annotated` (`sensor_msgs/Image`, boxes + labels drawn in)

If `ultralytics` isn't installed, the node logs the install command and exits
rather than spinning uselessly. Runs standalone too:
`ros2 run rosnav_bot yolo_detector.py --ros-args -p namespace:=robot1`.

**Testing it:** `worlds/warehouse.world` includes a few Gazebo Fuel objects that
map to real COCO classes — 2 people, a chair, a fire hydrant, a suitcase (the
rest of the world is hand-built primitives; see the world file header). gz sim
downloads and caches them on first launch (`~/.gz/fuel`, one-time, needs
internet); after that it's instant like everything else. They're spread near
the staging-zone edges and loading dock, out of the camera's 8 m range from
the *default* generic spawn — drive/explore toward them, or override
`spawn_x`/`spawn_y`/`spawn_yaw` to start closer, to get a detection quickly.

**Fine-tuning for sim assets:** stock `yolov8n.pt` (COCO) often misses low-poly
Gazebo furniture (see §29). Collect frames with `yolo_collect.py`, annotate
YOLO-format `labels/*.txt`, then `yolo_train.py` — deploy the resulting
`best.pt` via `yolo_model:=…`. Full pipeline → §35.

---

## 14. New Tool Scripts

### `fleet_manager.py` — Product-like fleet CLI
```bash
ros2 run rosnav_bot fleet_manager.py list          # list active robots
ros2 run rosnav_bot fleet_manager.py status        # map/SLAM/Nav2 (/map, /map_merged, /map_fused)
ros2 run rosnav_bot fleet_manager.py add robot3 1.0 2.0   # dynamic spawn
ros2 run rosnav_bot fleet_manager.py teleop robot1 # keyboard control
ros2 run rosnav_bot fleet_manager.py goto robot2 3.0 -1.0 # nav goal
ros2 run rosnav_bot fleet_manager.py dock robot1           # navigate to charging_dock
ros2 run rosnav_bot fleet_manager.py undock robot1         # back away 0.5 m from the dock
ros2 run rosnav_bot fleet_manager.py explore robot2        # frontier
ros2 run rosnav_bot fleet_manager.py savemap /tmp/my_map   # auto-picks /map_merged|/map_fused|/map
ros2 run rosnav_bot fleet_manager.py stop robot1           # cancel nav
```

`status` and `savemap` watch `/map`, `/map_merged`, and `/map_fused`, so multi-SLAM
and scan-fusion fleets report/save the right OccupancyGrid without remapping by hand.

### `multi_teleop.py` — Interactive multi-robot keyboard teleop
```bash
ros2 run rosnav_bot multi_teleop.py
# → shows robot list, select one, drive it, switch with R, spawn new with N
```

### `scan_quality_gate.py` — Reject malformed LaserScans before SLAM

See §12. Default mode is **monitor** (log rejects); `slam_nav` launches it in
**gate** mode between `laser_filters` (`/scan_pre`) and `/scan`.

```bash
# Monitor a live stack
ros2 run rosnav_bot scan_quality_gate.py --ros-args -p use_sim_time:=true

# Manual gate (if you remapped laser_filters → scan_pre yourself)
ros2 run rosnav_bot scan_quality_gate.py --ros-args \
    -p mode:=gate -p use_sim_time:=true \
    -r scan_in:=scan_pre -r scan_out:=scan
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

- Keep ROS package source under `src/rosnav_bot/` only.
- Generated maps live in `src/rosnav_bot/maps/` (`map_*.yaml` / `map_*.pgm`).
- Docs at repo root: `README.md` (quickstart), `concepts.md` (deep reference).
- Launch everything with `ros2 launch` / `ros2 run` — see README §2–§5.


## 17. A* Path Planner (`path_planning.py`)

Standalone global planner using the **A\* algorithm**:
- Converts the world to a discrete grid.
- Uses Euclidean distance as the heuristic.
- Supports 8-direction movement (including diagonals).
- Subscribes to `/map` (`OccupancyGrid`) and treats occupied/unknown cells as obstacles (with `safety_margin` inflation).
- If no map arrives within `map_wait_sec`, falls back to a tiny hardcoded demo obstacle set for offline smoke tests.

> Note: this is a teaching/demo planner — Nav2's `planner_server` is what the stack uses in real runs. Coverage (`coverage_planner.py`) already plans from `/map` independently of this script.

```bash
# With a live map (SLAM or map_server):
ros2 run rosnav_bot path_planning.py

# Custom start/goal (metres, map frame):
ros2 run rosnav_bot path_planning.py --ros-args \
  -p start_x:=0.0 -p start_y:=0.0 -p goal_x:=3.0 -p goal_y:=-1.0
```

---

## 17b. Navigation Platforms (AMR matrix)

Two independent launch args pick **how it moves** vs **how it looks**:

| Arg | Changes | Values |
|---|---|---|
| `drive_type` | Physics + Gazebo plugin + Nav2 params | `diff` (default) · `mecanum` · `ackermann` |
| `robot_model` | Chassis **visual** only (same footprint / wheels / Nav2) | `custom` (default) · `mir100` · `husky` |

`mir100` / `husky` only work with `drive_type:=diff`. Mixing them with mecanum/ackermann is ignored (warning in the log). **Husky is not car-like** — it is a skid-steer *look* on diff drive; car-like motion is `drive_type:=ackermann` only.

### All vehicles at a glance

| Vehicle | Launch knobs | Motion | Turn in place? | Strafe? | Nav2 | Sensors (always / optional) |
|---|---|---|---|---|---|---|
| Diff box | `drive_type:=diff` | Forward + rotate | Yes | No | DWB (Humble) or `controller:=mppi` | **2D lidar + IMU**; `lidar_type:=3d` OK; camera via `enable_camera` / `enable_rgbd` |
| MiR100 look | `robot_model:=mir100` | Same as diff | Yes | No | Same as diff | Same as diff (mesh skin only) |
| Husky look | `robot_model:=husky` | Same as diff | Yes | No | Same as diff | Same as diff (mesh skin only) |
| Mecanum | `drive_type:=mecanum` | Holonomic | Yes | Yes (`vy`) | `max_vel_y` unlocked | **2D lidar + IMU** (no 3D lidar path); camera opt-in |
| Ackermann (car-like) | `drive_type:=ackermann` | Front steer, ~0.66 m min radius | No | No | Always MPPI + Reeds-Shepp | **2D lidar + IMU** (no 3D lidar path); camera opt-in |

Shared footprint ≈ 0.55 × 0.4 m so costmaps stay consistent across skins. Lidar is on for every platform (`lidar.xacro` → `/scan` after the §12 filter chain). Camera defaults **off** in `slam_nav` (render cost); forced on for YOLO / `slam_algo:=vslam` / `enable_rgbd:=true` / `slam_algo:=3d` (RGB for RTAB loop-closure).

```bash
# Diff / skins / holonomic / car-like (SLAM or slam:=false both work)
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital drive_type:=diff
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital robot_model:=husky
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital robot_model:=mir100
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital drive_type:=mecanum
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital drive_type:=ackermann safety:=true

# Headless smoke: /clock /odom /scan /map + short drive for all five
./src/rosnav_bot/scripts/smoke_amr_matrix.sh
# WORLD=maze BOOT_WAIT_S=50 ./src/rosnav_bot/scripts/smoke_amr_matrix.sh
```

Deep dives: §18 mecanum · §18b ackermann · §19 MiR100 · §20 Husky · §12 lidar.

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
ros2 launch rosnav_bot robot.launch.py drive_type:=mecanum headless:=true rviz:=false

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
ros2 launch rosnav_bot robot.launch.py drive_type:=ackermann headless:=true rviz:=false

# Mapping by manually driving the car-like base:
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital drive_type:=ackermann

# drive forward while turning — steering angle emerges from angular.z:
ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

---

## 19. MiR100 Chassis Skin (`robot_model:=mir100`)

Swaps the chassis **visual only** — wheels, sensors (lidar/camera), physics,
and nav2 footprint/tuning are untouched. Only valid with `drive_type:=diff`
(the default).

```bash
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital robot_model:=mir100
ros2 launch rosnav_bot multi_robot.launch.py robot_model:=mir100   # fleet-wide
```

### How it's built
- `urdf/robot_core_mir100.xacro` — same as `robot_core.xacro` (identical wheel
  joints, inertials, Gazebo friction) except the chassis `<visual>` is a
  `<mesh>` (`meshes/mir100/visual/mir_100_base.stl`) instead of a `<box>`.
  `<collision>` stays the original box, so costmap/physics behavior is
  unchanged.
- `urdf/robot_mir100.urdf.xacro` — top-level file, mirrors `robot.urdf.xacro`
  but includes `robot_core_mir100.xacro`.
- `_common.urdf_filename_for(drive_type, robot_model)` picks it when
  `robot_model=mir100` (falls back to the normal diff/mecanum/ackermann
  selection otherwise, with a warning if `robot_model=mir100` is combined
  with a non-diff `drive_type`).

### Mesh vendoring
`meshes/mir100/visual/*.stl` are downloaded as static assets from
[DFKI-NI/mir_robot](https://github.com/DFKI-NI/mir_robot) (`humble` branch) —
**not** a git submodule or remote; this repo stays self-contained. BSD-3-Clause,
see `meshes/mir100/README.md` for attribution.

### Scale/placement math
The real MiR100 mesh is ~0.9 × 0.58 × 0.3 m (native STL units). Scaled by
`0.61` and placed at chassis-local origin `(0.275, 0, -0.047)`, it lands at
x:[0.001, 0.549] y:[-0.178, 0.179] z:[-0.035, 0.169] — matching this robot's
existing 0.55 × 0.4 × 0.15 footprint almost exactly (verified by loading the
STL and computing its bounding box directly), clearing the ground (wheel
radius puts ground at z=-0.075) and not overlapping the wheels (which sit at
y=±0.205 to ±0.245, outside the mesh's y-span).

---

## 20. Husky Chassis Skin (`robot_model:=husky`)

Same idea as the MiR100 skin above — chassis **visual only**, same wheels/
physics/nav2 footprint. Only valid with `drive_type:=diff`. This is **not**
ackermann/car-like motion; for that use `drive_type:=ackermann` (§18b).

```bash
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital robot_model:=husky
ros2 launch rosnav_bot multi_robot.launch.py robot_model:=husky   # fleet-wide
```

### How it's built
- `urdf/robot_core_husky.xacro` — same wheel joints/inertials/friction as
  `robot_core.xacro`; the `chassis` link's `<visual>` is the Husky hull mesh
  instead of a box (`<collision>` stays the original box). Four extra
  visual-only sibling links (`husky_top_chassis`, `husky_top_plate`,
  `husky_front_bumper`, `husky_rear_bumper`) are fixed-jointed to `chassis`
  to reassemble the rest of the Husky body panels, with joint origins scaled
  by the same factor as the mesh so the pieces still line up correctly.
- `urdf/robot_husky.urdf.xacro` — top-level file, mirrors `robot_mir100.urdf.xacro`.
- `_common.urdf_filename_for()` picks it for `robot_model=husky`, same
  diff-drive-only restriction and fallback warning as mir100.

### Mesh vendoring
`meshes/husky/visual/*.glb` are a flattened, sensor-free export of
Clearpath's `husky_description` chassis meshes. BSD-3-Clause, see
`meshes/husky/README.md`. **They were re-exported through `gltf-transform copy`
before vendoring** — the originals used `EXT_meshopt_compression` (from
`gltfpack`, a web-viewer-oriented compression scheme), which neither
RViz's Assimp nor Gazebo Sim's mesh loader can decode; both failed silently
(spawned the robot with the visual just missing) until decompressed to plain
glTF.

### Scale/placement math
Husky's own hull mesh has its local origin at floor level (matching its
`base_link` collision box's z=0 bottom edge), so it's placed at
chassis-local origin `(0.275, 0, 0)` — same x-offset convention as MiR100 —
scaled by `0.55` to fit the existing 0.55 × 0.4 × 0.15 footprint. The
accessory panel joints reuse husky_description's own relative offsets,
scaled by the same 0.55 factor and shifted by the same (0.275, 0, 0).

### Bug found and fixed along the way: `GZ_SIM_RESOURCE_PATH` missing `share/`
`with_gz_model_path()` in `_common.py` only added `share/rosnav_bot/models`
to `GZ_SIM_RESOURCE_PATH`. sdformat rewrites a URDF's
`package://rosnav_bot/meshes/...` mesh URIs to `model://rosnav_bot/meshes/...`
during URDF→SDF conversion, and gz sim could only resolve that against a
resource path *one level above* `share/rosnav_bot` (i.e. `share/` itself).
That directory was never added, so gz sim silently dropped **every**
`package://`-referenced mesh — confirmed this was already happening for the
MiR100 chassis (it spawned in Gazebo with no visual at all; RViz was fine
since it resolves `package://` directly via `resource_retriever`, a
different code path). Fixed by adding `share/` to the resource path
alongside `models/`; verified both mir100 and husky render correctly in
Gazebo after the fix.

---

## Quick Reference — Launch Files

| Launch file | What it does | Key args |
|---|---|---|
| `robot.launch.py` | Gazebo + robot + Nav2 full bringup with saved map | `map`, `world`, `robot_name`, `spawn_x/y/z/yaw`, `rviz`, `use_sim_time`, `drive_type` (`diff`\|`mecanum`\|`ackermann`) |
| `slam_nav.launch.py` | Gazebo + robot + SLAM/AMCL + Nav2 (+ optional frontier) | `world_name`, `slam` (`true`\|`false` — see §3), `explore`, `slam_algo` (`2d`\|`cartographer`\|`3d`\|`vslam`\|`multisensor`\|`cslam`), `lidar_type`, `rtabmap_db`, `octomap`, `drive_type`, `robot_model`, `controller`, `enable_camera`/`enable_rgbd`, `scan_gate`, `rviz` |
| `slam.launch.py` | Gazebo + robot + SLAM Toolbox mapping mode | `use_sim_time` |
| `multi_robot.launch.py` | Scalable N-robot fleet: SLAM+frontier or shared map + Nav2 per robot | `world`, `map`, `explore`, `slam_mode`, `lidar_type`, `slam_algo`, `enable_rgbd`, `collab_loop_closure`, `robot_count`, `robot_layout`, `robots_json`, `drive_type` (`diff`\|`mecanum`\|`ackermann`), `controller` (`dwb`\|`mppi`), `fleet_mgmt`, `rviz`, `headless` |
| `nav2.launch.py` | Nav2 only (attach to running Gazebo) | `map`, `world`, `use_sim_time` |
| `rmf_fleet.launch.py` | Open-RMF traffic scheduling on top of an already-running static-map fleet (see §11b) | `robot_count`, `robots`, `fleet_name`, `map_name`, `adapter_delay` |
| `gs_capture.launch.py` | Gazebo (headless) + the teleportable `gs_capture_rig` camera, no robot/nav stack (see §27) | `world_name` |
| `_common.py` | Not a launch file — shared helper module (world resolution, Gazebo/RSP/laser-filter/scan-quality-gate builders, nav2 params selection + multi-robot namespacing) imported by `slam_nav.launch.py` and `multi_robot.launch.py` | — |

## Quick Reference — Scripts

| Script | What it does | Key ROS params |
|---|---|---|
| `navigation.py` | Custom obstacle-avoidance FSM (no Nav2 needed) | `goal_x`, `goal_y`, `base_speed`, `obstacle_threshold` |
| `path_planning.py` | Standalone A* path planner (from `/map`, hardcoded fallback) | `map_topic`, `grid_size_x/y`, `resolution`, `safety_margin`, `start_x/y`, `goal_x/y` |
| `waypoint_nav.py` | Navigate through a sequence of waypoints via Nav2 | `waypoints_file`, `frame_id` |
| `frontier_explorer.py` | Autonomous map exploration via frontier detection + optional auto-save | `frontier_detector` (`wfd`\|`classic`\|`rrt`), `frontier_scorer`, `min_frontier_size`, `revisit_radius`, `poll_period`, `map_save_path`, `exploration_boundary`, `resume_session`, `session_state_path`, `info_gain_mode` (`ring`\|`fov`), `info_gain_fov`, `info_gain_max_depth` |
| `benchmark.py` | SLAM coverage / Nav2 goal / AMCL covariance benchmarks + offline `mode:=report` | `mode`, `label`, `duration_sec`, `out_dir`, `goals_file`, `inputs` |
| `benchmark_slam.sh` | Headless matrix over `slam_algo` values; wraps `benchmark.py` | env: `WORLD`, `DURATION_S`, `ALGOS`, `OUT_DIR` |
| `check_odometry.py` | Debug odometry data | — |
| `scan_quality_gate.py` | Find / reject malformed LaserScans before SLAM (see §12) | `mode` (`monitor`\|`gate`), `scan_in`, `scan_out`, `min_valid_ratio`, `min_valid_beams`, `max_nan_ratio` |
| `smoke_amr_matrix.sh` | Headless smoke of all five AMRs (diff/mecanum/ackermann/mir100/husky): topics + short drive (see §17b) | env: `WORLD`, `BOOT_WAIT_S`, `DRIVE_S`, `LOG_DIR` |
| `reset_pose.py` | Reset robot pose in simulation | `world_name`, `robot_name`, `reset_x/y/z/yaw` |
| `rmf_fleet_adapter.py` | Registers the fleet with Open-RMF, bridges RMF path commands to Nav2 (see §11b) | `--fleet-name`, `--map-name`, `--robots` |
| `rmf_submit_task.py` | Submits a patrol task to `rmf_task_dispatcher` to exercise traffic scheduling | `category`, `places`, `--rounds`, `--wait` |
| `obstacle_tracker.py` | Detects + tracks moving obstacles from `/scan`, with ellipse extent estimation (see §25) | `min_speed`, `cluster_radius`, `track_gate_dist`, `min_extent`, `extent_gain` |
| `pointcloud_to_scan.py` | Projects 3D `/points` → `/scan_raw` when `lidar_type:=3d` (fallback if `pointcloud_to_laserscan` not installed; see §12) | `min_height`, `max_height`, `range_min/max` |
| `dynamic_obstacle_driver.py` | Patrols a spawned `dynamic_obstacle` model back and forth (see §26) | `obstacle_name`, `axis`, `amplitude`, `speed` |
| `gs_capture.py` | Sweeps the `gs_capture_rig` camera through a waypoint grid, saves nerfstudio-format capture data (see §27) | `world_name`, `out_dir`, `centers_x/y`, `heights`, `yaw_steps`, `pitch_deg` |
| `gs_splat_to_pointcloud.py` | Not a ROS node — converts a `ns-export gaussian-splat` `.ply` to `(xyz, rgb, opacity)` `.npz` (see §27) | `argv`: ply path, out path, opacity thresh |
| `gs_view_pointcloud.py` | Publishes a converted splat `.npz` as a latched `PointCloud2` for viewing in RViz (see §27) | `npz_path`, `topic`, `frame_id` |
| `gs_mask_from_splat.py` | Not a ROS node — rasterizes a converted splat `.npz` into a Nav2 costmap-filter-mask (KeepoutFilter) PGM+YAML (see §28) | `argv`: `--npz`, `--out`, `--z-min/max`, `--dilate`, `--align-to` |
| `gs_train_keepout.sh` | Orchestrates capture → `ns-train` → export → keepout mask (see §28 / §35); `MASK_ONLY=1` skips train | env: `DATA_DIR`, `NPZ`, `OUT_MASK`, `MAX_ITERS`, `SKIP_CAPTURE` |
| `yolo_collect.py` | Saves RGB frames (+ empty labels) for YOLO fine-tuning (see §35) | `out_dir`, `max_frames`, `classes`, `split` |
| `yolo_train.py` | Not a ROS node — fine-tunes Ultralytics YOLO on `dataset.yaml` (see §35); `--smoke` for a synthetic 1-epoch dry run | `argv`: `--data`, `--model`, `--epochs`, `--smoke` |
| `train_ppo.py` | Not a ROS node — offline PPO on `ScanNavEnv` (map PGM raycast, no Gazebo; see §35) | `argv`: `--map`, `--timesteps`, `--out`, `--backend`, `--smoke` |
| `rl_policy_node.py` | Research `/cmd_vel` from a trained ScanNav policy (`.pt` or SB3 `.zip`; see §35) | `model_path`, `goal_x/y`, `scan_topic`, `odom_topic` |

---

## How to Source and Run

```bash
# Every new terminal needs this
source /opt/ros/humble/setup.bash           # or jazzy
source ~/rosnav/install/setup.bash

# Build after any changes
cd ~/rosnav
colcon build --symlink-install --packages-select rosnav_bot
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
| Map | `/map` | Map is loaded and being published (single-SLAM / map_server) |
| Map | `/map_merged` | Multi-SLAM merge live (`slam_mode:=multi`) |
| Map | `/map_fused` | Scan-fusion layer live (`merge_scans:=true`) |
| RobotModel | — | URDF loaded, TF tree working |
| LaserScan | `/scan` | 2D lidar, or projected scan when `lidar_type:=3d` |
| PointCloud2 | `/points` | 3D lidar cloud (`lidar_type:=3d`) |
| PointCloud2 | `/camera/depth/points` | RGB-D depth cloud (`enable_rgbd:=true`) |
| PointCloud2 | `/cloud_map` | RTAB-Map assembled cloud (`slam_algo:=3d`\|`vslam`) |
| PointCloud2 | `/octomap_occupied_space` | OctoMap voxels (`octomap:=true`) |
| Image | `/camera/image_raw` | RGB / RGB-D color |
| Image | `/camera/depth/image_raw` | Depth image (`enable_rgbd` / `vslam`) |
| Pose | `/amcl_pose` | AMCL is localising the robot (only in `robot.launch.py`) |
| Path | `/plan` | Nav2 planner computed a path |
| Map | `/local_costmap/costmap` | Local obstacle avoidance active |
| MarkerArray | `/explore/frontiers` | Frontier candidates (`explorer:=explore_lite`\|`frontier`, `visualize`/`frontier_marker_topic`) |
| MarkerArray | `/exploration/frontiers` | Frontier debug markers (`explorer:=builtin`, multi-robot `frontier_coordinator` only — single-robot `frontier_explorer.py` does not publish markers) |

`bot.rviz` already includes Lidar3D, DepthCloud, CloudMap, Depth, and OctomapOccupied displays.

### RViz config per mode (`slam_nav.launch.py`)

`slam_nav.launch.py` no longer always opens `bot.rviz` — it picks a config from
`src/rosnav_bot/rviz/` based on the resolved mode:

| Condition | Config | Notes |
|---|---|---|
| `slam_algo` != `2d` (cartographer/3d/vslam/multisensor/cslam) | `vslam.rviz` | Camera/Depth images + point clouds on, no SlamToolboxPlugin panel |
| `slam:=true` (default, `slam_algo:=2d`) | `slam_explore.rviz` | SlamToolboxPlugin panel for start/stop/save-map controls, no Navigation 2 panel (see segfault note below); LaserScan, Global+Local Costmap, footprint, Global+Local Plan, and `/explore/frontiers` markers all on by default |
| `slam:=false` + camera/RGB-D enabled | `cam_nav.rviz` | Navigation 2 panel + Camera/Depth images, no SlamToolboxPlugin |
| `slam:=false`, no camera | `localization.rviz` | Navigation 2 panel, lidar-only (images/point clouds off) |
| `robot.launch.py` (generic) / `multi_robot.launch.py` single-robot mode | `bot.rviz` / `multi_robot.rviz` | "one for all" fallback — Navigation 2 panel, everything else on |

**Why split it:** rviz2's Ogre1 GL backend reliably segfaults on startup on
this box (`RenderWindowImpl::resize` → `XFree()`, confirmed with gdb) whenever
a config docks **both** the SlamToolboxPlugin panel and the Navigation 2 panel
at once — the dock-layout resize storm during startup races the GL context.
No `.rviz` file in this repo may combine them; `bot.rviz` and
`multi_robot.rviz` had this bug and were fixed by dropping SlamToolboxPlugin.
Also fixed: the LaserScan display's Topic QoS was `Reliability Policy:
Reliable`, but `/scan` (and `/robotN/scan`) publish Best Effort — RViz was
silently dropping the incompatible subscription and never drawing the scan.

Set **Fixed Frame** to `map` in RViz Global Options.

---

## Checking SLAM / Localization from Terminal

```bash
# Which mapper is running?
ros2 node list | grep -Ei 'slam_toolbox|cartographer|rtabmap'

# Is the map being published? (width/height should grow while mapping)
ros2 topic echo /map --once --field info

# IMU (Cartographer) / depth (vslam|multisensor)
ros2 topic hz /imu
ros2 topic hz /camera/depth/image_raw

# Is AMCL running and localising? (saved-map mode only — not live SLAM)
ros2 topic echo /amcl_pose

# Is the TF tree complete? (map → odom → base_link → laser_frame [/ imu_link])
ros2 run tf2_tools view_frames   # saves frames.pdf

# Is Nav2 active?
ros2 node list | grep -E "amcl|planner|controller|bt_navigator"

# Is the robot moving? (should show non-zero during navigation)
ros2 topic hz /cmd_vel_safe

# Are LaserScans clean enough for SLAM? (scan_gate is on by default in slam_nav)
ros2 topic hz /scan
ros2 run rosnav_bot scan_quality_gate.py --ros-args -p use_sim_time:=true
# expect rejected≈0%; non-zero reasons → empty_frame_id / too_few_valid / …

# Save map after SLAM mapping (world-aware naming)
ros2 run nav2_map_server map_saver_cli -f src/rosnav_bot/maps/map_hospital
ros2 run nav2_map_server map_saver_cli -f src/rosnav_bot/maps/map_obstacles

# Multi-SLAM / scan-fusion: pick the merged topic explicitly
ros2 run nav2_map_server map_saver_cli -t /map_merged -f src/rosnav_bot/maps/map_hospital
ros2 run rosnav_bot fleet_manager.py savemap src/rosnav_bot/maps/map_hospital  # auto-picks topic

# Compare SLAM backends headless (see §3 Benchmarking)
./src/rosnav_bot/scripts/benchmark_slam.sh

# Load custom waypoints
ros2 run rosnav_bot waypoint_nav.py --ros-args \
    -p waypoints_file:=~/rosnav/waypoints.yaml

# Run frontier exploration (slam_nav.launch.py must be active)
ros2 run rosnav_bot frontier_explorer.py

# Reset robot to origin
ros2 run rosnav_bot reset_pose.py --ros-args \
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
ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital safety:=true

# Or run standalone:
ros2 run rosnav_bot collision_monitor.py --ros-args \
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
ros2 run rosnav_bot mission_server.py

# Patrol robot1 through 3 waypoints:
ros2 run rosnav_bot mission_server.py patrol robot1 1,2,0 3,4,90 0,0,180

# Single goal:
ros2 run rosnav_bot mission_server.py goto robot1 3.0 -1.0 45

# Check all robots:
ros2 run rosnav_bot mission_server.py status

# Check just one robot:
ros2 run rosnav_bot mission_server.py status robot1

# Cancel:
ros2 run rosnav_bot mission_server.py cancel

# Via fleet_manager:
ros2 run rosnav_bot fleet_manager.py mission robot1 patrol 1,2,0 3,4,90
ros2 run rosnav_bot fleet_manager.py mission robot1 status
ros2 run rosnav_bot fleet_manager.py collision robot1
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

It is already **map-driven**: it subscribes to `/map` (`OccupancyGrid`), treats `grid == 0` as free, erodes by `robot_radius`, then sweeps. Hardcoded obstacles belong to the standalone A* demo (`path_planning.py`), not coverage.

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
ros2 run rosnav_bot coverage_planner.py

# Tighter sweep (warehouse):
ros2 run rosnav_bot coverage_planner.py --ros-args -p sweep_spacing:=0.4

# Multi-robot:
ros2 run rosnav_bot coverage_planner.py --ros-args -p robot_ns:=robot2
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
ros2 run rosnav_bot task_allocator.py

# Add tasks:
ros2 run rosnav_bot task_allocator.py add 2.0 1.5 0 pickup_A
ros2 run rosnav_bot task_allocator.py add 4.0 -1.0 90 dock_B
ros2 run rosnav_bot task_allocator.py add 0.0 3.0 180

# Monitor:
ros2 run rosnav_bot task_allocator.py status

# Or via fleet_manager:
ros2 run rosnav_bot fleet_manager.py tasks add 2.0 1.5 0 pickup_A
ros2 run rosnav_bot fleet_manager.py tasks status
ros2 run rosnav_bot fleet_manager.py tasks clear
```

---

## 25. Moving Obstacle Tracking (`obstacle_tracker.py`)

`scripts/obstacle_tracker.py` detects and tracks moving obstacles from `/scan` alone (no separate perception sensor) — useful for spotting other robots/people crossing a robot's path that the local costmap only sees as static occupied cells.

**Algorithm:**
1. Keep a rolling buffer of recent `LaserScan` messages.
2. Compare each ray's range now vs. `lookback` frames ago — a ray "closing" faster than `min_speed` marks an approaching surface.
3. Transform closing-ray endpoints to the map frame via TF, then single-linkage cluster them within `cluster_radius`.
4. Fit an **ellipse** to each cluster's point covariance (major/minor axis lengths + orientation) — this is the object's *extent*, distinct from its position.
5. Feed cluster centroids into a constant-velocity Kalman filter per track (position + velocity), while extent is smoothed separately via an exponential moving average (`extent_gain`), since the Kalman filter only models position/velocity.
6. Greedy nearest-centroid association gates clusters to existing tracks (`track_gate_dist`); unmatched clusters spawn new tracks; tracks with too many consecutive misses (`track_max_misses`) are dropped.

This is **extended object tracking**: unlike a plain centroid tracker, a track carries a size and heading estimate, not just a point. Ellipse orientation is only defined mod π (an ellipse looks the same rotated 180°), so smoothing resolves that ambiguity before blending — otherwise the angle would flip back and forth between adjacent frames.

**Published:**

| Topic | Content |
|---|---|
| `/obstacle_tracker/markers` | MarkerArray — CYLINDER per track sized to its ellipse extent (`scale.x/y` = major/minor axis, orientation = fitted angle), velocity arrow, id/speed label |
| `/obstacle_tracker/state` | `std_msgs/String` JSON: `id, x, y, vx, vy, speed, points, length, width, orientation` per track |

```bash
ros2 run rosnav_bot obstacle_tracker.py
ros2 run rosnav_bot obstacle_tracker.py --ros-args -p robot_ns:=robot1 -p min_speed:=0.05
ros2 topic echo /obstacle_tracker/state
```

## 26. Dynamic Obstacles (`dynamic_obstacle_driver.py`)

Prior to this, every world was static — nothing to actually feed [[obstacle_tracker.py]] (§25) or Nav2's dynamic-obstacle avoidance during a real run. `slam_nav.launch.py` and `multi_robot.launch.py` can now spawn one or more patrolling `dynamic_obstacle` models via the `dynamic_obstacles` launch arg (default `0`, opt-in).

**How it works:**
1. `models/dynamic_obstacle/model.sdf` — a 0.25m-radius, 0.6m-tall cylinder with the Gazebo `VelocityControl` plugin (`ignition::gazebo::systems::VelocityControl`), which sets the link's velocity directly each physics step (also cancelling gravity, so the obstacle doesn't fall).
2. `_common.spawn_dynamic_obstacle_node()` spawns it via `ros_gz_sim create -file` (unlike robots, it isn't spawned from a `robot_description` topic).
3. `_common.dynamic_obstacle_bridge_node()` bridges ROS `geometry_msgs/Twist` → Gazebo `gz.msgs.Twist` on `/model/<name>/cmd_vel`.
4. `scripts/dynamic_obstacle_driver.py` publishes to that bridge, oscillating the obstacle back and forth between `+-dynamic_obstacle_amplitude` metres along `dynamic_obstacle_axis`, reversing direction at each end (logged each time).

Each obstacle is named `dynamic_obstacle_<n>`, spaced 2m apart starting at `(dynamic_obstacle_x, dynamic_obstacle_y)`.

**Gotcha:** the `dynamic_obstacle_axis` arg takes `x_axis`/`y_axis`, not bare `x`/`y` — a ROS 2 `--params-file` is YAML 1.1, which parses unquoted `y`/`n` as booleans, so a bare `y` silently becomes the boolean `True` instead of the string `"y"`.

```bash
# Single robot, one obstacle patrolling +-3m along y in the maze
ros2 launch rosnav_bot slam_nav.launch.py world_name:=maze dynamic_obstacles:=1

# Multi-robot SLAM, two obstacles patrolling along x at 0.6 m/s
ros2 launch rosnav_bot multi_robot.launch.py dynamic_obstacles:=2 \
    dynamic_obstacle_axis:=x_axis dynamic_obstacle_speed:=0.6

# Watch obstacle_tracker.py pick it up
ros2 run rosnav_bot obstacle_tracker.py
ros2 topic echo /obstacle_tracker/state
```

## 27. Gaussian Splatting Capture Rig (`gs_capture.py`)

Feasibility-spike tool for turning any world into a [3D Gaussian Splatting](https://docs.nerf.studio/nerfology/methods/splat.html) training set — no robot, no COLMAP structure-from-motion needed, since poses come straight from Gazebo ground truth.

**How it works:**
1. `models/gs_capture_rig/model.sdf` — a static, geometry-free camera-only model (no visual/collision mesh, so it can't occlude the scene or itself), spawned standalone via `gs_capture.launch.py` (Gazebo server headless + rig spawn + a dedicated `gs_capture_bridge.yaml` bridge — separate from the main nav/robot bridge since a capture run has no robot, lidar, or `cmd_vel`).
2. `scripts/gs_capture.py` sweeps a grid of `(x, y, z, yaw, pitch)` waypoints (positions × heights × pitches × a full yaw sweep at each), teleporting the rig via the `/world/<world>/set_pose` Gazebo service — physics doesn't fight the placement since the model is `static`.
3. After each teleport it waits `SETTLE_TIME_S` (0.35s) for the renderer to catch up, then waits for a *fresh* frame (timestamped after the teleport, not a stale one already in flight) before saving. Near-uniform (low-stddev) frames get dropped — usually means the rig teleported inside geometry.
4. Saves `<out_dir>/images/frame_%05d.png` + `<out_dir>/transforms.json` in nerfstudio-data format (`OPENCV` camera model, per-frame 4×4 pose matrices), ready for `ns-train splatfacto --data <out_dir>` in a nerfstudio venv.

**Gotcha:** Gazebo's camera-sensor local frame is X-forward/Y-left/Z-up (confirmed empirically — identity orientation renders looking down +X, not +Z, i.e. NOT the usual Z-forward optical convention). `gs_capture.py` computes rig orientation directly in that body frame, then remaps to nerfstudio's OpenGL/NeRF convention (X-right, Y-up, Z-back) only when writing `transforms.json`.

```bash
# Bring up Gazebo + the rig (headless, no robot/nav stack)
ros2 launch rosnav_bot gs_capture.launch.py world_name:=cafe

# Run the capture sweep (default grid tuned to cafe.world's furniture layout)
ros2 run rosnav_bot gs_capture.py --ros-args \
    -p world_name:=cafe -p out_dir:=/home/asimov/gs_data/cafe

# Train (separate nerfstudio venv, e.g. ~/venvs/nerfstudio)
source ~/venvs/nerfstudio/bin/activate
ns-train splatfacto --data /home/asimov/gs_data/cafe --pipeline.model.random-init True nerfstudio-data

# View the trained splat
ns-viewer --load-config outputs/.../splatfacto/<timestamp>/config.yml   # → http://localhost:7007
```

`--pipeline.model.random-init True` is needed because this pipeline has no COLMAP sparse point cloud to seed the Gaussians from — splatfacto falls back to random initialization instead.

**Viewing in RViz instead of the nerfstudio browser viewer:** `ns-viewer`'s live web viewer competes with an in-progress `ns-train` for the same GPU, so it can get stuck on a broken preview while training is still running (fine once training is finished, though). As an alternative — RViz has no Gaussian-Splat renderer, but a splat's Gaussian centers can be shown as a colored `PointCloud2`, which RViz displays natively:
1. `scripts/gs_splat_to_pointcloud.py` (run inside the nerfstudio venv, needs `plyfile`) — reads a `ns-export gaussian-splat` `.ply`, converts each Gaussian's spherical-harmonics DC color term to RGB (`0.5 + 0.28209479177387814 * f_dc`), filters by opacity, writes `(xyz, rgb, opacity)` to a `.npz`. **Pass `--dataparser-transform <dataparser_transforms.json>`** — see §29b, this is required to get true-world-meter coordinates.
2. `scripts/gs_view_pointcloud.py` — a ROS 2 node (run in the normal ROS environment, not the nerfstudio venv) that loads that `.npz` and publishes it as a latched (`TRANSIENT_LOCAL`) `sensor_msgs/PointCloud2` on `/gs_capture/splat_points`, RGB packed PCL-style (uint8 r/g/b packed into the float32 `rgb` field).
3. `rviz/gs_capture.rviz` — minimal, Fixed Frame `world`, just the capture-rig `Image` feed + splat `PointCloud2`. `rviz/gs_overview.rviz` (§29) is the fuller config — Fixed Frame `map`, adds SLAM map, keepout/speed costmap, robot model, 3D lidar, semantic markers, and camera alongside the splat cloud, so the splat can be viewed against a live running sim instead of standalone.

This shows the reconstruction as discrete colored dots (no soft Gaussian blending/alpha), not photoreal like the nerfstudio viewer — good enough to sanity-check shape and color at a glance without leaving RViz.

## 28. GS Costmap Keepout Filter (`gs_mask_from_splat.py`)

First real Nav2 integration for the Gaussian-Splatting spike (§27) — turns a trained splat into a Nav2 **KeepoutFilter** costmap layer, the "GS as costmap layer" path from the GS-navigation integration plan: enrich free-space, not the planner/controller. `planner_server`/`controller_server`/MPPI/DWB are untouched; GS only ever contributes cost, never `cmd_vel`.

**Pipeline:** `splat.ply` (nerfstudio venv) → `gs_splat_to_pointcloud.py` → `.npz` → `gs_mask_from_splat.py` (plain ROS env, no venv) → Nav2 costmap-filter-mask (`.pgm` + `.yaml`, same trinary format as a normal `map_server` map: 0=occupied/black, 254=free/white) → `filter_mask_server` + `costmap_filter_info_server` → `keepout_filter` plugin on `local_costmap`/`global_costmap`.

**One-shot orchestrator:** `scripts/gs_train_keepout.sh` chains §27 capture → `ns-train splatfacto` → `ns-export` → `gs_splat_to_pointcloud.py` → `gs_mask_from_splat.py`. Skip training with an existing npz:

```bash
MASK_ONLY=1 NPZ=$HOME/gs_data/cafe_points_final.npz \
  bash src/rosnav_bot/scripts/gs_train_keepout.sh cafe
# full train (needs Gazebo + ~/venvs/nerfstudio):
# bash src/rosnav_bot/scripts/gs_train_keepout.sh cafe
```

**`gs_mask_from_splat.py`:**
- Height-band filters Gaussian centers (default `z ∈ [0.05, 2.0]`, matching the `obstacle_layer`/`VoxelLayer` `min/max_obstacle_height` already used in `nav2_params.yaml`), rasterizes the XY footprint to occupied cells, dilates a couple of cells to absorb splat noise.
- `--align-to <existing map.yaml>` inherits that map's resolution/origin/size for pixel-exact overlay with a real SLAM map's `static_layer` (e.g. `maps/map_hospital.yaml`); omit it to auto-fit a bounding box + margin instead.
- No coordinate-frame correction beyond the height band — rasterizes whatever xyz the `.npz` carries, same "good enough for a spike" convention `gs_view_pointcloud.py` already uses. Mask quality is only as good as the underlying splat training (see §27's `gs_capture.py` coordinate gotcha).

```bash
python3 src/rosnav_bot/scripts/gs_splat_to_pointcloud.py splat_export/splat.ply cafe_points.npz \
    --dataparser-transform outputs/.../splatfacto/<timestamp>/dataparser_transforms.json  # nerfstudio venv, see §29b
ros2 run rosnav_bot gs_mask_from_splat.py \
    --npz cafe_points.npz --out src/rosnav_bot/maps/gs_keepout_cafe.yaml \
    --align-to src/rosnav_bot/maps/map_hospital.yaml   # or omit for auto-fit bbox
```

**Nav2 wiring:**
- `config/gs_keepout_filter.yaml` — params for `filter_mask_server` (a `nav2_map_server::MapServer` instance, just serving the mask instead of the main map, on `/gs/keepout_filter_mask`) and `costmap_filter_info_server` (publishes `/gs/costmap_filter_info`, `type: 0` = Keepout).
- `nav2_params*.yaml` (all 6 variants — humble/jazzy × default/mecanum/ackermann/mppi) and both `nav2_multirobot_params*.yaml` templates always list a `keepout_filter` (`nav2_costmap_2d::KeepoutFilter`) plugin on both costmaps, pointed at `/gs/costmap_filter_info` — safe by default since the plugin is inert until that topic is actually published. `filter_info_topic` is an absolute (leading-`/`) topic name, so `_common.namespace_nav2_params()` leaves it alone — one shared publisher covers every `robotN` namespace.
- `slam_nav.launch.py gs_keepout_mask:=<mask.yaml>` (default `''` = disabled) starts `filter_mask_server` + `costmap_filter_info_server` + a dedicated `lifecycle_manager_gs_keepout` (autostart) 9s after nav2 comes up.
- `multi_robot.launch.py gs_keepout_mask:=<mask.yaml>` does the same, but only **once** for the whole fleet (not per-robot) — every robot's namespaced costmaps subscribe to the same shared `/gs/costmap_filter_info`.

**Verified (2026-08-20):**
- Exported a real (if undertrained — 999-step smoke-test checkpoint) `cafe` splat via `ns-export gaussian-splat` and ran it through the full script — output PGM/YAML is well-formed but ~50% "occupied" since the checkpoint hadn't converged (splatfacto default budget in this spike's config was only 1000 steps); this is a training-data-quality issue, not a pipeline bug.
- Ran `slam_nav.launch.py world_name:=hospital slam:=false gs_keepout_mask:=<synthetic aligned mask>` headless — log confirms `filter_mask_server`/`costmap_filter_info_server` configure+activate and bond to `lifecycle_manager_gs_keepout`, and both `local_costmap` and `global_costmap` log `Initialized plugin "keepout_filter"`.
- Ran `multi_robot.launch.py world:=hospital explore:=false robot_count:=2 gs_keepout_mask:=<same mask>` headless — one `filter_mask_server`/`costmap_filter_info_server` pair started (not two), and both `robot1.local_costmap`/`robot1.global_costmap` and `robot2.local_costmap`/`robot2.global_costmap` independently logged `Initialized plugin "keepout_filter"`, confirming the fleet-wide-shared-topic design works. Clean process exit both times, no lingering PIDs.
- A real end-to-end demo (visible keepout dent in `/global_costmap/costmap`) needs a properly-converged splat — this session's checkpoint wasn't trained long enough for that.

---

## 29. GS Semantic Fusion (`gs_semantic_fusion.py`)

Second GS-navigation integration, built on the same converged `cafe` splat as §28: lifts `yolo_detector.py`'s 2D image detections into real 3D map-frame positions/extents by projecting the splat point cloud through the camera, instead of the flat-ground-plane assumption ROADMAP_CONCEPTS.md's plain "Semantic costmap layer from YOLO detections" idea would need. This is the perception/labeling half of a Splat-Nav-style fusion — **not** wired into a live Nav2 costmap filter yet (the existing `gs_keepout_filter.yaml` pipeline is a static, offline-generated mask; feeding live per-frame semantic regions into it would need a mutable `filter_mask_server`, left as follow-up).

**How it works:** for each `Detection2DArray` from `yolo_detector.py`, look up the `map` → camera-optical-frame TF at the detection's timestamp, coarse-crop the splat point cloud by range from the camera (cheap vectorized pass, keeps per-frame cost low even at ~500k+ points), project the survivors through the camera intrinsics (`sensor_msgs/CameraInfo`'s `k` matrix, standard pinhole/optical convention — the URDF's `camera_optical_frame` is a distinct child link from `camera_link` specifically to give ROS the standard x-right/y-down/z-forward convention regardless of Gazebo's native sensor-frame convention, same pattern as the lidar), and keep whichever splat points land inside each detection's 2D bbox. The matched points' map-frame median + robust (90th-percentile) extent become a `visualization_msgs/MarkerArray` (a colored `CUBE` + `TEXT_VIEW_FACING` label per detection) on `gs_semantic/markers`.

```bash
# enable_yolo:=true is required — gs_semantic_fusion has nothing to fuse without it
ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe enable_yolo:=true enable_camera:=true \
  gs_semantic_npz:=/home/asimov/gs_data/cafe_points_clean2.npz
```
In RViz: add a `MarkerArray` display on `gs_semantic/markers`.

**Verified (2026-08-21):** ran the full projection pipeline against the real converged `cafe` splat (583592 points post scale-filter, see §28) and a live TF tree from a running `slam_nav.launch.py world_name:=cafe` — a synthetic full-frame `Detection2DArray` (bypassing YOLO, see below) correctly resolved to map=(0.81, 0.04, 0.12) with extent (1.91, 1.52, 0.84)m from 24211 matched splat points, and published a well-formed `MarkerArray` (`DELETEALL` + `CUBE` + `TEXT_VIEW_FACING`) confirmed via `ros2 topic echo`.

**Known limitation, not a pipeline bug:** `yolov8n.pt` (COCO-trained on real photos) detects nothing in the `cafe` world's low-poly stylized furniture (single-leg pedestal tables, flat-shaded) even down to `confidence:=0.15` — a real sim-to-real domain gap for this asset style, not specific to this node. The mechanical fusion pipeline (TF → projection → bbox filter → marker) is verified correct via the synthetic-detection test above; an actual end-to-end "YOLO sees a chair → 3D marker appears" demo needs either a more photorealistic world/asset set or a fine-tuned detector. §35-A's `yolo_auto_label_cafe.yaml` mode is built to close this for the **`table`** class specifically (auto-labeled from `cafe.world`'s known table poses, no manual annotation) — see §35-A for current status (mechanism verified, an actual fine-tuned model not yet trained). `chair` stays unclosed either way, since cafe's chairs are baked into the opaque `model://Cafe` mesh with no separate per-object pose to auto-label from.

---

## 29b. GS Coordinate Scale Bug (`--dataparser-transform`)

Real bug, found by eyeballing the splat next to the robot model in `rviz/gs_overview.rviz` (§29) and it looking implausibly small — not caught by any automated check in §28-29, since those verified geometry/logic *shape* (masks rasterize, markers publish) without an independent ground-truth distance to check absolute scale against.

**Root cause:** `ns-train` normalizes camera poses into a roughly unit-scale training space (`camera_utils.auto_orient_and_center_poses` then `poses[:, :3, 3] *= scale_factor`, saved as `dataparser_transforms.json`: `X_train = scale * (R @ X_world + t)`). Confirmed by reading nerfstudio's own exporter source directly (`nerfstudio/scripts/exporter.py`, `ExportGaussianSplat.main`): `positions = model.means.cpu().numpy()` — the exported `.ply` is the model's raw internal (training-space) Gaussian positions, with **no inverse transform applied**. Every `.ply` this repo has exported this session was in that squashed space, off by `scale` (0.2222 observed on the real `cafe` capture — the whole scene came out ~4.5x too small vs. true Gazebo-world meters).

**Blast radius:** every GS-derived artifact built on an uncorrected npz this session was undersized by the same factor relative to the robot/map TF frames it's meant to align with — §28's keepout masks, §29's semantic-fusion 3D positions, §30's speed masks, §32's merge tool, §34's relocalization check. None of those are wrong in *logic* (the projection/rasterization math is correct), just in absolute scale, since they all operate purely on whatever xyz the npz carries.

**Fix:** `gs_splat_to_pointcloud.py` now takes `--dataparser-transform <path to dataparser_transforms.json>` (from the training output dir, `outputs/.../splatfacto/<timestamp>/`) and applies the inverse: `X_world = R^T @ (X_train / scale - t)`. Also corrects the `max_scale` floater-filter threshold (§29 body text) by the same factor, since Gaussian extent is a length quantity in the same squashed space. Omitting the flag still works (backward compatible) but prints a stderr warning and leaves xyz in training-space, matching this session's prior (buggy) behavior — every example command in this doc and the README has been updated to pass it.

**Verified (2026-08-21):** re-exported the real converged `cafe` splat with the flag — bbox grew from (11.1m, 12.0m, 7.5m) to (58.1m, 59.6m, 51.1m) span, ~4.5-5.2x per axis (roughly consistent with `1/scale = 4.5`; not exactly uniform since the pre-correction bbox included floater outliers at the extremes, not a clean scale reference). Re-launched `rviz/gs_overview.rviz` against a live sim with the corrected npz — splat geometry now sits at plausible, consistent scale next to the `RobotModel` and TF frames (screenshot: `images/gs_overview_3d_lidar.png`).

**Not done:** the *committed* example command outputs (§28/§30's demonstrated mask percentages, §29's marker centroid/extent numbers) were generated before this fix and are now stale by the same ~4.5x factor — left as-is rather than mass-edited, since re-verifying every downstream number would mean re-running the full pipeline again; treat those specific figures as illustrating the *mechanism*, not current absolute scale.

---

## 30. GS Speed Costmap Filter (`gs_speed_mask_from_splat.py`)

Density-graded sibling of §28's binary `KeepoutFilter`: `gs_speed_mask_from_splat.py` rasterizes the same per-cell Gaussian COUNT grid `gs_mask_from_splat.py` uses, but instead of thresholding to occupied/free, linearly maps count → a 0-100 mask value (0 = empty cell, 100 = at/above `--dense-percentile`) written with `mode: scale` (not `trinary`). A `nav2_costmap_2d::SpeedFilter` (`type: 1`) plugin reads it and slows the robot through sparse/hazy splat regions instead of refusing to enter them outright, while still forcing a near-stop through the densest (most likely solid) cells — useful when a `KeepoutFilter`'s all-or-nothing cutoff is too blunt (e.g. splat noise near a real doorway shouldn't hard-block it).

**Wiring:** `config/gs_speed_filter.yaml` configures two nodes — `gs_speed_filter_mask_server` (serves the mask on `/gs/speed_filter_mask`) and `gs_speed_costmap_filter_info_server` (publishes `/gs/speed_filter_info`, `type: 1`, `base: 100.0`/`multiplier: -1.0` so mask value 0→100% speed and 100→0% speed). Deliberately distinct node names from §28's `filter_mask_server`/`costmap_filter_info_server` so both filters can run simultaneously without a name collision. `nav2_params*.yaml` (all 8 variants: humble/jazzy × default/mecanum/ackermann/mppi × single/multirobot) list a `speed_filter` plugin alongside `keepout_filter` on both costmaps, pointed at `/gs/speed_filter_info` — inert until that topic is actually published, same safe-by-default pattern as §28. `slam_nav.launch.py gs_speed_mask:=<mask.yaml>` starts the two filter nodes + a dedicated `lifecycle_manager_gs_speed`.

**Real bug hit and fixed during this build:** a ROS 2 node-parameters YAML file is silently ignored if its top-level key doesn't match the actual launched node's name (no error — the node just runs on pure defaults). `gs_speed_filter.yaml` initially kept §28's key names (`filter_mask_server:`/`costmap_filter_info_server:`) while the launch file (correctly) gave the nodes distinct names to avoid colliding with a simultaneously-running keepout filter — so none of the file's params applied, and the mask server defaulted to `topic_name: "map"`, i.e. a **second publisher on the real SLAM `/map` topic**. Fixed by renaming the YAML's top-level keys to match the actual node names. Caught by checking `ros2 param get /gs_speed_filter_mask_server topic_name` live rather than trusting that "no launch errors" meant it was wired correctly.

**Verified (2026-08-21):** ran `gs_speed_mask_from_splat.py` against the real converged+floater-filtered `cafe` splat (§28/§29's `cafe_points_clean2.npz`, same `--xy-crop` as the keepout mask) — 3.1% of cells fully restricted, mean mask value 8.1/100, visibly graded (not a hard blob) when rendered. Launched `slam_nav.launch.py world_name:=cafe gs_speed_mask:=<that mask>` headless on an isolated `ROS_DOMAIN_ID` — after the yaml-key fix, `ros2 param get` confirmed both nodes picked up the real config (`topic_name=/gs/speed_filter_mask`, `filter_info_topic=/gs/speed_filter_info`, `type=1`), and both `local_costmap` and `global_costmap` logged `SpeedFilter: Received filter info` and `SpeedFilter: Received filter mask`.

---

## 31. GS Mask as a Static-Layer Prior (`map_override`)

A `gs_mask_from_splat.py` KeepoutFilter mask (§28) is already a plain Nav2 map — trinary PGM+YAML, same format `map_server`/`static_layer` expect — so instead of only *subtracting* space via a filter overlay, it can seed nav-on-map mode (`slam:=false`) directly as a GS-derived floor-plan prior, letting Nav2 navigate with global structure from one offline splat flythrough before the robot has mapped anything itself with lidar.

`slam_nav.launch.py` previously had no way to point nav-on-map mode at an arbitrary map file — `map_yaml` was always auto-resolved to `maps/map_<world_name>.yaml`. Added a `map_override:=<path>` launch arg (default `''`) that takes priority over the auto-resolved path when set:

```bash
ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe slam:=false \
  map_override:=src/rosnav_bot/maps/gs_keepout_cafe_final.yaml
```

**Not yet live-verified** (static-verified only — `py_compile` + a full offline `LaunchDescription` build both pass; the 2-line resolution change — `map_override` if set, else the existing `_resolve_map_yaml()` call — is low-risk, unlike §30's filter-info wiring, which is why it wasn't prioritized for a live Gazebo run on an already heavily-loaded shared machine this session). Follow-up: launch it for real and confirm `map_server` loads the GS mask and Nav2 plans against it before any SLAM/AMCL correction.

---

## 32. Multi-Splat Merging (`gs_merge_splats.py`)

Extends the "one shared `/gs/costmap_filter_info` topic covers every `robotN` namespace" design `multi_robot.launch.py`'s `gs_keepout_mask` already uses (§28) upstream to the splat data itself: `gs_merge_splats.py` concatenates several `splat_points.npz` files (e.g. one per robot, or one per capture session covering a different room) into a single merged npz that `gs_mask_from_splat.py`/`gs_speed_mask_from_splat.py`/`gs_semantic_fusion.py` can all consume unchanged.

Common case needs no registration: multiple `gs_capture.py` runs against the *same* Gazebo world already share one absolute coordinate frame (see §27's coordinate-convention note), so their npz files can just be concatenated directly. For a capture that used its own independent frame, `--transform X Y Z YAW_DEG` (paired positionally with the preceding `--npz`) applies a rigid translation + yaw-about-Z before merging.

```bash
ros2 run rosnav_bot gs_merge_splats.py \
    --npz robot1_lobby.npz --npz robot2_kitchen.npz --transform 8.5 -2.0 0 90 \
    --out merged_facility.npz
```

**Known limitation:** overlapping Gaussians where two captures see the same physical area are not deduplicated — left as-is since downstream density-threshold rasterization already tolerates overlap fine (it counts per-cell, so overlap just adds count); a raw merged point count in an overlap region just isn't independently meaningful.

**Verified (2026-08-21):** merged the real `cafe` splat with a translated+rotated copy of itself (`--transform 20 0 0 90`) — output point count was exactly the sum of both inputs (1167184 = 583592×2), and the transformed copy's X range shifted to the expected ~20-26m band, confirming the transform math and the `--npz`/`--transform` positional-pairing argparse logic both work correctly.

---

## 33. Capture View-Planning (`gs_suggest_capture_waypoints.py`)

Closes the GS↔navigation loop in the other direction from §28-32 (which all use a splat to improve navigation): use what SLAM has already mapped to suggest where `gs_capture.py`'s *next* orbit sweep should point, so the splat grows to cover what the robot has actually explored instead of staying fixed to one manually-chosen capture area.

Reads a saved Nav2 map (`map_saver_cli` output) and, optionally, an existing splat's npz to know what's already covered (grid-approximated: a free cell counts as covered if it's within `--cover-radius` of any covered-npz point, checked via a 3×3 bin neighborhood rather than an O(n·m) distance query). Remaining uncovered free cells are grid-binned into clusters (`--cell-size`); each cluster's centroid, ranked by cluster size, becomes a suggested capture waypoint.

```bash
ros2 run rosnav_bot gs_suggest_capture_waypoints.py \
    --map src/rosnav_bot/maps/map_cafe.yaml \
    --covered-npz /home/asimov/gs_data/cafe_points_clean2.npz \
    --cell-size 2.0 --top-n 6
```

**Known limitation:** `gs_capture.py`'s `centers_x`/`centers_y` parameters form a *cartesian grid* (`[(x, y) for x in centers_x for y in centers_y]`), not an arbitrary waypoint list — so scattered cluster centroids from an irregular uncovered region don't collapse cleanly into that parameterization. This script prints the suggested `(x, y)` points directly; turning each into an actual capture pass currently means running `gs_capture.py` once per point (single-element `centers_x`/`centers_y` each time), or extending `gs_capture.py` to accept an explicit waypoint list — not attempted here.

**Verified (2026-08-21):** two checks. (1) Cross-world sanity check (`map_hospital.yaml` vs. the unrelated `cafe` splat) ran without error but wasn't semantically meaningful — different physical spaces that happen to both be centered near the same absolute coordinates, so "coverage" there is coincidental, not real. (2) Self-consistent check — used `gs_keepout_cafe_final.yaml` (§28, itself derived from `cafe_points_clean2.npz`) as `--map` against that *same* npz as `--covered-npz`: 93.6% of the mask's own free space was correctly recognized as already covered, with the uncovered fringe (~6.4%) landing exactly at `gs_mask_from_splat.py`'s own 1.0m auto-fit-bbox margin (no real data there by construction) — confirms the coverage-detection grid logic is doing the right computation, not just returning plausible-looking noise.

---

## 34. GS Relocalization Confidence Check (`gs_relocalization_check.py`)

A coarse, cheap localization-confidence signal from the splat, layered on top of the geometric one RTAB-Map/AMCL already provide: at the robot's current estimated pose, project the splat's own point *colors* through the camera (same TF + intrinsics projection as §29's `gs_semantic_fusion.py`) into a low-res (`--grid-bins`² ) color grid, and compare it against the live camera frame downsampled to the same grid. If the pose estimate is right, average color per region should roughly agree; a bad pose (drift, kidnapped robot, bad loop closure) desyncs them.

**Deliberately not** full photometric splat rendering — a real Splat-Nav-style render-and-compare needs the actual differentiable Gaussian rasterizer (gsplat/nerfstudio's rendering pipeline) running live inside a ROS node, a much heavier GPU/dependency lift than this spike takes on. This is a coarse proxy — binned average point color vs. binned average pixel color, nothing photoreal — and its output (`gs_reloc/color_error`, `gs_reloc/coverage`) is meant as a soft signal to log/monitor, not something wired into gating actual localization; no such consumer exists.

```bash
ros2 run rosnav_bot gs_relocalization_check.py --ros-args \
    -p use_sim_time:=true -p npz_path:=/home/asimov/gs_data/cafe_points_clean2.npz
```

**Verified (2026-08-21):** ran live against the real converged `cafe` splat and a running `slam_nav.launch.py world_name:=cafe` sim — stable output across multiple cycles: `color_error=41.0/255` (~16%, plausible given Gazebo's flat-shaded low-poly rendering vs. the splat's training-blended colors won't match pixel-perfectly), `coverage=0.99` (254/256 grid bins had splat points to compare, i.e. the splat's capture footprint covers almost the whole camera view from this pose). Check-interval grew from ~2s to ~20s over the run — real-time-factor slowdown from several *other* concurrent Gazebo instances competing for the same GPU/CPU on this shared machine (confirmed via `ROS_DOMAIN_ID` isolation checks — several unrelated sessions were running simultaneously), not a bug in this node.

---

## 35. AI Training Pipelines (YOLO · GS keepout · RL)

Classic SLAM Toolbox / AMCL / Nav2+MPPI stay **tuned, not neural-net trained**. These three add-ons are where actual model training lands — perception weights, a splat used as cost, and a research local planner. They do not replace the core stack.

### A — YOLO fine-tune (`yolo_collect.py` / `yolo_train.py`)

Closes the §13 / §29 domain gap: collect sim RGB, annotate in YOLO format (`class cx cy w h` normalized), fine-tune Ultralytics, point `yolo_detector.py` at `best.pt`.

```bash
ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe enable_camera:=true
ros2 run rosnav_bot yolo_collect.py --ros-args \
    -p out_dir:=$HOME/yolo_data/cafe -p max_frames:=200 -p classes:=chair,table
# annotate labels/train/*.txt (empty files = background), then:
python3 src/rosnav_bot/scripts/yolo_train.py \
    --data $HOME/yolo_data/cafe/dataset.yaml --epochs 50
# smoke (no Gazebo / no labels):
python3 src/rosnav_bot/scripts/yolo_train.py --smoke

ros2 launch rosnav_bot slam_nav.launch.py enable_yolo:=true \
    yolo_model:=runs/detect/rosnav_yolo/weights/best.pt
```

`yolo_collect.py` writes `dataset.yaml` + `images/{train,val}` / `labels/{train,val}`. Collect a second pass with `-p split:=val` for a real validation split (smoke mode synthesizes both).

**Verified (2026-08-21):** `--smoke` built a tiny synthetic 2-class set and completed 1 epoch on CPU; weights landed under `runs/detect/_smoke/weights/best.pt`.

**Auto-labeling (no manual annotation) for `cafe`'s tables:** `yolo_collect.py`'s `auto_label_config` param (see `config/yolo_auto_label_cafe.yaml`) projects each of cafe's 5 known-pose `table1..table5` 3D AABBs into every captured frame via TF + camera intrinsics — same projection convention as `gs_semantic_fusion.py` — and writes real YOLO labels instead of empty ones. Only `table` is covered this way; `chair`/other cafe furniture is baked into the opaque `model://Cafe` mesh with no separate pose to auto-label from.

Two real bugs surfaced while building this:
1. **Degenerate boxes at near-camera crossings.** Projecting only the AABB corners still in front of the camera (`z > 0.1`) and taking their 2D hull is wrong when *some but not all* 8 corners cross the near plane — the hull of the survivors can stretch across most of the frame (confirmed visually: a thin sliver box spanning ~90% of image height that didn't correspond to anything in the rendered image). Fixed by requiring *all 8* corners in front before accepting a box — a strictly stricter, monotonic tightening (can only shrink the accept set, never introduce a wrong box).
2. **`map` frame is the wrong frame for static ground-truth coordinates.** `slam_toolbox`'s `map` frame is an active *estimate* it keeps re-correcting via scan matching — confirmed empirically by solving the world→map rigid transform from live data twice a few minutes apart in the same run: the same solve implied `table1` was at two different map-frame positions ~0.9m/~23° apart. A frame that drifts mid-session can't hold a hardcoded object list. Fixed by using **`odom`** instead (`-p map_frame:=odom`) — fixed at launch, no active re-estimation, only (much smaller) wheel-odometry integration error. `yolo_auto_label_cafe.yaml`'s coordinates were produced by solving the exact world↔odom rigid transform from one live sample (odom-frame `base_link` TF vs `gz topic -e -t /world/cafe/pose/info` ground truth) right after launch, before wheel drift could accumulate — see the YAML's header comment for the derivation. This is specific to cafe's default spawn; re-solve if `spawn_x/y/yaw` are overridden.

**Status (2026-08-21):** both fixes are verified analytically (a hand-built synthetic camera pose reproduced the correct sign/scale conventions; the odom-frame transform was checked live and correctly clips off-screen tables to zero-width rather than a degenerate box) and partially verified live (individual frames showed correct clip-to-edge behavior for out-of-view tables). Not yet confirmed: a rendered overlay of an actual in-view, non-empty-label frame using the corrected odom-frame config — two capture attempts happened to have frontier exploration wander to other parts of the cafe (the kitchen/counter area) rather than the table row during the capture window, a matter of exploration timing/luck rather than a code issue. Re-running the capture for longer, or driving toward the table row first, should get a confirmable frame — left as a follow-up before actually fine-tuning on this dataset.

### B — GS train → keepout (`gs_train_keepout.sh`)

Orchestrates §27 → nerfstudio `splatfacto` → §28 mask in one script. Env knobs: `DATA_DIR`, `NS_VENV` (default `~/venvs/nerfstudio`), `OUT_MASK`, `MAX_ITERS`, `SKIP_CAPTURE`, `ALIGN_TO`, `DENSITY_PERCENTILE`.

```bash
# mask only (existing npz — no GPU train):
MASK_ONLY=1 NPZ=$HOME/gs_data/cafe_points_final.npz \
    OUT_MASK=src/rosnav_bot/maps/gs_keepout_cafe.yaml \
    bash src/rosnav_bot/scripts/gs_train_keepout.sh cafe

# then:
ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe \
    gs_keepout_mask:=src/rosnav_bot/maps/gs_keepout_cafe.yaml
```

**Verified (2026-08-21):** `MASK_ONLY=1` on `cafe_points_final.npz` wrote a well-formed keepout PGM/YAML (~5.6% cells occupied at density-percentile 90). Full `ns-train` path is the same script without `MASK_ONLY` (long GPU job; not re-run in that smoke).

### C — RL local planner (`rosnav_bot/rl` + `train_ppo.py` + `rl_policy_node.py`)

Research controller from ROADMAP_CONCEPTS ("learned local planner"): train offline on a static Nav2 occupancy map with raycast LiDAR — **no Gazebo in the train loop** — then deploy as a `/cmd_vel` source. Observation = downsampled scan (`n_beams`, default 36, normalized by `max_range`) + goal `(dx, dy, dyaw)` in the robot frame; action = `[v, w]` in `[-1, 1]` scaled by `v_max`/`w_max`.

| Piece | Role |
|---|---|
| `rosnav_bot/rl/scan_nav_env.py` | Gymnasium env: load PGM+YAML, unicycle dynamics, progress/collision reward |
| `rosnav_bot/rl/policy.py` | Small continuous Actor–Critic (PyTorch) |
| `scripts/train_ppo.py` | PPO loop; default `--backend torch` (pure PyTorch). `--backend sb3` optional if stable-baselines3 works on your stack |
| `scripts/rl_policy_node.py` | Loads `.pt` (or SB3 `.zip`), subscribes `/scan` + `/odom` (+ `/goal_pose`), publishes `/cmd_vel` |

```bash
python3 src/rosnav_bot/scripts/train_ppo.py --smoke
python3 src/rosnav_bot/scripts/train_ppo.py \
    --map src/rosnav_bot/maps/map_maze.yaml --timesteps 200000 --out runs/rl/ppo_maze

# with robot up — disable / remap Nav2's controller when testing this node
ros2 run rosnav_bot rl_policy_node.py --ros-args \
    -p model_path:=runs/rl/ppo_maze/ppo_scan_nav.pt \
    -p goal_x:=2.0 -p goal_y:=1.0
```

Not for production nav — benchmark against MPPI in `maze`, blog/demo angle. Live Gazebo-as-Gym wrapper is still optional follow-up (ROADMAP); the PGM raycast env is deliberately enough to train and deploy the same observation layout.

**Verified (2026-08-21):** `--smoke` (2048 steps, `map_maze.yaml`, `--backend torch`) saved `runs/rl/_smoke/ppo_scan_nav.pt`; deterministic forward pass loads and infers. On this machine `stable-baselines3` `PPO(...)` segfaults during construction (Torch 2.12+cu130 interaction) — hence the pure-torch default; use `--backend sb3` only where SB3 is known-good.
