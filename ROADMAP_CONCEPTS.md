# Concepts to Level Up rosnav

Things the stack doesn't have yet. Checked against `src/rosnav_bot/scripts/` and `config/` so this doesn't repeat what's already built (fleet_manager, task_allocator, deadlock_recovery, obstacle_tracker, priority_collision_avoidance, map_fusion, bt_executor/bt_server, RMF, YOLO, ArUco docking, LLM voice nav, mecanum/ackermann, MiR100/Husky skins).

Each entry: what it is → why it'd help *this* project → what to touch.

---

## 1. Localization & Sensor Fusion

### `robot_localization` EKF/UKF
Fuses wheel odom + IMU (+ GPS outdoors) into a smoothed `/odometry/filtered`, replacing raw `/odom` as the TF source. Right now `odom_tf_broadcaster.py` looks like it publishes odom → base_link directly from one source — an EKF would cut drift a lot, especially on mecanum/ackermann where wheel-only odom lies during slip/skid turns.
→ `ros-*-robot-localization`, new `ekf.yaml`, swap the odom source Nav2 listens to.

### Dual-EKF + navsat_transform for `outdoor` / `multi_terrain` worlds
Those two worlds are listed in the README as "no map yet — use explore:=true." A GPS+IMU global EKF would let them get a real geo-referenced map instead of relying purely on SLAM/frontier explore.
→ `navsat_transform_node`, a `world_name:=outdoor` GPS plugin in the Gazebo world.

---

## 2. Planning & Control

### Smac Planner (Hybrid-A* / State Lattice)
NavFn (current global planner) doesn't respect kinematics. For `drive_type:=ackermann` this matters most — Hybrid-A* plans curvature-feasible paths so the car-like robot doesn't get global paths it has to fight the controller to follow.
→ swap `planner_server` plugin in `nav2_params_ackermann.yaml` to `nav2_smac_planner::SmacPlannerHybrid`.

### TEB local planner as an MPPI alternative
TEB (Timed Elastic Band) is better than MPPI in narrow corridors / tight doorways because it explicitly reasons about robot footprint vs. time. Worth an A/B against MPPI in `maze`/`corridor` worlds.
→ `ros-*-teb-local-planner`, new `nav2_params_teb.yaml`, wire a `controller:=teb` launch arg next to the existing `controller:=mppi`.

### Speed & keepout costmap filters (mask-based, not just static zones)
`no_go_zones.yaml` exists but Nav2 also ships `SpeedFilter` and `KeepoutFilter` driven by a filter-mask image (per-pixel, not polygon) — useful for "slow zone near the dock" or "no-go near the charging bay" without hand-writing polygons.
→ `nav2_map_server::CostmapFilterInfoServer` + a mask PGM per world.

### Learned local planner (DRL) as a research controller — started
Offline PPO on occupancy maps (`rosnav_bot/rl/ScanNavEnv` + `scripts/train_ppo.py`) → live `scripts/rl_policy_node.py` on `/scan`+odom. Not for production nav — benchmark vs MPPI in `maze`. Optional next: Gymnasium wrapper around live Gazebo instead of the PGM raycast env.

---

## 3. Perception

### Semantic costmap layer from YOLO detections
`yolo_detector.py` publishes detections but nothing feeds them back into planning. A semantic layer that marks "person" detections as a soft-cost region (not a hard obstacle) would make Nav2 slow down / give people berth without stopping dead like a static obstacle does.
→ custom `nav2_costmap_2d` plugin subscribing to `yolo/detections`, project image-space boxes to costmap cells via camera intrinsics + TF.

### Object permanence / persistent semantic map
Store YOLO detections with world-frame coordinates in a lightweight DB (sqlite) so `llm_nav.py` can answer "where did you last see the pallet jack?" — extends the existing LLM voice nav from navigation-only to query-answering.
→ small `semantic_memory.py` node, SQLite table `(class, x, y, timestamp, confidence)`.

---

## 4. Multi-Robot / Fleet

### Conflict-Based Search (CBS) for multi-robot path planning
`deadlock_recovery.py` + `priority_collision_avoidance.py` are reactive (detect conflict, then resolve). CBS plans conflict-free paths for the whole fleet *before* execution — fewer runtime deadlocks in tight corridors during `multi_robot.launch.py` runs.
→ new `scripts/cbs_planner.py`, feeds waypoints into `mission_server.py` instead of each robot planning independently.

### Battery-aware cost in task allocation
`task_allocator.py` already solves Hungarian optimal assignment on travel distance — that part's done well. What it doesn't factor in: battery level. Add a battery-drain estimate (or just `1/charge_pct`) as a penalty term in the cost matrix so a low-battery robot gets routed to the dock instead of the farthest task, and gets skipped for new assignments below a threshold.
→ extend the cost matrix build in `task_allocator.py`, needs a `/battery_state` topic per robot (Gazebo battery plugin or simulated drain).

### Elevator / multi-floor handling for Open-RMF
RMF integration (`rmf_fleet.launch.py`) is flagged experimental and single-floor per the README. Multi-floor lane graphs + a mock elevator door-plugin would be the natural next RMF milestone.

---

## 5. Cloud / Dashboard Integration

Given RoboCloud Hub exists as a separate web project, this is the highest-leverage addition: turn rosnav into something that *demonstrates* on a web dashboard, not just RViz.

### rosbridge_suite / Foxglove Bridge → web telemetry
Expose `/map`, `/odom`, fleet status, and YOLO detections over a WebSocket so a browser dashboard (or Foxglove Studio) can watch the fleet live without a ROS install.
→ `ros-*-rosbridge-suite` or `foxglove_bridge`, one launch arg `web_bridge:=true`.

### MQTT bridge for fleet health → RoboCloud
`fleet_health.py` already computes health — publish it to an MQTT topic (`mosquitto` broker) so an external dashboard can subscribe without needing ROS 2 DDS discovery across a network boundary.
→ `ros-*-mqtt-client` or a small paho-mqtt bridge node.

### rosbag2 record/replay for debugging + demos
No recording script visible. A `record_session.py` wrapping `ros2 bag record` on a curated topic set (`/map /odom /scan /tf /yolo/detections /cmd_vel_safe`) makes it trivial to replay a run for debugging or to generate demo GIFs without re-running Gazebo.

---

## 6. Safety & Testing

### `diagnostic_updater` / `diagnostic_aggregator` for fleet_health
Standardizes health reporting into ROS 2's diagnostics framework (severity levels, hardware IDs) instead of a bespoke format — makes it pluggable into `rqt_robot_monitor` or any dashboard that already speaks `DiagnosticArray`.

### launch_testing + colcon test for regression coverage
With this many launch permutations (drive_type × lidar_type × slam_mode × robot_model), a regression bug is easy to introduce silently. `launch_testing` smoke tests (spin up `slam_nav.launch.py` headless, assert `/map` starts publishing within N seconds) would catch breakage before it's discovered manually.

### SROS2 (DDS security) — optional, if ever multi-machine
Only relevant once the fleet spans more than one physical machine/network — otherwise skip. Worth a one-line note in concepts.md if the RMF work ever goes distributed.

---

## 7. Simulation Quality

### Domain randomization in Gazebo worlds
Randomize lighting, texture, and obstacle placement per `explore:=true` run so YOLO/SLAM don't overfit to the fixed `hospital`/`warehouse` layouts — cheap to add via a world-spawn script that jitters a few SDF params before launch.

### Headless batch simulation for benchmarking
Run N episodes of `slam_nav.launch.py ... explore:=true` headless (`gz sim -s`, no GUI) and log time-to-map-complete, path length, collision count — gives a real benchmark number when comparing MPPI vs. TEB or NavFn vs. Smac from §2, instead of eyeballing it in RViz.

---

## 8. More

### Path smoothing for waypoint routes
`waypoint_nav.py` drives `FollowWaypoints` through raw yaml points — corners between waypoints are hard stops/turns. Nav2's `nav2_smoother` (or a simple Catmull-Rom/B-spline pre-pass on the waypoint list) gives continuous curvature, noticeably smoother multi-point patrol/coverage runs.

### Conversational loop for `llm_nav.py`
Right now it looks like one-shot command → action. Adding TTS feedback ("heading to room_b, ETA 12s") plus a wake-word (`porcupine`/`openWakeWord`) so it doesn't need push-to-talk turns it from a command parser into an actual voice interface — bigger demo value for the same LLM plumbing already in place.

### Containerized dev environment
This many interacting pieces (ROS distro, Gazebo Harmonic, Ultralytics, Ollama, RMF) is a lot to ask a fresh machine to reproduce. A `Dockerfile`/`docker-compose.yaml` pinning ROS distro + deps means `docker compose up` gets someone to a working `slam_nav.launch.py` — also the cleanest way to eventually run CI (see launch_testing above) without a bare-metal ROS box.

### Edge deployment: quantized YOLO for Jetson
`yolo_model:=yolov8n.pt` is already the small variant, but running it on a Jetson (as opposed to a sim-only dev box) benefits from TensorRT export (`yolo export format=engine`) — same detection topic, much lower inference latency, relevant the moment this leaves Gazebo for a real chassis.

### GitHub Actions CI on `colcon build` + lint
No CI visible in the repo. A minimal workflow — `colcon build --symlink-install` + `ament_lint` on push — catches build breaks from the Humble/Jazzy dual-config split (this project maintains parallel `_jazzy` yaml files, easy for one distro's config to silently drift from the other).

---

## Suggested order of attack

1. **Semantic costmap layer from YOLO** — cheapest, reuses `yolo_detector.py` already running, immediately visible improvement in the `warehouse` world.
2. **rosbridge/Foxglove bridge** — unlocks the RoboCloud dashboard demo angle, no changes to nav logic required.
3. **Smac Planner for ackermann** — fixes a real kinematic-feasibility gap, isolated to one config file.
4. **robot_localization EKF** — foundational, makes everything downstream (docking, YOLO projection, RMF) more accurate.
5. **CBS multi-robot planning** — biggest effort, biggest payoff for fleet deadlock reduction.
