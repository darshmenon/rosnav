# ORB-SLAM3 sidecar (Docker)

Feature-based visual SLAM, run as a separate container alongside rosnav_bot's
sim — for comparison against the RTAB-Map backends already built into this
repo (`slam_algo:=vslam|3d|multisensor`, see `concepts.md` §3). Recipe is
re-derived from the working build at
[suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker),
with `params/rosnav_rgbd*.yaml` swapped in for this repo's own camera
topics/frames/intrinsics (`urdf/rgbd_camera.xacro`).

Why a separate container instead of building this into the main workspace:
ORB-SLAM3 has no ROS distro apt package — it needs a from-source OpenCV 4.4 +
Pangolin + ORB-SLAM3 native build (~30-60 min) — and it's GPLv3 (`rtabmap_slam`
is BSD). Keeping it a separate opt-in container means the main `rosnav_bot`
image stays fast to build and license-clean, and this can be dropped in
against any RGB-D source without pulling GPL code into the main workspace.

It **is** wired as `slam_algo:=orbslam3` on `slam_nav.launch.py`, though —
that starts `scripts/orb_slam3_occupancy_bridge.py`, a node that turns this
sidecar's `map->odom` TF + the depth camera into a real `nav_msgs/OccupancyGrid`
on `/map`, the same way RTAB-Map's `Grid/Sensor=1` does internally for
`slam_algo:=vslam`. ORB-SLAM3 itself only publishes a sparse feature-point
map, not an occupancy grid, so without this bridge Nav2/the frontier explorer
would have nothing to consume. The tracker process still can't join
`slam_nav.launch.py`'s process tree (GPL/build isolation above) — you start
it separately, in parallel:

```bash
ros2 launch rosnav_bot slam_nav.launch.py slam_algo:=orbslam3 world_name:=cafe explore:=true
docker compose up orb_slam3          # separate terminal
```

`/map` stays all-unknown (bridge logs a warning, throttled) until the sidecar
starts publishing `map->odom` — Nav2 and the frontier explorer just wait, same
as any SLAM backend that hasn't produced its first map yet.

## Build

```bash
docker build -t rosnav_orb_slam3:humble -f docker/orb_slam3/Dockerfile .
# or: docker compose build orb_slam3
```

CPU-only. This box has no NVIDIA GPU — for a CUDA build, follow the `nvidia_gpu`
target in the upstream Dockerfile linked above and adapt the CUDA stage here.

### Without Docker

Same recipe, no container — `docker/orb_slam3/build_bare_metal.sh` runs the
identical OpenCV 4.4/Pangolin/ORB-SLAM3/wrapper build directly on the host,
into `~/orb_slam3_ws` (kept separate from this repo's own colcon workspace,
same GPLv3-isolation reasoning as the Docker image above):

```bash
bash docker/orb_slam3/build_bare_metal.sh
source ~/orb_slam3_ws/install/setup.bash
ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py sensor_config:=rgbd
```

Don't run this at the same time as `docker build .../Dockerfile` — both
compile the same heavy OpenCV/Pangolin/ORB-SLAM3 stack and will contend for
CPU/RAM.

## Run

1. Start the sim with `slam_algo:=orbslam3` — this enables the RGB-D camera
   automatically and starts `orb_slam3_occupancy_bridge.py`:
   ```bash
   ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe slam_algo:=orbslam3 explore:=true
   ```
2. In another terminal:
   ```bash
   docker compose up orb_slam3
   ```
   `network_mode: host` means it discovers `/camera/image_raw`,
   `/camera/depth/image_raw`, `/tf` etc. over normal ROS 2 DDS — no explicit
   container linking needed, same as running a second terminal on the host.
3. `visualization` defaults to `false` (see Known caveats) — no viewer
   window pops up. Confirm it's actually tracking via the logs instead:
   ```bash
   # "Current ORB-SLAM3 tracking frequency: ..." once a second means it's alive
   ros2 topic echo /map --once --field info    # width/height fill in once map->odom exists
   ```
   Once ORB-SLAM3 starts publishing `map->odom`, `/map` fills in and
   Nav2/the frontier explorer pick it up like any other backend — verified
   end-to-end (see concepts.md §3's ORB-SLAM3 section for the full bug list
   this took to get working).

Textured worlds (`cafe`, `hospital`, Fuel worlds via
`scripts/download_fuel_worlds.sh`) track much better than flat/untextured
ones — ORB-SLAM3 needs visual features to match, same caveat as this repo's
`slam_algo:=vslam`.

## Localization / save-reload

Same two-phase flow as `slam_algo:=vslam`'s `rtabmap_db`:

1. Map once with `System.SaveAtlasToFile: ./atlas` uncommented in
   `params/rosnav_rgbd.yaml` (rebuild the image, or edit the file inside a
   running container and `colcon build` again).
2. Ctrl-C **once** and wait — saving the atlas takes a moment.
3. For subsequent runs, comment `SaveAtlasToFile` back out and uncomment
   `System.LoadAtlasFromFile: ./atlas` instead.

## Known caveats

- `Camera.fx/fy/cx/cy` in `params/rosnav_rgbd.yaml` are computed from the
  xacro's declared FOV/resolution, not a real calibration — fine for sim,
  re-run `ros2 run camera_calibration cameracalibrator` against real hardware.
- `odometry_mode: true` (see comment in `rosnav_rgbd_ros_params.yaml`) is
  load-bearing: it keeps `ekf_filter_node` as the sole `odom->base_link`
  publisher and has ORB-SLAM3 only add `map->odom` on top — same
  one-TF-publisher rule the rest of this repo's SLAM backends follow. Don't
  flip it without also disabling `ekf_filter_node`, or you'll get two nodes
  fighting over the same TF edge.
- `orb_slam3_occupancy_bridge.py`'s occupancy grid is a simple ray-traced
  projection of the depth image (Bresenham free-space tracing + height-band
  occupied endpoints), not RTAB-Map's more sophisticated grid assembly — good
  enough for Nav2 costmaps/frontier exploration, not a pixel-perfect map.
- `costmap_ghost_clear.py` already runs for `slam_algo:=orbslam3` (clears
  Nav2's costmap on a `map->odom` jump, same as every other backend) —
  `slam_accuracy_monitor.py` doesn't know about this backend yet.
- `visualization: false` is the default — ORB_SLAM3's `Viewer::Run()`
  bundles the Pangolin 3D view together with a GTK-based `cv::imshow` debug
  window under the same flag. The Docker image *does* build OpenCV with GTK
  support (matching upstream) — flip this to `true` there any time, no
  rebuild needed. The bare-metal build deliberately skips GTK/GStreamer dev
  headers, so it needs an OpenCV rebuild with `libgtk-3-dev` present first
  (see `params/rosnav_rgbd_ros_params.yaml`'s comment) or it crashes on
  startup (`Rebuild the library with ... GTK+ ... support`).
