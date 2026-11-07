#!/bin/bash
# benchmark_kiss_icp.sh — KISS-ICP lidar odometry vs wheel odom / EKF vs Gazebo ground truth.
# Needs: kiss-icp built in the workspace (src/kiss-icp), lidar_type:=3d (publishes /points).
# Drives a slow circle for DUR s; localization_accuracy_bench.py rigidly aligns each estimator at
# the first sample and reports RMSE / final drift (% of path) per estimator, incl. kiss_icp.
# Usage: scripts/benchmark_kiss_icp.sh [world]      Env: ROS_DOMAIN_ID (default 206), OUT, DUR
set -u
WORLD=${1:-cafe}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-206}; export GZ_PARTITION=${GZ_PARTITION:-kiss$ROS_DOMAIN_ID}
REPO=$(cd "$(dirname "$0")/../../.." && pwd); OUT=${OUT:-/tmp/kiss_bench_$WORLD}; DUR=${DUR:-120}; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
log(){ echo "[kiss_bench $(date +%T)] $*" | tee -a "$OUT/bench.log"; }
ros2 pkg prefix kiss_icp >/dev/null 2>&1 || { log "kiss_icp not built — see colcon build --packages-select kiss_icp"; exit 1; }
PIDS=()
cleanup(){ for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done; "$REPO/src/rosnav_bot/scripts/clean_fastdds_shm.sh" --delete >/dev/null 2>&1 || true; }
trap cleanup EXIT
log "launching sim (world=$WORLD lidar_type=3d, domain $ROS_DOMAIN_ID)"
setsid timeout 600 ros2 launch rosnav_bot slam_nav.launch.py world_name:=$WORLD lidar_type:=3d slam:=false \
    headless:=true rviz:=False > "$OUT/sim.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 60
log "starting KISS-ICP (tf off — keeps existing odom->base_link)"
setsid ros2 launch kiss_icp odometry.launch.py topic:=/points base_frame:=base_link publish_odom_tf:=false \
    visualize:=false use_sim_time:=true > "$OUT/kiss.log" 2>&1 < /dev/null & PIDS+=($!)
MODEL=${MODEL:-diff_drive}
setsid ros2 run ros_gz_bridge parameter_bridge \
    "/model/$MODEL/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance" \
    --ros-args -r /model/$MODEL/odometry_with_covariance:=/gt_odom > "$OUT/gt.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 10
setsid timeout $((DUR+20)) ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2}, angular: {z: 0.1}}" > /dev/null 2>&1 < /dev/null & PIDS+=($!)
log "measuring for ${DUR}s"
ros2 run rosnav_bot localization_accuracy_bench.py --ros-args -p use_sim_time:=true \
    -p duration:=${DUR}.0 -p model:=$MODEL -p out:="$OUT/result.json" > "$OUT/bench.out" 2>&1
log "result: $(cat "$OUT/result.json" 2>/dev/null || echo NO-RESULT)"
