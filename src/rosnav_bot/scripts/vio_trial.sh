#!/usr/bin/env bash
# VIO trial (concepts.md sec 44b): record ONE cafe weave run with three visual estimators live
#   - ORB-SLAM3 RGB-D-inertial (VIO)        -> /orb_vio/pose
#   - ORB-SLAM3 RGB-D (same code, no IMU)   -> /orb_vo/pose      (ablation: does the IMU help?)
#   - RTAB-Map rgbd_odometry (camera only)  -> /rtab_vo/odom
# Needs ~/orb_slam3_ws (docker/orb_slam3/build_bare_metal.sh) and ros-humble-rtabmap-ros.
# GPLv3 note: ORB-SLAM3 stays outside this repo's build (same isolation as the sidecar).
# Own ROS_DOMAIN_ID + GZ_PARTITION; kills only processes tagged with that partition.
# Usage: vio_trial.sh [DUR=150]   Output: $OUT (default ~/vio_trial/<stamp>)
set -u
DUR=${1:-150}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-162}; export GZ_PARTITION=vio_$ROS_DOMAIN_ID
OUT=${OUT:-$HOME/vio_trial/$(date +%Y%m%d_%H%M)}; mkdir -p "$OUT"
HERE=$(cd "$(dirname "$0")/../../.." && pwd)            # repo root
PAR=$HERE/docker/orb_slam3/params
VOC=${ORB_VOC:-$HOME/orb_slam3_src/ORB_SLAM3/Vocabulary/ORBvoc.txt}
log(){ echo "[vio_trial $(date +%T)] $*" | tee -a "$OUT/run.log"; }
MAXWAIT=${MAXWAIT:-2400}; t0=$SECONDS
log "waiting for free machine (no gz sim, load<6, max ${MAXWAIT}s)"
while pgrep -f "gz sim" >/dev/null || [ "$(awk '{print int($1)}' /proc/loadavg)" -ge 6 ]; do
  [ $((SECONDS-t0)) -gt $MAXWAIT ] && { log "gave up waiting"; exit 2; }; sleep 20; done
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"   # Pangolin + OpenCV 4.4 from build_bare_metal.sh
set +u; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"; source "$HOME/orb_slam3_ws/install/setup.bash"; set -u
PIDS=()
mine_kill(){ for d in /proc/[0-9]*; do p=${d#/proc/}
    tr '\0' '\n' < $d/environ 2>/dev/null | grep -qx "GZ_PARTITION=$GZ_PARTITION" && kill -9 "$p" 2>/dev/null
  done; }
trap 'for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 4; for p in "${PIDS[@]}"; do kill -9 -- -$p 2>/dev/null; done; mine_kill' EXIT
log "domain $ROS_DOMAIN_ID partition $GZ_PARTITION out $OUT"
setsid timeout $((DUR+200)) ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe enable_rgbd:=true slam:=false headless:=true rviz:=False > "$OUT/sim.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 60
log "sim topics:"; ros2 topic list 2>/dev/null | grep -E "camera|imu" | tee -a "$OUT/run.log"
ros2 topic hz /imu --window 50 > "$OUT/hz_imu.log" 2>&1 & H1=$!
ros2 topic hz /camera/image_raw --window 20 > "$OUT/hz_cam.log" 2>&1 & H2=$!
setsid ros2 run ros_gz_bridge parameter_bridge "/model/diff_drive/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance" --ros-args -r /model/diff_drive/odometry_with_covariance:=/gt_odom > "$OUT/gt.log" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run orb_slam3_ros2_wrapper rgbd_imu "$VOC" "$PAR/rosnav_rgbd_imu.yaml" --ros-args --params-file "$PAR/rosnav_vio_trial_ros_params.yaml" -p use_sim_time:=true -r __node:=orb_vio -r robot_pose_slam:=/orb_vio/pose > "$OUT/orb_vio.log" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run orb_slam3_ros2_wrapper rgbd "$VOC" "$PAR/rosnav_rgbd.yaml" --ros-args --params-file "$PAR/rosnav_vio_trial_ros_params.yaml" -p use_sim_time:=true -r __node:=orb_vo -r robot_pose_slam:=/orb_vo/pose > "$OUT/orb_vo.log" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run rtabmap_odom rgbd_odometry --ros-args -r rgb/image:=/camera/image_raw -r depth/image:=/camera/depth/image_raw -r rgb/camera_info:=/camera/camera_info -r odom:=/rtab_vo/odom -p use_sim_time:=true -p frame_id:=base_link -p publish_tf:=false -p approx_sync:=true -p qos:=1 -p Reg/Force3DoF:="'true'" > "$OUT/rtab_vo.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 15
setsid ros2 bag record -o "$OUT/bag" --use-sim-time /clock /imu /gt_odom /odom /orb_vio/pose /orb_vo/pose /rtab_vo/odom > "$OUT/bag.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 5
log "weave ${DUR}s (0.5 m/s, +-0.8 rad/s every 4 s)"
end=$((SECONDS+DUR)); sgn=1
while [ $SECONDS -lt $end ]; do
  timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: $(echo "$sgn*0.8" | bc -l)}}" >/dev/null 2>&1
  sgn=$((-sgn))
done
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
sleep 3; kill -INT "${PIDS[-1]}" 2>/dev/null; sleep 5
kill $H1 $H2 2>/dev/null
log "done. bag: $OUT/bag"; ros2 bag info "$OUT/bag" 2>&1 | tee -a "$OUT/run.log" | head -20
