#!/bin/bash
# lio_stamp_sweep.sh - sweep the lidar adapter's stamp_shift_s for LIO-SAM against Gazebo ground truth.
# One sim (3D lidar); per shift: restart ONLY the LIO-SAM launch, let it initialise with the robot STILL for 25 s (LIO-SAM diverges
# if it starts mid-motion: the first sweep restarted it while the robot was driving and every shift blew up), then drive a fig8 with
# the obstacle guard (sim time), score MEASURE s with localization_accuracy_bench and run lio_doctor --ref-odom /gt_odom.
# Usage: scripts/lio_stamp_sweep.sh "-0.2 -0.15 -0.1 -0.05 0.0"   Env: ROS_DOMAIN_ID (default 93), OUT, MEASURE (90)
set -u
SHIFTS=${1:--0.2 -0.15 -0.1 -0.05 0.0}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}; export GZ_PARTITION=${GZ_PARTITION:-sweep$ROS_DOMAIN_ID}
REPO=$(cd "$(dirname "$0")/../../.." && pwd); OUT=${OUT:-/tmp/lio_sweep}; MEASURE=${MEASURE:-90}; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; source "$HOME/lio_ws/install/setup.bash"; set -u
export PYTHONPATH="$REPO/src/rosnav_bot/tools/lio_doctor:${PYTHONPATH:-}"
log(){ echo "[lio_sweep $(date +%T)] $*" | tee -a "$OUT/sweep.log"; }
PGIDS=()
cleanup(){ for p in "${PGIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 8; for p in "${PGIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done; }
trap cleanup EXIT
setsid ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe lidar_type:=3d slam:=false headless:=true rviz:=False > "$OUT/sim.log" 2>&1 < /dev/null & PGIDS+=($!)
sleep 60
setsid ros2 run ros_gz_bridge parameter_bridge "/model/diff_drive/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance" \
    --ros-args -r /model/diff_drive/odometry_with_covariance:=/gt_odom > "$OUT/gt.log" 2>&1 < /dev/null & PGIDS+=($!)
for s in $SHIFTS; do
  log "stamp_shift_s=$s: restarting LIO-SAM"
  setsid ros2 launch rosnav_bot lio_sam_sim.launch.py stamp_shift_s:=$s > "$OUT/lio_$s.log" 2>&1 < /dev/null & L=$!
  sleep 25          # robot still: LIO-SAM initialises
  setsid python3 "$REPO/src/rosnav_bot/scripts/drive_pattern.py" --ros-args -p use_sim_time:=true -p pattern:=fig8 \
      -p duration:=$((MEASURE + 20)).0 > "$OUT/drive_$s.log" 2>&1 < /dev/null & DR=$!
  timeout $((MEASURE + 30)) python3 -m lio_doctor.doctor --imu /imu_rot --cloud /points_lio --ref-odom /gt_odom --interval $MEASURE \
      --duration $MEASURE --once --json > "$OUT/doctor_$s.json" 2> /dev/null &
  D=$!
  ros2 run rosnav_bot localization_accuracy_bench.py --ros-args -p use_sim_time:=true -p duration:=$MEASURE.0 \
      -p model:=diff_drive -p out:="$OUT/result_$s.json" > "$OUT/bench_$s.out" 2>&1
  wait $D 2>/dev/null
  RTF=$(gz topic -e -n 1 -t /stats 2>/dev/null | grep -m1 real_time_factor | awk '{print $2}')
  log "stamp_shift_s=$s done, rtf ${RTF:-?}, result: $(python3 -c "import json;d=json.load(open('$OUT/result_$s.json'));e=d['estimators']['lio_sam'];print('path',d['ground_truth_path_m'],'lio_sam rmse',e['rmse_m'],'drift%',e['final_pct_of_path'],'yaw',e['yaw_rmse_deg'])" 2>/dev/null || echo NO-RESULT)"
  kill -INT -- -$DR 2>/dev/null; kill -INT -- -$L 2>/dev/null; sleep 10; kill -KILL -- -$L 2>/dev/null; kill -KILL -- -$DR 2>/dev/null
done
log "sweep finished"
