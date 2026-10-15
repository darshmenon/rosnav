#!/bin/bash
# benchmark_vslam_variants.sh — run slam variants one at a time on the same world/route and
# compare map quality + stability.  Variants:
#   A,B,C = slam_algo:=vslam with vslam_variant A/B/C (see _common.rtabmap_vslam_node)
#   D     = slam_algo:=2d (slam_toolbox, lidar) baseline
#   O     = slam_algo:=orbslam3 (bare-metal sidecar from ~/orb_slam3_ws, default params)
#   O-<stem> = same, with docker/orb_slam3/params/<stem>.yaml swapped in as the ORB config (tuning runs)
# Usage: scripts/benchmark_vslam_variants.sh [world] [seconds_per_variant] [variants...]
#   e.g. scripts/benchmark_vslam_variants.sh cafe 420 A B C D
# Env: ROS_DOMAIN_ID (default 201 — NOT the shared default; kills only this domain's processes)
# Output: $OUT (default /tmp/vslam_bench_<world>/) per-variant log + map, then
#         docs/benchmarks/BENCHMARK_VSLAM_VARIANTS.md.
set -u
WORLD=${1:-cafe}; SECS=${2:-420}; shift 2 2>/dev/null || shift $# ; VARIANTS=${*:-A B C D}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-201}
export GZ_PARTITION=${GZ_PARTITION:-bench$ROS_DOMAIN_ID}   # isolate gz-transport too (parallel runs)
REPO=$(cd "$(dirname "$0")/../../.." && pwd); OUT=${OUT:-/tmp/vslam_bench_$WORLD}; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
log(){ echo "[bench $(date +%T)] $*" | tee -a "$OUT/bench.log"; }

kill_domain(){  # only processes whose environ has OUR ROS_DOMAIN_ID — never another agent's sim
  for round in 1 2 3; do
    for p in $(ps -eo pid=); do
      [ "$p" = "$$" ] && continue
      [ -r /proc/$p/environ ] || continue
      tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || continue
      case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in *benchmark_vslam_variants*|*claude*) continue;; esac
      kill -9 "$p" 2>/dev/null
    done; sleep 2
  done
}
num(){ grep -m1 -E '^-?[0-9.e+-]+$'; }
odom_x(){ timeout 8 ros2 topic echo /odom --once --field pose.pose.position.x 2>/dev/null | num; }

ORB_WS=${ORB_WS:-$HOME/orb_slam3_ws}
ORB_CFG=$ORB_WS/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/gazebo_rgbd.yaml   # path hardcoded in rgbd.launch.py
start_orb(){  # bare-metal ORB-SLAM3 sidecar; O-<stem> swaps docker/orb_slam3/params/<stem>.yaml in as its config
  local V=$1 stem=${1#O-}
  [ -f "$ORB_CFG.orig" ] || cp "$ORB_CFG" "$ORB_CFG.orig"
  if [ "$V" = O ]; then cp "$ORB_CFG.orig" "$ORB_CFG"; else cp "$REPO/docker/orb_slam3/params/$stem.yaml" "$ORB_CFG"; fi
  log "  $V: starting ORB-SLAM3 sidecar (cfg=$([ "$V" = O ] && echo default || echo $stem))"
  ( set +u; source "$ORB_WS/install/setup.bash"; export LD_LIBRARY_PATH="${ORB_SLAM3_PREFIX:-$HOME/.local}/lib:${LD_LIBRARY_PATH:-}"   # Pangolin/OpenCV 4.4
    setsid ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py sensor_config:=rgbd > "$OUT/$V.orb.log" 2>&1 < /dev/null & )
}

run_variant(){
  local V=$1 ARGS
  case $V in
    A|B|C|E|F|H) ARGS="slam_algo:=vslam vslam_variant:=$V rtabmap_db:=$OUT/$V.db";;
    D)     ARGS="slam_algo:=2d";;
    O|O-*) ARGS="slam_algo:=orbslam3";;
    G)     ARGS="slam_algo:=cartographer";;
    M)     ARGS="slam_algo:=multisensor rtabmap_db:=$OUT/$V.db";;
    # C-<feat>: variant C with a specific visual feature extractor (orb sift surf gftt_brief gftt_orb fast_brief ...)
    C-*)   ARGS="slam_algo:=vslam vslam_variant:=C vslam_features:=${V#C-} rtabmap_db:=$OUT/$V.db";;
    # P-<planner> / K-<controller>: lidar slam_toolbox + chosen Nav2 global planner / local controller
    S-*)   ARGS="slam_algo:=2d slam_params_file:=${V#S-}";;   # S-<stem>: slam_toolbox with config/<stem>.yaml
    P-*)   ARGS="slam_algo:=2d planner:=${V#P-}";;
    K-*)   ARGS="slam_algo:=2d controller:=${V#K-}";;
    *) log "unknown variant $V"; return;;
  esac
  rm -f "$OUT/$V.db" "$OUT/$V.log" "$OUT/$V.acc.log"
  kill_domain
  log "variant $V: ros2 launch ... $ARGS (${SECS}s)"
  setsid timeout $((SECS+60)) ros2 launch rosnav_bot slam_nav.launch.py world_name:=$WORLD $ARGS \
        explore:=true headless:=true rviz:=False slam_debug:=true > "$OUT/$V.log" 2>&1 < /dev/null &
  sleep 100   # world bake + Gazebo + Nav2 bring-up
  case $V in O|O-*) start_orb "$V";; esac
  setsid ros2 run rosnav_bot slam_accuracy_monitor.py --ros-args -p use_sim_time:=true \
        -p ground_truth_topic:=/ground_truth -p ground_truth_type:=odometry > "$OUT/$V.acc.log" 2>&1 < /dev/null &
  local t=100 nudges=0 prev_nodes=-1 prev_geom=""
  while [ $t -lt $SECS ]; do
    # Log-based stuck detection (no ROS CLI calls): RTAB-Map publishes no /map until the robot
    # has moved once, and explore_lite then just fires trivial "goals" at the robot's own cell.
    # Nudge while the map stream is empty (msgs=0) or the explorer is spinning in place.
    maplast=$(grep "slam_debug_map" "$OUT/$V.log" | tail -1)
    msgs=$(echo "$maplast" | grep -oE "msgs=[0-9]+" | cut -d= -f2); msgs=${msgs:-0}
    tail1=$(grep -c "Goal succeeded" "$OUT/$V.log")
    # slam_toolbox publishes /map from t=0, so msgs>=3 never trips the check above while the
    # robot sits at spawn; also nudge when the map geometry hasn't changed since the last check.
    geom=$(echo "$maplast" | grep -oE "geom=[0-9x@(.,-]+\)")
    stuck=0; [ -n "$geom" ] && [ "$geom" = "$prev_geom" ] && stuck=1; prev_geom=$geom
    if { [ "$msgs" -lt 3 ] || [ $stuck -eq 1 ]; } && [ $nudges -lt 12 ]; then
      nudges=$((nudges+1)); log "  $V: /map msgs=$msgs stuck=$stuck -> nudge $nudges (explorer goals ok=$tail1)"
      timeout 10 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}" >/dev/null 2>&1
    fi
    sleep 25; t=$((t+35))
  done
  echo "$nudges" > "$OUT/$V.nudges"
  echo "$(timeout 8 ros2 topic echo /clock --once --field clock.sec 2>/dev/null | num)" > "$OUT/$V.simsec"   # sim seconds reached (RTF check)
  ros2 run nav2_map_server map_saver_cli -f "$OUT/map_$V" --ros-args -p use_sim_time:=true > "$OUT/$V.mapsave.log" 2>&1
  log "variant $V done (map: $(ls $OUT/map_$V.pgm 2>/dev/null || echo MISSING))"
  kill_domain
}

for V in $VARIANTS; do run_variant "$V"; done
[ "${SKIP_SUMMARY:-0}" = 1 ] || python3 -I "$(dirname "$0")/summarize_vslam_variants.py" "$OUT" "$WORLD" "${REPORT:-$REPO/docs/benchmarks/BENCHMARK_VSLAM_VARIANTS.md}" $VARIANTS
log "all done -> ${REPORT:-$REPO/docs/benchmarks/BENCHMARK_VSLAM_VARIANTS.md}"
