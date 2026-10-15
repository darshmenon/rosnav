#!/bin/bash
# benchmark_localization.sh — kidnapped-robot recovery on a saved map (AMCL, slam:=false).
# Arms:  L = AMCL alone            R = AMCL + RTAB-Map visual reseed (visual_reloc:=true)
#        O = AMCL + ORB-SLAM3 reseed (sidecar pose, aligned by orb_pose_to_amcl.py)
#        B = AMCL + RTAB-Map + ORB-SLAM3 reseed (both)
# Each arm: settle ~40s while driving a slow circle, kidnap via /initialpose (+2m,+1m,+90deg),
# then kidnap_eval.py logs time-to-recover (<0.5m for 3s) vs ground truth. gs_reloc/color_error
# is logged too if gs_relocalization_check.py is running (GS=<npz path> starts it).
# Usage: scripts/benchmark_localization.sh [world] [arms...]     e.g. cafe L R O B
# Env: ROS_DOMAIN_ID (default 205, isolated), OUT, RTAB_DB (default ~/.ros/rtabmap_<world>.db), GS
set -u
WORLD=${1:-cafe}; shift 1 2>/dev/null || true; ARMS=${*:-L R O B}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-205}; export GZ_PARTITION=${GZ_PARTITION:-loc$ROS_DOMAIN_ID}
REPO=$(cd "$(dirname "$0")/../../.." && pwd); OUT=${OUT:-/tmp/loc_bench_$WORLD}; mkdir -p "$OUT"
RTAB_DB=${RTAB_DB:-$HOME/.ros/rtabmap_$WORLD.db}; ORB_WS=${ORB_WS:-$HOME/orb_slam3_ws}
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
log(){ echo "[loc_bench $(date +%T)] $*" | tee -a "$OUT/bench.log"; }
kill_domain(){
  for round in 1 2 3; do for p in $(ps -eo pid=); do
    [ "$p" = "$$" ] && continue; [ -r /proc/$p/environ ] || continue
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || continue
    case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in *benchmark_localization*|*claude*) continue;; esac
    kill -9 "$p" 2>/dev/null; done; sleep 2; done
  "$REPO/src/rosnav_bot/scripts/clean_fastdds_shm.sh" --delete >/dev/null 2>&1 || true
}
for A in $ARMS; do
  kill_domain; rm -f "$OUT/$A".*
  EXTRA=""; case $A in R|B) EXTRA="visual_reloc:=true rtabmap_db:=$RTAB_DB";; O) EXTRA="enable_rgbd:=true";; esac
  case $A in B) EXTRA="$EXTRA enable_rgbd:=true";; esac
  log "arm $A: $EXTRA"
  setsid timeout 600 ros2 launch rosnav_bot slam_nav.launch.py world_name:=$WORLD slam:=false \
      headless:=true rviz:=False $EXTRA > "$OUT/$A.launch.log" 2>&1 < /dev/null &
  sleep 80
  case $A in O|B)
    ( set +u; source "$ORB_WS/install/setup.bash"; export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
      setsid ros2 launch orb_slam3_ros2_wrapper rgbd.launch.py \
          params_file:=$REPO/docker/orb_slam3/params/rosnav_rgbd_ros_params_notf.yaml > "$OUT/$A.orb.log" 2>&1 < /dev/null &
      setsid ros2 run rosnav_bot orb_pose_to_amcl.py --ros-args -p use_sim_time:=true > "$OUT/$A.orbpose.log" 2>&1 < /dev/null &
      setsid ros2 run rosnav_bot visual_amcl_fusion.py --ros-args -r __node:=orb_amcl_fusion -p use_sim_time:=true \
          -p visual_pose_topic:=/orb_localization_pose > "$OUT/$A.orbfusion.log" 2>&1 < /dev/null & ) ;;
  esac
  [ -n "${GS:-}" ] && setsid ros2 run rosnav_bot gs_relocalization_check.py --ros-args -p use_sim_time:=true -p npz_path:=$GS > "$OUT/$A.gs.log" 2>&1 < /dev/null &
  # slow circle for the whole test so AMCL/ORB see motion (ORB needs it to initialise)
  setsid timeout 190 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.25}}" > /dev/null 2>&1 < /dev/null &
  timeout 170 ros2 run rosnav_bot kidnap_eval.py --ros-args -p use_sim_time:=true -p label:=$A -p out_dir:=$OUT \
      -p settle_sec:=40.0 > "$OUT/$A.eval.log" 2>&1
  log "  $A: $(cat "$OUT/$A.json" 2>/dev/null || echo NO-RESULT)"
  kill_domain
done
python3 -I - "$OUT" $ARMS <<'PY'
import json, sys, pathlib
out = pathlib.Path(sys.argv[1]); name = {'L': 'AMCL alone', 'R': 'AMCL + RTAB-Map', 'O': 'AMCL + ORB-SLAM3', 'B': 'AMCL + RTAB + ORB'}
print('| Arm | Pre err m | After kidnap m | Recovery s | Final err m | GS err pre | GS err post |\n|---|---|---|---|---|---|---|')
for a in sys.argv[2:]:
    f = out / f'{a}.json'
    if not f.exists(): print(f'| {name.get(a, a)} | no result |||||| '); continue
    r = json.loads(f.read_text()); n = lambda k: 'never' if r.get(k) is None and k == 'recovery_sec' else ('n/a' if r.get(k) is None else r[k])
    print(f"| {name.get(a, a)} | {n('pre_err')} | {n('kidnap_err')} | {n('recovery_sec')} | {n('final_err')} | {n('gs_color_err_pre')} | {n('gs_color_err_post')} |")
PY
