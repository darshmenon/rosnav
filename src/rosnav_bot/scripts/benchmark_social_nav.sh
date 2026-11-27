#!/bin/bash
# benchmark_social_nav.sh — Nav2 among social-force pedestrians: plain vs CV mask vs learned mask.
# Usage: scripts/benchmark_social_nav.sh [world] [config...]     config = plain | cv | learned
#   plain   = Nav2 as-is (reactive obstacle_layer only)
#   cv      = + dynobs KeepoutFilter fed by dynamic_obstacle_predictor.py (constant-velocity shadow)
#   learned = + dynobs KeepoutFilter fed by social_nav_node.py (learned predictor, widening discs)
# Same pedestrian seed + same waypoint loop for every config; one fresh launch per config.
# Env: ROS_DOMAIN_ID (default 205), OUT (/tmp/social_nav_<world>), PEDS (10), DURATION (120 s),
#      SEED (0), WEIGHTS (outputs/ped_predictor_n24.pt)
#      RVIZ=1  watch it live: opens RViz (social_nav.rviz: people, predicted paths, mask) BEFORE the sim
#              and shows the Gazebo window (headless:=false). Default is headless, no RViz.
set -u
source "$(dirname "${BASH_SOURCE[0]}")/bench_load_guard.sh"; bench_load_guard
WORLD=${1:-hospital}; shift 1 2>/dev/null || true
CONFIGS=${*:-plain cv learned}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-205}; export GZ_PARTITION=${GZ_PARTITION:-soc$ROS_DOMAIN_ID}
REPO=$(cd "$(dirname "$0")/../../.." && pwd); OUT=${OUT:-/tmp/social_nav_$WORLD}; mkdir -p "$OUT"
PEDS=${PEDS:-10}; DURATION=${DURATION:-120}; SEED=${SEED:-0}
WEIGHTS=${WEIGHTS:-$REPO/outputs/ped_predictor_n24.pt}
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
SHARE=$(ros2 pkg prefix rosnav_bot)/share/rosnav_bot
GOALS=$OUT/goals.yaml
printf 'waypoints:\n  - [6.0, 0.0, 0.0]\n  - [-6.0, 0.0, 180.0]\n  - [6.0, 0.5, 0.0]\n  - [-6.0, -0.5, 180.0]\n' > "$GOALS"
log(){ echo "[social_nav $(date +%T)] $*" | tee -a "$OUT/bench.log"; }
kill_domain(){   # exact PIDs on THIS domain only (never name patterns: the machine is shared)
  for round in 1 2; do for p in $(ps -eo pid=); do
    [ "$p" = "$$" ] && continue; [ -r /proc/$p/environ ] || continue
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || continue
    case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in *benchmark_social_nav*|*claude*) continue;; esac
    kill -INT "$p" 2>/dev/null; done; sleep 3; done
  for p in $(ps -eo pid=); do [ -r /proc/$p/environ ] || continue
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || continue
    case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in *benchmark_social_nav*|*claude*) continue;; esac
    kill -9 "$p" 2>/dev/null; done
  "$REPO/src/rosnav_bot/scripts/clean_fastdds_shm.sh" --delete >/dev/null 2>&1 || true
}
trap kill_domain EXIT
for C in $CONFIGS; do
  case $C in plain) PIPE=none;; cv) PIPE=cv;; learned) PIPE=external;; *) log "unknown config $C"; continue;; esac
  kill_domain; rm -f "$OUT/$C".*
  log "config=$C (dynobs_pipeline=$PIPE) peds=$PEDS seed=$SEED"
  HL=true; [ "${RVIZ:-0}" = 1 ] && HL=false
  if [ "${RVIZ:-0}" = 1 ]; then
    setsid rviz2 -d "$REPO/src/rosnav_bot/rviz/social_nav.rviz" > "$OUT/$C.rviz.log" 2>&1 < /dev/null &
    sleep 15                                  # RViz first, let it load, then the sim
  fi
  setsid ros2 launch rosnav_bot slam_nav.launch.py world_name:=$WORLD slam:=false headless:=$HL rviz:=False \
      dynobs_pipeline:=$PIPE max_speed:=${SPEED:-0.5} > "$OUT/$C.launch.log" 2>&1 < /dev/null &
  # wait until Nav2 is really up (not a fixed sleep: under load the stack took >80 s and runs recorded nothing)
  up=0; for i in $(seq 1 60); do
    if grep -q "Nav2 active" "$OUT/$C.launch.log" 2>/dev/null; then up=1; break; fi
    sleep 5; done
  [ $up = 1 ] && log "nav2 active after ~$((i*5)) s" || { log "nav2 NOT active after 300 s -> skipping $C"; kill_domain; continue; }
  sleep 10
  setsid ros2 run rosnav_bot social_force_peds.py --ros-args -p use_sim_time:=false -p world:=$WORLD \
      -p map:=$SHARE/maps/map_$WORLD.yaml -p n:=$PEDS -p seed:=$SEED \
      -p offset_x:=${SPAWN_X:-1.5} -p offset_y:=${SPAWN_Y:-1.0} > "$OUT/$C.peds.log" 2>&1 < /dev/null &
  if [ "$C" = learned ]; then
    setsid /usr/bin/python3 "$REPO/src/rosnav_bot/scripts/social_nav_node.py" --ros-args -p use_sim_time:=true \
        -p weights:=$WEIGHTS -p mask_topic:=/dynobs/keepout_mask > "$OUT/$C.social.log" 2>&1 < /dev/null &
  fi
  sleep $((PEDS * 3 + 15))
  setsid ros2 run rosnav_bot waypoint_nav.py --ros-args -p use_sim_time:=true -p waypoints_file:=$GOALS \
      -p loop:=true -p loop_count:=-1 > "$OUT/$C.wp.log" 2>&1 < /dev/null &
  ros2 run rosnav_bot social_metrics.py --ros-args -p duration:=$(printf "%.1f" "$DURATION") -p label:=$C -p out:=$OUT/$C.json \
      > "$OUT/$C.metrics.log" 2>&1
  log "  $(grep RESULT "$OUT/$C.metrics.log" | sed 's/.*RESULT //')"
  kill_domain
done
python3 -I - "$OUT" $CONFIGS <<'PY'
import json,sys
out=sys.argv[1]; rows=[]
for c in sys.argv[2:]:
    try: d=json.load(open(f'{out}/{c}.json'))
    except Exception: rows.append(f'| {c} | n/a | n/a | n/a | n/a | n/a | n/a |'); continue
    rows.append(f"| {c} | {d['collisions']} ({d.get('moving_collisions','-')} moving) | {d['min_clearance_m']} | {d['intrusion_s']} | {d['path_m']} | {d['mean_speed']} | {d['smoothness']} |")
print('| Config | Collisions | Min clearance m | Intrusion s | Path m | Mean speed | Smoothness |\n|---|---|---|---|---|---|---|\n'+'\n'.join(rows))
PY
