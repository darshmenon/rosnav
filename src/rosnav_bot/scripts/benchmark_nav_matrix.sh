#!/bin/bash
# benchmark_nav_matrix.sh — fixed-goal Nav2 benchmark over planner x controller combos on a saved map
# (AMCL, slam:=false). Same goals for every combo; one fresh launch per combo.
# Usage: scripts/benchmark_nav_matrix.sh [world] [combo...]   combo = <planner>:<controller>
#   e.g. scripts/benchmark_nav_matrix.sh cafe navfn:dwb smac2d:dwb navfn:mppi
# Env: ROS_DOMAIN_ID (default 204, isolated), OUT (default /tmp/nav_matrix_<world>)
#      EXTRA_LAUNCH  extra launch args, e.g. EXTRA_LAUNCH='dynamic_obstacles:=2' (moving obstacles)
set -u
WORLD=${1:-cafe}; shift 1 2>/dev/null || true
COMBOS=${*:-navfn:dwb navfn_dijkstra:dwb smac2d:dwb thetastar:dwb navfn:mppi navfn:rpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-204}; export GZ_PARTITION=${GZ_PARTITION:-nav$ROS_DOMAIN_ID}
REPO=$(cd "$(dirname "$0")/../../.." && pwd); OUT=${OUT:-/tmp/nav_matrix_$WORLD}; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
GOALS=$OUT/goals.yaml
printf 'waypoints:\n  - [1.5, 0.0, 0.0]\n  - [2.5, 0.0, 0.0]\n  - [2.0, -1.0, 0.0]\n' > "$GOALS"
log(){ echo "[nav_matrix $(date +%T)] $*" | tee -a "$OUT/matrix.log"; }
kill_domain(){
  for round in 1 2; do for p in $(ps -eo pid=); do
    [ "$p" = "$$" ] && continue; [ -r /proc/$p/environ ] || continue
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || continue
    case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in *benchmark_nav_matrix*|*claude*) continue;; esac
    kill -9 "$p" 2>/dev/null; done; sleep 2; done
  # kill -9 leaves Fast DDS shm segments; stale ones made the next combo's controller lose map->odom
  "$REPO/src/rosnav_bot/scripts/clean_fastdds_shm.sh" --delete >/dev/null 2>&1 || true
}
for C in $COMBOS; do
  P=${C%%:*}; K=${C##*:}; L=${P}_${K}
  kill_domain; rm -f "$OUT/$L".*
  log "combo planner=$P controller=$K"
  setsid timeout 420 ros2 launch rosnav_bot slam_nav.launch.py world_name:=$WORLD slam:=false \
      planner:=$P controller:=$K headless:=true rviz:=False ${EXTRA_LAUNCH:-} > "$OUT/$L.launch.log" 2>&1 < /dev/null &
  sleep 75
  timeout 300 ros2 run rosnav_bot benchmark.py --ros-args -p use_sim_time:=true -p mode:=nav -p label:=$L \
      -p goals_file:=$GOALS -p out_dir:=$OUT > "$OUT/$L.bench.log" 2>&1
  log "  done: $(grep -E 'goal [0-9]+/' "$OUT/$L.bench.log" | sed 's/.*\[nav\] //' | tr '\n' ';')"
  kill_domain
done
python3 -I - "$OUT" $COMBOS <<'PY'
import json,sys
out=sys.argv[1]; rows=[]
for c in sys.argv[2:]:
    p,k=c.split(':'); f=f'{out}/{p}_{k}_nav.json'
    try: d=json.load(open(f))
    except Exception: rows.append(f'| {p} | {k} | n/a | n/a | n/a | n/a |'); continue
    rows.append(f"| {p} | {k} | {d['goals_succeeded']}/{d['goals_total']} | {d['avg_speed_mps']} | {d['avg_goal_error_m']} | {d['total_recoveries']} |")
print('| Planner | Controller | Goals ok | Avg speed m/s | Avg goal err m | Recoveries |\n|---|---|---|---|---|---|\n'+'\n'.join(rows))
PY
