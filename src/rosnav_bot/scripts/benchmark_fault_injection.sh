#!/usr/bin/env bash
# benchmark_fault_injection.sh — kill a component mid-exploration and see whether the
# fleet still maps. Each fault run is compared to a no-fault baseline (same world/seed).
# Faults are process kills scoped to this run's ROS_DOMAIN_ID (never pkill by name).
#
#   nav_crash   controller_server of robot2   (one robot loses motion control)
#   planner     planner_server of robot2
#   slam_crash  slam_toolbox (robot1 is the SLAM host in multi-robot mode)
#   scan_gate   scan_quality_gate (all robots lose /scan)
#   coord       frontier_coordinator (no more goals assigned)
#
# Usage: FAULTS="nav_crash slam_crash" WORLD=cafe FAULT_AT=90 DURATION_S=240 ./scripts/benchmark_fault_injection.sh
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/common_bench.sh"
bench_env_init
WORLD="${WORLD:-cafe}"; FAULTS="${FAULTS:-nav_crash planner slam_crash scan_gate coord}"
FAULT_AT="${FAULT_AT:-90}"; DURATION_S="${DURATION_S:-240}"; BOOT_WAIT_S="${BOOT_WAIT_S:-90}"
OUT_DIR="${OUT_DIR:-/tmp/rosnav_faults/$(date +%Y%m%d_%H%M%S)_$WORLD}"
mkdir -p "$OUT_DIR"
trap 'domain_kill_all' EXIT

pattern_for() {
  case "$1" in
    nav_crash)  echo 'controller_server.*__ns:=/robot2' ;;
    planner)    echo 'planner_server.*__ns:=/robot2' ;;
    slam_crash) echo 'slam_toolbox' ;;
    scan_gate)  echo 'scan_quality_gate' ;;
    coord)      echo 'frontier_coordinator' ;;
    none)       echo '' ;;
    *) echo "unknown fault $1" >&2; return 1 ;;
  esac
}

for fault in none $FAULTS; do
  dir="$OUT_DIR/$fault"; mkdir -p "$dir"
  bench_domain; export ROS_HOME="$dir/ros_home"; mkdir -p "$ROS_HOME/log"
  pat="$(pattern_for "$fault")"
  echo "[fault] $fault domain=$ROS_DOMAIN_ID"
  setsid ros2 launch rosnav_bot multi_robot.launch.py world_name:="$WORLD" robot_count:=2 \
      headless:=true rviz:=false explore:=true >"$dir/launch.log" 2>&1 &
  if wait_topics /clock; then
    sleep 45
    (
      sleep "$FAULT_AT"
      if [[ -n "$pat" ]]; then
        n="$(domain_kill_match "$pat" 2>/dev/null || true)"
        echo "$(date +%T) killed $n procs matching /$pat/" >"$dir/fault.txt"
      fi
    ) &
    timeout $((DURATION_S + 45)) ros2 run rosnav_bot benchmark.py --ros-args -p mode:=slam \
        -p label:="$fault" -p map_topic:=/map_merged -p duration_sec:="$DURATION_S" \
        -p out_dir:="$dir" -p use_sim_time:=true >>"$dir/launch.log" 2>&1 || echo "  WARN bench rc!=0"
    rep="$(ls -1 "$dir"/*_slam.json 2>/dev/null | head -1 || true)"
    [[ -n "$rep" ]] && cp -f "$rep" "$dir/summary.json" || echo '{"failed":true}' >"$dir/summary.json"
  else
    echo '{"failed":true,"reason":"boot"}' >"$dir/summary.json"
  fi
  domain_kill_all
done

python3 -I - "$OUT_DIR" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
def cov(n):
    f = out / n / 'summary.json'
    return None if not f.is_file() else json.loads(f.read_text()).get('final_coverage_pct')
base = cov('none')
rows = ['| fault | coverage % | vs baseline | fault note |', '|---|---|---|---|']
for d in sorted(out.iterdir()):
    if not d.is_dir(): continue
    c = cov(d.name); note = (d / 'fault.txt').read_text().strip() if (d / 'fault.txt').is_file() else ''
    rel = f'{100*c/base:.0f}%' if c is not None and base else '-'
    rows.append(f'| {d.name} | {c if c is not None else "FAILED"} | {rel} | {note} |')
(out / 'faults.md').write_text('\n'.join(rows) + '\n'); print('\n'.join(rows))
PY
