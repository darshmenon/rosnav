#!/usr/bin/env bash
# benchmark_scaling.sh — exploration coverage vs fleet size (multi_robot.launch.py).
# Finds where adding robots stops helping (TF/DDS/CPU saturation, frontier contention).
# Optional ALLOC="greedy hungarian" crosses each size with both assignment modes.
#
# Usage: SIZES="1 2 4 8" WORLD=cafe DURATION_S=240 ./scripts/benchmark_scaling.sh
# Output: $OUT_DIR/scaling.md (+ per-run JSON). Loadavg is sampled to show CPU saturation.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/common_bench.sh"
bench_env_init
WORLD="${WORLD:-cafe}"; SIZES="${SIZES:-1 2 4 8}"; ALLOC="${ALLOC:-greedy}"
DURATION_S="${DURATION_S:-240}"; BOOT_WAIT_S="${BOOT_WAIT_S:-90}"
OUT_DIR="${OUT_DIR:-/tmp/rosnav_scaling/$(date +%Y%m%d_%H%M%S)_$WORLD}"
mkdir -p "$OUT_DIR"; PGID=""
trap 'domain_kill_all' EXIT

for n in $SIZES; do for a in $ALLOC; do
  tag="n${n}_${a}"; dir="$OUT_DIR/$tag"; mkdir -p "$dir"
  bench_domain; export ROS_HOME="$dir/ros_home"; mkdir -p "$ROS_HOME/log"
  echo "[scaling] robots=$n alloc=$a domain=$ROS_DOMAIN_ID"
  setsid ros2 launch rosnav_bot multi_robot.launch.py world_name:="$WORLD" robot_count:="$n" \
      robot_layout:=grid headless:=true rviz:=false explore:=true allocation:="$a" \
      >"$dir/launch.log" 2>&1 &
  if wait_topics /clock; then
    sleep 45   # let Nav2 + SLAM come up on every robot before measuring
    ( while :; do echo "$(date +%s) $(cut -d' ' -f1 /proc/loadavg)"; sleep 5; done ) >"$dir/load.txt" &
    LP=$!
    timeout $((DURATION_S + 45)) ros2 run rosnav_bot benchmark.py --ros-args -p mode:=slam \
        -p label:="$tag" -p map_topic:=/map_merged -p duration_sec:="$DURATION_S" \
        -p out_dir:="$dir" -p use_sim_time:=true >>"$dir/launch.log" 2>&1 || echo "  WARN bench rc!=0"
    kill "$LP" 2>/dev/null || true
    rep="$(ls -1 "$dir"/*_slam.json 2>/dev/null | head -1 || true)"
    [[ -n "$rep" ]] && cp -f "$rep" "$dir/summary.json" || echo '{"failed":true}' >"$dir/summary.json"
  else
    echo '{"failed":true,"reason":"boot"}' >"$dir/summary.json"
  fi
  domain_kill_all
done; done

python3 -I - "$OUT_DIR" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
rows = ['| robots | alloc | coverage % | free cells | peak load1 | note |', '|---|---|---|---|---|---|']
for d in sorted(out.iterdir(), key=lambda p: (len(p.name), p.name)):
    f = d / 'summary.json'
    if not f.is_file(): continue
    s = json.loads(f.read_text()); n, a = d.name[1:].split('_', 1)
    loads = [float(l.split()[1]) for l in (d / 'load.txt').read_text().splitlines()] if (d / 'load.txt').is_file() else []
    rows.append(f"| {n} | {a} | {s.get('final_coverage_pct','-')} | {s.get('final_free_cells','-')} | "
                f"{max(loads) if loads else '-'} | {'FAILED' if s.get('failed') else ''} |")
(out / 'scaling.md').write_text('\n'.join(rows) + '\n'); print('\n'.join(rows))
PY
