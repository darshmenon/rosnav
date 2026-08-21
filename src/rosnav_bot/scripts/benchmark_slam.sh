#!/usr/bin/env bash
# Headless SLAM algorithm matrix — launches each slam_algo, runs
# rosnav_bot/benchmark.py (mode:=slam), then writes a comparison table.
#
# Usage:
#   ./src/rosnav_bot/scripts/benchmark_slam.sh
#   WORLD=maze DURATION_S=90 ./src/rosnav_bot/scripts/benchmark_slam.sh
#   ALGOS="2d cartographer multisensor" ./src/rosnav_bot/scripts/benchmark_slam.sh
#
# Optional: sudo apt install ros-$ROS_DISTRO-cartographer-ros
# Does not commit or push.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"

# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

WORLD="${WORLD:-maze}"
DURATION_S="${DURATION_S:-120}"
BOOT_WAIT_S="${BOOT_WAIT_S:-50}"
LOG_DIR="${LOG_DIR:-/tmp/rosnav_slam_bench}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-$LOG_DIR/$STAMP}"
mkdir -p "$OUT_DIR"

DEFAULT_ALGOS=(
  "2d|slam_algo:=2d"
  "cartographer|slam_algo:=cartographer"
  "vslam|slam_algo:=vslam"
  "multisensor|slam_algo:=multisensor"
  "3d|lidar_type:=3d slam_algo:=3d"
)

if [[ -n "${ALGOS:-}" ]]; then
  VARIANTS=()
  for a in $ALGOS; do
    case "$a" in
      2d) VARIANTS+=("2d|slam_algo:=2d") ;;
      cartographer|carto) VARIANTS+=("cartographer|slam_algo:=cartographer") ;;
      vslam) VARIANTS+=("vslam|slam_algo:=vslam") ;;
      multisensor|multi) VARIANTS+=("multisensor|slam_algo:=multisensor") ;;
      3d) VARIANTS+=("3d|lidar_type:=3d slam_algo:=3d") ;;
      *) echo "unknown algo '$a' (2d|cartographer|vslam|multisensor|3d)"; exit 2 ;;
    esac
  done
else
  VARIANTS=("${DEFAULT_ALGOS[@]}")
fi

LAUNCH_PGIDS=()

cleanup_sim() {
  local pgid
  local pgids=("${LAUNCH_PGIDS[@]:-}")
  LAUNCH_PGIDS=()
  for pgid in "${pgids[@]}"; do
    [[ -n "${pgid:-}" ]] || continue
    kill -TERM -"$pgid" 2>/dev/null || true
  done
  sleep 2
  for pgid in "${pgids[@]}"; do
    [[ -n "${pgid:-}" ]] || continue
    kill -KILL -"$pgid" 2>/dev/null || true
  done
}

topic_once() {
  local topic="$1"
  local secs="${2:-8}"
  timeout "$secs" ros2 topic echo "$topic" --once >/dev/null 2>&1
}

wait_boot() {
  local deadline=$((SECONDS + BOOT_WAIT_S))
  while (( SECONDS < deadline )); do
    if topic_once /clock 3 && topic_once /odom 3 && topic_once /scan 3; then
      return 0
    fi
    sleep 2
  done
  return 1
}

pkg_ok() {
  ros2 pkg prefix "$1" >/dev/null 2>&1
}

run_one() {
  local name="$1" extra="$2"
  local domain=$((70 + RANDOM % 80))
  local part="slam_bench_${name}_${domain}"
  local run_dir="$OUT_DIR/$name"
  local log="$run_dir/launch.log"
  local launch_pid pgid
  mkdir -p "$run_dir"

  echo
  echo "======== $name  DOMAIN=$domain  ${DURATION_S}s ========"

  if [[ "$name" == "cartographer" ]] && ! pkg_ok cartographer_ros; then
    echo "  SKIP cartographer (sudo apt install ros-${ROS_DISTRO:-humble}-cartographer-ros)"
    echo '{"label":"cartographer","skipped":true,"reason":"cartographer_ros missing"}' >"$run_dir/summary.json"
    return 0
  fi
  if [[ "$name" =~ ^(vslam|multisensor|3d)$ ]] && ! pkg_ok rtabmap_slam; then
    echo "  SKIP $name (rtabmap_slam missing)"
    echo "{\"label\":\"$name\",\"skipped\":true,\"reason\":\"rtabmap_slam missing\"}" >"$run_dir/summary.json"
    return 0
  fi

  cleanup_sim
  export ROS_DOMAIN_ID="$domain"
  export GZ_PARTITION="$part"
  export GZ_IP=127.0.0.1
  export ROS_HOME="$run_dir/ros_home"
  mkdir -p "$ROS_HOME/log"

  # shellcheck disable=SC2086
  setsid bash -c "ros2 launch rosnav_bot slam_nav.launch.py \
    world_name:=$WORLD \
    headless:=true \
    rviz:=false \
    explore:=true \
    safety:=true \
    $extra" >"$log" 2>&1 &
  launch_pid=$!
  pgid="$launch_pid"
  LAUNCH_PGIDS+=("$pgid")

  if ! wait_boot; then
    echo "  FAIL boot (see $log)"
    echo '{"label":"'"$name"'","failed":true,"reason":"boot"}' >"$run_dir/summary.json"
    cleanup_sim
    return 1
  fi
  echo "  OK boot — collecting /map metrics for ${DURATION_S}s"

  # Existing harness: scripts/benchmark.py mode:=slam
  local wait_s=$((DURATION_S + 45))
  if timeout "$wait_s" ros2 run rosnav_bot benchmark.py --ros-args \
      -p mode:=slam \
      -p label:="$name" \
      -p duration_sec:="$DURATION_S" \
      -p out_dir:="$run_dir" \
      -p use_sim_time:=true >>"$log" 2>&1; then
    :
  else
    echo "  WARN benchmark.py exited non-zero or timed out"
  fi

  local report
  report="$(ls -1 "$run_dir"/*_slam.json 2>/dev/null | head -1 || true)"
  if [[ -n "$report" && -f "$report" ]]; then
    cp -f "$report" "$run_dir/summary.json"
    python3 - <<PY
import json
s=json.load(open("$run_dir/summary.json"))
print(f"  coverage={s.get('final_coverage_pct')}%  "
      f"converge_s={s.get('time_to_converge_sec')}  "
      f"free={s.get('final_free_cells')} occ={s.get('final_occupied_cells')}")
PY
  else
    echo '{"label":"'"$name"'","failed":true,"reason":"no_report"}' >"$run_dir/summary.json"
    echo "  FAIL no *_slam.json (see $log)"
    kill -TERM -"$pgid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    cleanup_sim
    return 1
  fi

  if ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    timeout 30 ros2 run nav2_map_server map_saver_cli -f "$run_dir/map" -t /map \
      >>"$log" 2>&1 || true
  fi

  kill -TERM -"$pgid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  cleanup_sim
  echo "  OK $name → $run_dir/summary.json"
  return 0
}

write_comparison() {
  python3 - <<'PY' "$OUT_DIR"
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
rows = []
inputs = []
for d in sorted(p for p in out.iterdir() if p.is_dir()):
    p = d / 'summary.json'
    if not p.is_file():
        continue
    s = json.loads(p.read_text())
    rows.append(s)
    if not s.get('skipped') and not s.get('failed') and 'final_coverage_pct' in s:
        inputs.append(str(p))

cmp = {'world': None, 'runs': rows}
(out / 'comparison.json').write_text(json.dumps(cmp, indent=2) + '\n')

lines = [
    '# SLAM benchmark comparison',
    '',
    '| algo | coverage_% | converge_s | free | occupied | unknown | note |',
    '|---|---:|---:|---:|---:|---:|---|',
]
for s in rows:
    note = 'skip' if s.get('skipped') else ('fail' if s.get('failed') else '')
    def fmt(v):
        if v is None: return '-'
        if isinstance(v, float): return f'{v:.2f}'
        return str(v)
    lines.append(
        f"| {s.get('label','?')} | {fmt(s.get('final_coverage_pct'))} | "
        f"{fmt(s.get('time_to_converge_sec'))} | {fmt(s.get('final_free_cells'))} | "
        f"{fmt(s.get('final_occupied_cells'))} | {fmt(s.get('final_unknown_cells'))} | {note} |"
    )
md = out / 'comparison.md'
md.write_text('\n'.join(lines) + '\n')
print(md.read_text())
if inputs:
    # Also invoke benchmark.py report mode when available.
    print('JSON reports:', *inputs)
print(f'wrote {out / "comparison.json"}')
PY
}

trap cleanup_sim EXIT

echo "SLAM benchmark  world=$WORLD  duration=${DURATION_S}s  out=$OUT_DIR"
failed=0
passed=0
for entry in "${VARIANTS[@]}"; do
  IFS='|' read -r name extra <<<"$entry"
  if run_one "$name" "$extra"; then
    passed=$((passed + 1))
  else
    failed=$((failed + 1))
  fi
done

write_comparison
echo "======== summary: $passed ok, $failed failed → $OUT_DIR/comparison.md ========"
[[ "$failed" -eq 0 ]]
