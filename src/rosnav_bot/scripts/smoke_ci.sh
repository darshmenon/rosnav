#!/usr/bin/env bash
# smoke_ci.sh — short headless regression gate: single robot explores for a minute or two
# and the run FAILS (exit 1) if the stack didn't boot or map coverage stayed below MIN_COVERAGE.
# Catches TF freezes / dead Nav2 / dead explorer without a human watching RViz.
#
# Usage: WORLD=maze DURATION_S=90 MIN_COVERAGE=8 ./scripts/smoke_ci.sh   (exit 0 pass, 1 fail)
# MIN_COVERAGE is % of the *map grid* benchmark.py reports (grows with SLAM), so keep it
# conservative and tune from a known-good run's summary.json.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/common_bench.sh"
bench_env_init
WORLD="${WORLD:-maze}"; DURATION_S="${DURATION_S:-90}"; MIN_COVERAGE="${MIN_COVERAGE:-8}"
BOOT_WAIT_S="${BOOT_WAIT_S:-60}"
OUT_DIR="${OUT_DIR:-/tmp/rosnav_smoke/$(date +%Y%m%d_%H%M%S)}"; mkdir -p "$OUT_DIR"
trap 'domain_kill_all' EXIT
bench_domain; export ROS_HOME="$OUT_DIR/ros_home"; mkdir -p "$ROS_HOME/log"

setsid ros2 launch rosnav_bot slam_nav.launch.py world_name:="$WORLD" headless:=true rviz:=false \
    explore:=true >"$OUT_DIR/launch.log" 2>&1 &
wait_topics /clock /scan /odom || { echo "SMOKE FAIL: stack did not boot (see $OUT_DIR/launch.log)"; exit 1; }
timeout $((DURATION_S + 60)) ros2 run rosnav_bot benchmark.py --ros-args -p mode:=slam -p label:=smoke \
    -p duration_sec:="$DURATION_S" -p out_dir:="$OUT_DIR" -p use_sim_time:=true >>"$OUT_DIR/launch.log" 2>&1 || true
python3 -I - "$OUT_DIR/smoke_slam.json" "$MIN_COVERAGE" <<'PY'
import json, sys
try:
    s = json.load(open(sys.argv[1]))
except Exception as e:
    print(f'SMOKE FAIL: no report ({e})'); sys.exit(1)
cov = s.get('final_coverage_pct') or 0.0
ok = cov >= float(sys.argv[2])
print(f"SMOKE {'PASS' if ok else 'FAIL'}: coverage={cov}% (min {sys.argv[2]}%)")
sys.exit(0 if ok else 1)
PY
