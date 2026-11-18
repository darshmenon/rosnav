#!/usr/bin/env bash
# benchmark_explore_trials.sh — repeated exploration trials with mean ± std / 95% CI.
# Single runs are noise; this varies spawn yaw per seed (deterministic from the
# seed) and reports final coverage + time-to-converge across N trials per arm.
#
# Usage:
#   WORLD=maze TRIALS=5 ARMS="builtin explore_lite" ./scripts/benchmark_explore_trials.sh
#   ARMS="builtin builtin,info_gain_mode:=fov" ...   (arm = explorer[,extra launch arg])
# Env: WORLD (maze) TRIALS (5) DURATION_S (180) BOOT_WAIT_S (120) OUT_DIR  (rerun with the same OUT_DIR to resume)
# Does not commit or push. Uses a random isolated ROS_DOMAIN_ID per trial and
# kills only its own process group (shared machine — no pkill by name).
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/bench_load_guard.sh"; bench_load_guard

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

WORLD="${WORLD:-maze}"
TRIALS="${TRIALS:-5}"
DURATION_S="${DURATION_S:-180}"
BOOT_WAIT_S="${BOOT_WAIT_S:-120}"
ARMS="${ARMS:-builtin explore_lite}"
EXPLORABLE_AREA_M2="${EXPLORABLE_AREA_M2:-0}"   # >0: report coverage_true_pct (free area / this); maze = 323.3 (nav_explore_eval truth-from-world)
MIN_FREE_CELLS="${MIN_FREE_CELLS:-4000}"         # a trial that ends below this never really moved (sim/bridge failure) -> retried
MAX_RETRIES="${MAX_RETRIES:-2}"
OUT_DIR="${OUT_DIR:-/tmp/rosnav_explore_trials/$(date +%Y%m%d_%H%M%S)_$WORLD}"
mkdir -p "$OUT_DIR"

PGID=""
cleanup() {
  [[ -n "$PGID" ]] || return 0
  kill -TERM -"$PGID" 2>/dev/null || true
  sleep 2
  kill -KILL -"$PGID" 2>/dev/null || true
  PGID=""
  "$ROOT/src/rosnav_bot/scripts/clean_fastdds_shm.sh" --delete >/dev/null 2>&1 || true
}
trap cleanup EXIT

wait_boot() {
  local deadline=$((SECONDS + BOOT_WAIT_S))
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 topic echo /clock --once >/dev/null 2>&1 \
       && timeout 3 ros2 topic echo /scan --once >/dev/null 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

# Pause BETWEEN trials while the shared machine is busy (a mid-trial freeze would corrupt the wall-clock duration).
wait_idle() {
  local max="${PAUSE_LOAD:-8}" waited=0
  while [[ "$(cut -d. -f1 /proc/loadavg)" -ge "$max" ]]; do
    (( waited % 300 == 0 )) && echo "[pause] load $(cut -d' ' -f1 /proc/loadavg) >= $max, waiting before next trial"
    sleep 30; waited=$((waited + 30))
  done
}

run_trial() {
  local arm="$1" seed="$2"
  local explorer="${arm%%,*}" extra=""
  [[ "$arm" == *,* ]] && extra="${arm#*,}"
  local tag; tag="$(echo "$arm" | tr -c 'A-Za-z0-9\n' _ | cut -c1-60)_s$seed"
  local dir="$OUT_DIR/$tag"; mkdir -p "$dir"
  # resume: keep trials that already produced a good report
  if [[ -f "$dir/summary.json" ]] && ! grep -q '"failed"' "$dir/summary.json"; then
    echo "[trial] $tag already done — skipping"; return 0
  fi
  wait_idle
  # deterministic spawn yaw from the seed, 0..2pi
  local yaw; yaw=$(python3 -c "import math;print(round((($seed*2.399963)%(2*math.pi)),3))")
  local domain=$((100 + RANDOM % 120))
  export ROS_DOMAIN_ID="$domain" GZ_PARTITION="exp_trial_$domain" GZ_IP=127.0.0.1
  export ROS_HOME="$dir/ros_home"; mkdir -p "$ROS_HOME/log"

  echo "[trial] arm=$arm seed=$seed yaw=$yaw domain=$domain"
  local -a EXTRA=()
  # shellcheck disable=SC2206
  [[ -n "$extra" ]] && EXTRA=($extra)   # split on spaces; JSON values must contain none
  setsid ros2 launch rosnav_bot slam_nav.launch.py world_name:="$WORLD" \
      headless:=true rviz:=false explore:=true explorer:="$explorer" spawn_yaw:="$yaw" \
      "${EXTRA[@]}" >"$dir/launch.log" 2>&1 &
  PGID=$!
  if ! wait_boot; then
    echo "  FAIL boot"; echo '{"failed":true,"reason":"boot"}' >"$dir/summary.json"; cleanup; return 0
  fi
  timeout $((DURATION_S + 45)) ros2 run rosnav_bot benchmark.py --ros-args \
      -p mode:=slam -p label:="$tag" -p duration_sec:="$DURATION_S" \
      -p out_dir:="$dir" -p explorable_area_m2:="$EXPLORABLE_AREA_M2" -p use_sim_time:=true >>"$dir/launch.log" 2>&1 || echo "  WARN bench rc!=0"
  local rep; rep="$(ls -1 "$dir"/*_slam.json 2>/dev/null | head -1 || true)"
  if [[ -n "$rep" ]]; then cp -f "$rep" "$dir/summary.json"
  else echo '{"failed":true,"reason":"no_report"}' >"$dir/summary.json"; fi
  cleanup
}

echo "explore trials world=$WORLD trials=$TRIALS dur=${DURATION_S}s arms=[$ARMS] out=$OUT_DIR"
for arm in $ARMS; do
  for seed in $(seq 1 "$TRIALS"); do
    attempt=0
    while :; do
      run_trial "$arm" "$seed"
      tag="$(echo "$arm" | tr -c 'A-Za-z0-9\n' _ | cut -c1-60)_s$seed"
      free="$(python3 -I -c "import json,sys;d=json.load(open(sys.argv[1]));print(-1 if d.get('failed') else d.get('final_free_cells',0))" "$OUT_DIR/$tag/summary.json" 2>/dev/null || echo 0)"
      if [[ "$free" -ge "$MIN_FREE_CELLS" ]] || (( attempt >= MAX_RETRIES )); then break; fi
      attempt=$((attempt + 1))
      echo "[retry] $tag ended with free_cells=$free (< $MIN_FREE_CELLS: robot never moved or boot failed) — attempt $attempt/$MAX_RETRIES"
      mv "$OUT_DIR/$tag" "$OUT_DIR/${tag}_invalid$attempt"
    done
  done
done

python3 -I - "$OUT_DIR" <<'PY'
import json, math, statistics as st, sys
from pathlib import Path
out = Path(sys.argv[1]); arms = {}
for d in sorted(p for p in out.iterdir() if p.is_dir()):
    f = d / 'summary.json'
    if not f.is_file(): continue
    arm = d.name.rsplit('_s', 1)[0]
    if '_invalid' in d.name: continue
    arms.setdefault(arm, []).append(json.loads(f.read_text()))
def ci(xs):  # 95% CI half-width, Student t for small n
    n = len(xs)
    if n < 2: return float('nan')
    t = {2:12.71,3:4.30,4:3.18,5:2.78,6:2.57,7:2.45,8:2.36,9:2.31,10:2.26}.get(n, 1.96)
    return t * st.stdev(xs) / math.sqrt(n)
def col(runs, k): return [r[k] for r in runs if not r.get('failed') and r.get(k) is not None]
lines = ['| arm | ok/total | map-box coverage % (mean ± 95% CI) | true coverage % (mean ± 95% CI) | std | converge s (mean ± CI) |', '|---|---|---|---|---|---|']
for arm, runs in arms.items():
    c, t, tc = col(runs, 'final_coverage_pct'), col(runs, 'time_to_converge_sec'), col(runs, 'coverage_true_pct')
    f = lambda xs: f'{st.mean(xs):.1f} ± {ci(xs):.1f}' if len(xs) > 1 else (f'{xs[0]:.1f}' if xs else '-')
    sd = f'{st.stdev(c):.1f}' if len(c) > 1 else '-'
    lines.append(f'| {arm} | {len(c)}/{len(runs)} | {f(c)} | {f(tc)} | {sd} | {f(t)} |')
(out / 'trials.md').write_text('\n'.join(lines) + '\n')
(out / 'trials.json').write_text(json.dumps(arms, indent=2) + '\n')
print('\n'.join(lines)); print(f'wrote {out}/trials.md')
PY
