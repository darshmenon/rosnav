#!/usr/bin/env bash
# Generalized batch SLAM+explorer sweep — parameterized by EXPLORER and
# SLAM_ALGO, appends to one combined CSV. Not committed to the repo
# (scratch/session tooling) - lives in sweep_logs/.
#
# Usage: EXPLORER=builtin SLAM_ALGO=2d bash batch_map_all_worlds_v2.sh
cd /home/asimov/rosnav
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-152}"
export GZ_PARTITION="${GZ_PARTITION:-rosnav-batch-152}"

EXPLORER="${EXPLORER:-explore_lite}"
SLAM_ALGO="${SLAM_ALGO:-2d}"
WORLDS="${WORLDS:-bench_room_small bench_room_cluttered corridor coverage_100 simple_rooms house office warehouse aws_warehouse tugbot_warehouse warehouse_depot lake_house multi_terrain}"
PER_WORLD_SEC="${PER_WORLD_SEC:-200}"
RESULTS=/home/asimov/rosnav/sweep_logs/batch_results_combined.csv

if [ ! -f "$RESULTS" ]; then
  echo "explorer,slam_algo,world,start_epoch,launch_ok,final_free_cells,final_total_cells,final_pct,offgrid_errors,elapsed_sec,stuck,stuck_at_sec,completed_early" > "$RESULTS"
fi

kill_sim() {
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" bash src/rosnav_bot/scripts/kill_rosnav_sim.sh -y >/dev/null 2>&1
  sleep 2
  # Belt-and-suspenders (see batch_map_all_worlds.sh v1 for the 2026-08-26
  # incident this guards against): force-kill any baked-world gz sim still
  # alive on our own domain after the cleanup script ran.
  for pid in $(pgrep -f 'rosnav_baked_world' 2>/dev/null); do
    dom=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
    if [ "$dom" = "$ROS_DOMAIN_ID" ]; then
      echo "WARNING: kill_sim left pid=$pid alive on domain $ROS_DOMAIN_ID — force killing" >&2
      kill -KILL "$pid" 2>/dev/null
    fi
  done
  sleep 1
  # 2026-08-26 finding: orphaned FastRTPS /dev/shm segments (left behind by
  # non-graceful teardown across many launch/kill cycles) accumulate fast —
  # observed 195 new orphans after just 3 world launches — and once enough
  # build up, new nodes fail to bind DDS SHM transport ports
  # ("Failed init_port fastrtps_portNNNNN: open_and_lock_file failed"),
  # which manifests as bt_navigator/lifecycle_manager hanging forever
  # (never reaching "Managed nodes are active"). Clean between every world,
  # not just when things start failing. Verifies zero live file-descriptor
  # references before removing anything (safe on a shared machine — never
  # touches a segment any live process, ours or another session's, holds
  # open) and caps the scan at 15s so a slow /proc walk can't stall the
  # sweep indefinitely.
  timeout 15 bash -c '
    inuse=$(for pid in /proc/[0-9]*; do ls -l "$pid/fd" 2>/dev/null; done | grep -oP "(?<=/dev/shm/)fastrtps_\S+" | sort -u)
    cd /dev/shm 2>/dev/null && comm -23 <(ls 2>/dev/null | grep fastrtps | sort) <(echo "$inuse") | xargs -r rm -f --
  ' 2>/dev/null
}

for W in $WORLDS; do
  echo "=== [$EXPLORER / $SLAM_ALGO] $W ==="
  kill_sim
  LOG="/tmp/batch_${EXPLORER}_${SLAM_ALGO}_${W}.log"
  START=$(date +%s)
  ros2 launch rosnav_bot slam_nav.launch.py world_name:="$W" explore:=true explorer:="$EXPLORER" \
    slam_algo:="$SLAM_ALGO" \
    headless:=true rviz:=false map_prefix:="$(pwd)/src/rosnav_bot/maps/map_${W}_${EXPLORER}_${SLAM_ALGO}" \
    > "$LOG" 2>&1 &
  LAUNCH_PID=$!

  CONNECTED=0
  for i in $(seq 1 40); do
    # See batch_map_all_worlds.sh v1 (2026-08-26) — explore_lite's own
    # connect line is unreliable due to non-TTY stdout buffering; nav2's
    # lifecycle_manager "Managed nodes are active" is the robust signal.
    if grep -qE "Managed nodes are active|Connected to move_base nav2 server|Starting Exploration!" "$LOG" 2>/dev/null; then
      CONNECTED=1
      break
    fi
    sleep 1
  done

  if [ "$CONNECTED" -eq 0 ]; then
    echo "$EXPLORER,$SLAM_ALGO,$W,$START,0,,,,,,,," >> "$RESULTS"
    kill -TERM "$LAUNCH_PID" 2>/dev/null
    kill_sim
    continue
  fi

  ELAPSED=0
  LAST_POSE=""
  STUCK=0
  STUCK_AT=""
  REPEAT=0
  COMPLETED_EARLY=0
  while [ "$ELAPSED" -lt "$PER_WORLD_SEC" ]; do
    sleep 10
    ELAPSED=$((ELAPSED+10))
    if grep -qiE "All frontiers traversed|No frontiers found, stopping|exploration complete" "$LOG" 2>/dev/null; then
      COMPLETED_EARLY=1
      break
    fi
    POSE=$(grep -oE "odom_diag:.*pose=\([^)]*\)" "$LOG" 2>/dev/null | tail -1)
    if [ -n "$POSE" ] && [ "$POSE" == "$LAST_POSE" ]; then
      REPEAT=$((REPEAT+1))
      if [ "$REPEAT" -ge 6 ] && [ "$STUCK" -eq 0 ]; then
        STUCK=1
        STUCK_AT=$ELAPSED
      fi
    else
      REPEAT=0
    fi
    LAST_POSE="$POSE"
  done

  END=$(date +%s)
  # grep -c always prints a count (even 0) and only fails via exit-status on
  # no match, so the old `|| echo 0` fallback double-printed "0\n0" whenever
  # there were zero matches, splitting the CSV row across two physical
  # lines. grep -c's own output is sufficient; default only if grep itself
  # errored (missing file, etc).
  OFFGRID=$(grep -c "Pose Goes Off Grid" "$LOG" 2>/dev/null)
  OFFGRID="${OFFGRID:-0}"

  timeout 20 ros2 run nav2_map_server map_saver_cli -f "$(pwd)/src/rosnav_bot/maps/map_${W}_${EXPLORER}_${SLAM_ALGO}" \
    --ros-args -p save_map_timeout:=5000.0 >> "$LOG" 2>&1

  STATS=$(timeout 15 python3 -c "
try:
    from PIL import Image
    im = Image.open('$(pwd)/src/rosnav_bot/maps/map_${W}_${EXPLORER}_${SLAM_ALGO}.pgm')
    px = list(im.getdata())
    total = len(px)
    free = sum(1 for v in px if v > 250)
    print(f'{free},{total},{100*free/total:.1f}' if total else '0,0,0')
except Exception:
    print('0,0,0')
" 2>/dev/null)
  IFS=',' read -r FREE TOTAL PCT <<< "${STATS:-0,0,0}"

  echo "$EXPLORER,$SLAM_ALGO,$W,$START,1,$FREE,$TOTAL,$PCT,$OFFGRID,$((END-START)),$STUCK,$STUCK_AT,$COMPLETED_EARLY" >> "$RESULTS"

  kill -TERM "$LAUNCH_PID" 2>/dev/null
  sleep 3
  kill_sim
done

echo "=== [$EXPLORER / $SLAM_ALGO] PASS COMPLETE ==="
