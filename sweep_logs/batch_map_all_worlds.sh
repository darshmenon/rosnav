#!/usr/bin/env bash
# Batch SLAM+explore_lite sweep across all bounded worlds, post-IMU-fix.
# Not committed to the repo (scratch/session tooling) - lives in sweep_logs/.
cd /home/asimov/rosnav
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID=152
export GZ_PARTITION=rosnav-batch-152

WORLDS="bench_room_small bench_room_cluttered corridor coverage_100 simple_rooms house office warehouse aws_warehouse tugbot_warehouse warehouse_depot lake_house multi_terrain"
PER_WORLD_SEC=200
RESULTS=/home/asimov/rosnav/sweep_logs/batch_results.csv
echo "world,start_epoch,launch_ok,final_free_cells,final_total_cells,final_pct,offgrid_errors,elapsed_sec,stuck,stuck_at_sec,completed_early" > "$RESULTS"

kill_sim() {
  ROS_DOMAIN_ID=152 bash src/rosnav_bot/scripts/kill_rosnav_sim.sh -y >/dev/null 2>&1
  sleep 2
  # Verify: kill_rosnav_sim.sh's pattern match previously missed baked-world
  # gz sim processes entirely (2026-08-26 bug, now fixed upstream), which
  # silently stacked sims across worlds and corrupted results. Belt-and-
  # suspenders check here so a future pattern gap fails loud, not silent.
  for pid in $(pgrep -f 'rosnav_baked_world' 2>/dev/null); do
    dom=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
    if [ "$dom" = "152" ]; then
      echo "WARNING: kill_sim left pid=$pid alive on domain 152 — force killing" >&2
      kill -KILL "$pid" 2>/dev/null
    fi
  done
  sleep 1
  # See batch_map_all_worlds_v2.sh (2026-08-26) — orphaned FastRTPS /dev/shm
  # segments accumulate fast across launch/kill cycles and eventually block
  # new nodes from binding DDS SHM transport ports, manifesting as
  # bt_navigator/lifecycle_manager hanging forever. Clean between every
  # world. Verified safe: only removes segments with zero live
  # file-descriptor references anywhere on the machine.
  timeout 15 bash -c '
    inuse=$(for pid in /proc/[0-9]*; do ls -l "$pid/fd" 2>/dev/null; done | grep -oP "(?<=/dev/shm/)fastrtps_\S+" | sort -u)
    cd /dev/shm 2>/dev/null && comm -23 <(ls 2>/dev/null | grep fastrtps | sort) <(echo "$inuse") | xargs -r rm -f --
  ' 2>/dev/null
}

for W in $WORLDS; do
  echo "=== $W ==="
  kill_sim
  LOG="/tmp/batch_${W}.log"
  START=$(date +%s)
  ros2 launch rosnav_bot slam_nav.launch.py world_name:="$W" explore:=true explorer:=explore_lite \
    headless:=true rviz:=false map_prefix:="$(pwd)/src/rosnav_bot/maps/map_${W}" \
    > "$LOG" 2>&1 &
  LAUNCH_PID=$!

  # Wait for connection (up to 40s)
  CONNECTED=0
  for i in $(seq 1 40); do
    # 2026-08-26 finding: explore_lite's own "Connected to move_base nav2
    # server" line is unreliable here — its C++ stdout is fully-buffered
    # (not a TTY once piped through ros2 launch to a redirected file), so
    # the line can sit unflushed indefinitely even though the node is
    # actually running fine. nav2's own lifecycle_manager "Managed nodes are
    # active" line is a much more robust readiness signal (confirmed
    # printed reliably at ~12s in a run where explore's own line never
    # appeared at all even by 38s). Keep the old string as a fallback.
    if grep -qE "Managed nodes are active|Connected to move_base nav2 server" "$LOG" 2>/dev/null; then
      CONNECTED=1
      break
    fi
    sleep 1
  done

  if [ "$CONNECTED" -eq 0 ]; then
    echo "$W,$START,0,,,,,,,," >> "$RESULTS"
    kill -TERM "$LAUNCH_PID" 2>/dev/null
    kill_sim
    continue
  fi

  # Let it explore for PER_WORLD_SEC. Keeps running even if stuck (never
  # aborts early on stall) - only stops early on a genuine completion
  # signal, or after the full time budget either way.
  ELAPSED=0
  LAST_POSE=""
  STUCK=0
  STUCK_AT=""
  REPEAT=0
  COMPLETED_EARLY=0
  while [ "$ELAPSED" -lt "$PER_WORLD_SEC" ]; do
    sleep 10
    ELAPSED=$((ELAPSED+10))
    if grep -qiE "All frontiers traversed|No frontiers found, stopping" "$LOG" 2>/dev/null; then
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
  # grep -c already prints a count (even 0) and only fails via exit-status
  # on no match, so `|| echo 0` double-printed "0\n0" whenever there were
  # zero matches, splitting every such CSV row across two physical lines
  # (see gen_slam_explorer_compare.py's read_csv, which then silently
  # dropped every one of this script's rows entirely — no `explorer` column
  # either, worth adding here too so future runs don't need the same
  # after-the-fact CSV patch batch_results_combined.csv got on 2026-08-26).
  OFFGRID=$(grep -c "Pose Goes Off Grid" "$LOG" 2>/dev/null)
  OFFGRID="${OFFGRID:-0}"

  # Save map
  timeout 20 ros2 run nav2_map_server map_saver_cli -f "$(pwd)/src/rosnav_bot/maps/map_${W}" \
    --ros-args -p save_map_timeout:=5000.0 >> "$LOG" 2>&1

  # Coverage from the saved yaml/pgm via python (free/total)
  STATS=$(timeout 15 python3 -c "
import sys
try:
    from PIL import Image
    im = Image.open('$(pwd)/src/rosnav_bot/maps/map_${W}.pgm')
    px = list(im.getdata())
    total = len(px)
    free = sum(1 for v in px if v > 250)
    print(f'{free},{total},{100*free/total:.1f}' if total else '0,0,0')
except Exception as e:
    print('0,0,0')
" 2>/dev/null)
  IFS=',' read -r FREE TOTAL PCT <<< "${STATS:-0,0,0}"

  echo "$W,$START,1,$FREE,$TOTAL,$PCT,$OFFGRID,$((END-START)),$STUCK,$STUCK_AT,$COMPLETED_EARLY" >> "$RESULTS"

  kill -TERM "$LAUNCH_PID" 2>/dev/null
  sleep 3
  kill_sim
done

echo "=== BATCH COMPLETE ==="
cat "$RESULTS"
