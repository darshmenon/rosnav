#!/usr/bin/env bash
# Headless smoke: each AMR platform can publish odom/scan/map and move.
# Usage:
#   ./src/rosnav_bot/scripts/smoke_amr_matrix.sh
#   WORLD=maze BOOT_WAIT_S=50 ./src/rosnav_bot/scripts/smoke_amr_matrix.sh
#   ONLY=ackermann,diff_mir100,diff_husky ./src/rosnav_bot/scripts/smoke_amr_matrix.sh
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"

# ROS setup scripts reference unset vars; enable nounset only after sourcing.
# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

WORLD="${WORLD:-obstacles}"
BOOT_WAIT_S="${BOOT_WAIT_S:-55}"
DRIVE_S="${DRIVE_S:-8}"
SETTLE_S="${SETTLE_S:-5}"
LOG_DIR="${LOG_DIR:-/tmp/rosnav_amr_smoke}"
ONLY="${ONLY:-}"
mkdir -p "$LOG_DIR"

LAUNCH_PGIDS=()
CURRENT_ROS_HOME=""

# name|drive_type|robot_model|extra_launch_args
ALL_VARIANTS=(
  "diff_custom|diff|custom|"
  "mecanum|mecanum|custom|"
  "ackermann|ackermann|custom|safety:=true"
  "diff_mir100|diff|mir100|"
  "diff_husky|diff|husky|"
)

cleanup_sim() {
  local pgid
  local pgids=("${LAUNCH_PGIDS[@]:-}")
  LAUNCH_PGIDS=()
  for pgid in "${pgids[@]}"; do
    [[ -n "${pgid:-}" ]] || continue
    kill -TERM -"$pgid" 2>/dev/null || true
  done
  sleep "$SETTLE_S"
  for pgid in "${pgids[@]}"; do
    [[ -n "${pgid:-}" ]] || continue
    kill -KILL -"$pgid" 2>/dev/null || true
  done
  sleep 2
}

topic_once() {
  local topic="$1"
  local secs="${2:-6}"
  timeout "$secs" ros2 topic echo "$topic" --once >/dev/null 2>&1
}

wait_topics() {
  local start=$SECONDS
  local t
  while (( SECONDS - start < BOOT_WAIT_S )); do
    local ok=1
    for t in /clock /odom /scan /map; do
      if ! topic_once "$t" 3; then
        ok=0
        break
      fi
    done
    if [[ "$ok" -eq 1 ]]; then
      return 0
    fi
    sleep 2
  done
  return 1
}

odom_xy() {
  # One line: "x y"
  timeout 5 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null \
    | awk '/x:/{x=$2} /y:/{y=$2; print x+0, y+0; exit}'
}

map_ok() {
  local info
  info="$(timeout 5 ros2 topic echo /map --once --field info 2>/dev/null || true)"
  echo "$info" | grep -qE 'width: [1-9]' && echo "$info" | grep -qE 'height: [1-9]'
}

dist_xy() {
  python3 -c "import math,sys
b=sys.argv[1].split(); a=sys.argv[2].split()
if len(b)<2 or len(a)<2: print(0.0); raise SystemExit
print(math.hypot(float(a[0])-float(b[0]), float(a[1])-float(b[1])))" "$1" "$2"
}

run_one() {
  local name="$1" drive="$2" model="$3" extra="$4"
  local domain=$((200 + (RANDOM % 500) + (SECONDS % 97)))
  local part="amr_${name}_${domain}_$$"
  local log="$LOG_DIR/${name}.log"
  local pass=1
  local launch_pid pgid

  echo
  echo "======== $name (drive=$drive model=$model) DOMAIN=$domain ========"

  cleanup_sim
  export ROS_DOMAIN_ID="$domain"
  export GZ_PARTITION="$part"
  export GZ_IP=127.0.0.1
  CURRENT_ROS_HOME="$LOG_DIR/ros_home_$name"
  export ROS_HOME="$CURRENT_ROS_HOME"
  mkdir -p "$ROS_HOME/log"
  : >"$log"

  # Own process group so cleanup is scoped to this smoke run only.
  # shellcheck disable=SC2086
  setsid env ROS_DOMAIN_ID="$domain" GZ_PARTITION="$part" GZ_IP=127.0.0.1 ROS_HOME="$ROS_HOME" \
    bash -c "ros2 launch rosnav_bot slam_nav.launch.py \
      world_name:=$WORLD \
      drive_type:=$drive \
      robot_model:=$model \
      headless:=true \
      rviz:=false \
      explore:=false \
      scan_gate:=false \
      $extra" >"$log" 2>&1 &
  launch_pid=$!
  pgid="$launch_pid"
  LAUNCH_PGIDS+=("$pgid")

  if ! wait_topics; then
    echo "  FAIL topics not ready within ${BOOT_WAIT_S}s"
    echo "  topics now: $(timeout 5 ros2 topic list 2>/dev/null | tr '\n' ' ' | head -c 200)"
    for t in /clock /odom /scan /map; do
      if topic_once "$t" 4; then
        echo "  OK topic $t"
      else
        echo "  FAIL topic $t"
        pass=0
      fi
    done
  else
    for t in /clock /odom /scan /map; do
      echo "  OK topic $t"
    done
  fi

  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "FAIL $name: launch exited early (see $log)"
    tail -40 "$log" || true
    cleanup_sim
    return 1
  fi

  local before after dist
  before="$(odom_xy || true)"
  [[ -n "${before:-}" ]] || before='0 0'
  timeout "$DRIVE_S" ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist \
    "{linear: {x: 0.30}, angular: {z: 0.25}}" >/dev/null 2>&1 || true
  ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  sleep 1
  after="$(odom_xy || true)"
  [[ -n "${after:-}" ]] || after='0 0'
  dist="$(dist_xy "$before" "$after")"
  if python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) > 0.05 else 1)" "$dist"; then
    echo "  OK odom moved ($before -> $after, dist=$dist)"
  else
    echo "  FAIL odom did not move ($before -> $after, dist=$dist)"
    pass=0
  fi

  if map_ok; then
    echo "  OK /map has size"
  else
    echo "  FAIL /map missing or empty"
    pass=0
  fi

  kill -TERM -"$pgid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  cleanup_sim

  if [[ "$pass" -eq 1 ]]; then
    echo "PASS $name"
    return 0
  fi
  echo "FAIL $name (log: $log)"
  grep -Ei 'error|fatal|exception|traceback' "$log" | tail -20 || true
  return 1
}

trap cleanup_sim EXIT

echo "AMR matrix smoke  world=$WORLD  boot=${BOOT_WAIT_S}s  drive=${DRIVE_S}s"
echo "logs → $LOG_DIR"

failed=0
passed=0
for entry in "${ALL_VARIANTS[@]}"; do
  IFS='|' read -r name drive model extra <<<"$entry"
  if [[ -n "$ONLY" && ",$ONLY," != *",$name,"* ]]; then
    continue
  fi
  if run_one "$name" "$drive" "$model" "$extra"; then
    passed=$((passed + 1))
  else
    failed=$((failed + 1))
  fi
done

echo
echo "======== summary: $passed passed, $failed failed ========"
[[ "$failed" -eq 0 ]]
