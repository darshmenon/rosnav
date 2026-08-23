#!/usr/bin/env bash
# kill_rosnav_sim.sh — clean up leftover rosnav_bot sim processes.
#
# `ros2 launch`'s own SIGINT/SIGTERM shutdown does NOT reliably cascade to
# `gz sim` (it's spawned via a `sh -c ruby $(which gz) sim ...` wrapper, and
# the actual gz sim process gets reparented to systemd --user if that wrapper
# exits first) — confirmed repeatedly: killing the launch PID, even via
# `timeout N ros2 launch ...`, can leave `gz sim` (server + `-g` client) and
# assorted nav2/slam_toolbox/robot_localization nodes running as orphans.
#
# This machine is also shared with other independent ROS instances/agents, so
# this script never matches on generic executable names alone (`gz sim`,
# `rviz2`, ...) — only on this package's own install path / launch files
# (rosnav_bot-specific), and — if ROS_DOMAIN_ID is set when you run this
# script — further narrows to processes actually on that domain (checked via
# /proc/$PID/environ) before touching anything.
#
# Usage:
#   bash src/rosnav_bot/scripts/kill_rosnav_sim.sh          # list + confirm
#   bash src/rosnav_bot/scripts/kill_rosnav_sim.sh -y        # skip confirmation
#   ROS_DOMAIN_ID=142 bash src/rosnav_bot/scripts/kill_rosnav_sim.sh
#       # only touch rosnav_bot processes on domain 142; matches on another
#       # domain are listed but left alone (could be another agent's session)

set -u
AUTO_YES=0
[[ "${1:-}" == "-y" || "${1:-}" == "--yes" ]] && AUTO_YES=1

PATTERN='rosnav_bot'

domain_of() {
  local pid="$1"
  tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1
}

mapfile -t CANDIDATES < <(
  for pid in $(ls /proc | grep -E '^[0-9]+$'); do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    # Exclude this script's own process tree (its path also contains
    # "rosnav_bot" and would otherwise self-match).
    [[ "$cmd" == *"kill_rosnav_sim.sh"* ]] && continue
    [[ "$cmd" == *"$PATTERN"* ]] && echo "$pid"
  done
)

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No rosnav_bot-related processes found."
  exit 0
fi

echo "Found ${#CANDIDATES[@]} rosnav_bot-related process(es):"
TO_KILL=()
for pid in "${CANDIDATES[@]}"; do
  [[ -d "/proc/$pid" ]] || continue
  dom=$(domain_of "$pid")
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-100)
  if [[ -n "${ROS_DOMAIN_ID:-}" && -n "$dom" && "$dom" != "$ROS_DOMAIN_ID" ]]; then
    printf '  [SKIP domain %-4s] pid=%-7s %s\n' "$dom" "$pid" "$cmd"
    continue
  fi
  printf '  [KILL domain %-4s] pid=%-7s %s\n' "${dom:-?}" "$pid" "$cmd"
  TO_KILL+=("$pid")
done

if [[ ${#TO_KILL[@]} -eq 0 ]]; then
  echo "Nothing to kill (all matches on a different ROS_DOMAIN_ID)."
  exit 0
fi

if [[ "$AUTO_YES" -ne 1 ]]; then
  read -r -p "Kill these ${#TO_KILL[@]} process(es)? [y/N] " reply
  [[ "$reply" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 1; }
fi

for pid in "${TO_KILL[@]}"; do
  kill -TERM "$pid" 2>/dev/null
done
sleep 5
for pid in "${TO_KILL[@]}"; do
  kill -0 "$pid" 2>/dev/null && { echo "pid=$pid still alive — SIGKILL"; kill -KILL "$pid" 2>/dev/null; }
done

echo "Done. Re-run this script or 'ps aux | grep rosnav_bot' to confirm nothing's left."
