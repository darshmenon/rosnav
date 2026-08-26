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
FORCE_ALL=0
[[ "${1:-}" == "--force-all-domains" || "${2:-}" == "--force-all-domains" ]] && FORCE_ALL=1

# Refuse to run domain-blind by default (2026-08-26 finding): a caller that
# forgets to export ROS_DOMAIN_ID before invoking this script — e.g. running
# it as a standalone tool call instead of inside the shell that already has
# it set — silently loses the whole domain cross-check below and kills
# rosnav_bot processes on ANY domain, including another concurrent session's.
# Confirmed this actually happened live: killed another Claude session's
# empty_room.world run on domain 161 while this shell had no ROS_DOMAIN_ID
# set at all. Opt out explicitly with --force-all-domains if you really do
# want to sweep every domain (e.g. final cleanup before ending a session).
if [[ -z "${ROS_DOMAIN_ID:-}" && "$FORCE_ALL" -ne 1 ]]; then
  echo "ROS_DOMAIN_ID is not set in this shell — refusing to run domain-blind."
  echo "Export the ROS_DOMAIN_ID this session is using first, e.g.:"
  echo "  ROS_DOMAIN_ID=151 bash $0"
  echo "Or pass --force-all-domains if you deliberately want to sweep every domain."
  exit 1
fi

# 2026-08-26 finding: a baked-world `gz sim` process's cmdline is
# `gz sim -r -s -v1 /tmp/rosnav_baked_world_XXXX/baked_<world>.world` — it
# does NOT contain the literal substring "rosnav_bot" anywhere (the tmp dir
# is "rosnav_baked_world", not "rosnav_bot"), so a plain 'rosnav_bot' pattern
# never matched it. Confirmed live: an unattended batch sweep left every
# previous world's gz sim running while starting the next, stacking up to 9
# concurrent instances on one ROS_DOMAIN_ID and corrupting results (frozen
# odom/scan timestamps, "stamp_went_backwards", launch connect failures).
# Broadened to also match the baked-robot/baked-world/cartographer runtime
# tmp dirs created by launch/_common.py.
PATTERN='rosnav_bot|rosnav_baked_world|rosnav_baked_robot|rosnav_cartographer'

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
    [[ "$cmd" =~ $PATTERN ]] && echo "$pid"
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
