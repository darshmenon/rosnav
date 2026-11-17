#!/bin/bash
# demo_social_nav.sh — one command for the live social-navigation demo (RViz first, then Gazebo + Nav2).
#
#   scripts/demo_social_nav.sh                 # hospital, 10 pedestrians, learned mask, then plain Nav2 for contrast
#   scripts/demo_social_nav.sh cafe            # another mapped world
#   CONFIGS="learned" PEDS=14 scripts/demo_social_nav.sh
#
# Shows in RViz (rviz/social_nav.rviz): people, predicted paths + uncertainty, the dynamic keepout mask.
# Watch `ros2 topic echo /social_nav/status` for the watchdog (OK / DEGRADED / FALLBACK) and live conformal radii.
# Runs benchmark_social_nav.sh with RVIZ=1 (RViz opens and loads first, as the demo needs), prints the metrics table.
# Needs the machine quiet (load guard inside); nothing is installed or committed.
set -u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORLD=${1:-hospital}
export RVIZ=1 PEDS=${PEDS:-10} DURATION=${DURATION:-90} SEED=${SEED:-0} OUT=${OUT:-/tmp/social_demo_$WORLD}
CONFIGS=${CONFIGS:-"learned plain"}
echo "[demo] world=$WORLD peds=$PEDS duration=${DURATION}s configs: $CONFIGS  (results -> $OUT)"
echo "[demo] RViz opens first; then Gazebo, pedestrians, social_nav_node (watchdog + online conformal) and the metrics."
"$HERE/benchmark_social_nav.sh" "$WORLD" $CONFIGS
echo "[demo] done. JSON per config in $OUT; repo visuals: assets/social_nav/"
