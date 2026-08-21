#!/usr/bin/env bash
# Symlink C-SLAM / explorer clones from ~/rosnav_sources into this workspace.
set -euo pipefail

SRC="${ROSNAV_SOURCES:-$HOME/rosnav_sources}"
WS="$(cd "$(dirname "$0")/../../.." && pwd)/src"

if [[ ! -d "$SRC" ]]; then
  echo "Missing $SRC — clone the upstream repos there first." >&2
  exit 1
fi

link() {
  local from="$1" to="$2"
  if [[ ! -e "$from" ]]; then
    echo "skip (missing): $from"
    return
  fi
  mkdir -p "$(dirname "$to")"
  ln -sfn "$from" "$to"
  echo "linked $to -> $from"
}

link "$SRC/m-explore-ros2/explore"            "$WS/explore_lite"
link "$SRC/m-explore-ros2/explore_lite_msgs"  "$WS/explore_lite_msgs"
link "$SRC/m-explore-ros2/map_merge"          "$WS/multirobot_map_merge"
link "$SRC/frontier_exploration_ros2"         "$WS/frontier_exploration_ros2"
link "$SRC/rrt-explore"                       "$WS/rrt_explore"

# Optional Swarm-SLAM (needs GTSAM / teaser++). Use: $0 --cslam
if [[ "${1:-}" == "--cslam" ]]; then
  link "$SRC/cslam"                                        "$WS/cslam"
  link "$SRC/cslam_experiments"                            "$WS/cslam_experiments"
  link "$SRC/cslam_interfaces/cslam_common_interfaces"     "$WS/cslam_common_interfaces"
fi

# RViz plugin is optional; ignore it so a default colcon build stays on the explorer node.
plugin_ignore="$SRC/frontier_exploration_ros2/plugin/frontier_exploration_ros2_rviz/COLCON_IGNORE"
if [[ -d "$(dirname "$plugin_ignore")" ]]; then
  touch "$plugin_ignore" 2>/dev/null || true
fi

echo
echo "Build explorers:"
echo "  cd $WS/.. && colcon build --symlink-install --packages-select explore_lite_msgs explore_lite frontier_exploration_ros2 rrt_explore"
echo "Then: explorer:=explore_lite | explorer:=frontier | explorer:=rrt"
if [[ "${1:-}" == "--cslam" ]]; then
  echo "C-SLAM: colcon build --symlink-install --packages-up-to cslam"
  echo "Then: slam_algo:=cslam lidar_type:=3d"
fi
