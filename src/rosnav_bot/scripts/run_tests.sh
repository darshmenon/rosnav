#!/usr/bin/env bash
# Run every rosnav unit test (no simulator, no GPU, ~1 min) - the same command CI runs.
#
#   src/rosnav_bot/scripts/run_tests.sh              # everything
#   src/rosnav_bot/scripts/run_tests.sh -k frontier  # extra args go straight to pytest
#   src/rosnav_bot/scripts/run_tests.sh --group tools   # one slice: exploration | navigation | tools | other |
#                                                       #            fault_inject | slam_doctor | explore_eval | rosnav
#                                                       # (CI runs each slice as its own check)
#
# Needs: ROS 2 (Humble or Jazzy) sourced or installed in /opt/ros, plus python3-pytest python3-numpy
# python3-yaml python3-scipy python3-matplotlib python3-opencv python3-pil. Tests that need an optional
# package skip with a reason instead of failing.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"

# Source ROS whenever its Python packages are not importable. Do not trust ROS_DISTRO alone: the
# ros:humble container sets it without sourcing setup.bash into PYTHONPATH.
if ! python3 -c "import rclpy" 2>/dev/null; then
  for d in "${ROS_DISTRO:-}" humble jazzy; do
    if [ -n "$d" ] && [ -f "/opt/ros/$d/setup.bash" ]; then
      # shellcheck disable=SC1090
      set +u; source "/opt/ros/$d/setup.bash"; set -u
      break
    fi
  done
fi
if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "ROS 2 not found: install Humble/Jazzy or source its setup.bash first" >&2
  exit 2
fi
python3 -c "import pytest, numpy, yaml" 2>/dev/null || {
  echo "missing python deps: sudo apt install python3-pytest python3-numpy python3-yaml python3-scipy" >&2
  exit 2
}

T=src/rosnav_bot/test
COMMON=(-q -W ignore -p no:cacheprovider)

GROUP=all
if [ "${1:-}" = "--group" ]; then GROUP="${2:?--group needs a name}"; shift 2; fi

# which test files belong to which rosnav group (everything not listed lands in "other")
EXPLORATION=(test_frontier_explorer.py test_frontier_coordinator_allocation.py test_frontier_coordinator_spurious.py
             test_frontier_detection_accuracy.py test_frontier_env.py test_collab_loop_closure.py test_map_merge_known.py)
NAVIGATION=(test_mpc_core.py test_nav2_controller_params.py test_multi_robot_profile.py test_fleet_health_progress.py
            test_leader_follower_trail.py test_supervisor_core.py test_launch_undefined_names.py)
TOOLS=(test_nav2_doctor.py test_rosnav_doctor.py test_rosnav_top.py test_pulse_rates.py test_resource_bench.py
       test_nav2_watchdog_pids.py test_scan_to_odom_cloud.py test_map_timelapse.py test_static_checks.py)
VENDORED=(--ignore="$T/nav2_fault_inject" --ignore="$T/slam_doctor" --ignore="$T/nav_explore_eval")

paths() { local a=(); for f in "$@"; do a+=("$T/$f"); done; printf '%s\n' "${a[@]}"; }
ignores() { for f in "$@"; do printf '%s\n' "--ignore=$T/$f"; done; }

fail=0
run() {   # run <title> <pytest args...>; pytest exit 5 (nothing collected, e.g. -k filter) is not a failure
  local title="$1"; shift
  echo "== $title =="
  python3 -m pytest "${COMMON[@]}" "$@" "${EXTRA[@]}"; local rc=$?
  { [ "$rc" -eq 0 ] || [ "$rc" -eq 5 ]; } || fail=1
}
EXTRA=("$@")

case "$GROUP" in
  all|rosnav)
    run "rosnav tests ($ROS_DISTRO)" "$T" "${VENDORED[@]}" ;;
  exploration) run "rosnav: exploration" $(paths "${EXPLORATION[@]}") ;;
  navigation)  run "rosnav: navigation + fleet" $(paths "${NAVIGATION[@]}") ;;
  tools)       run "rosnav: tools + doctors" $(paths "${TOOLS[@]}") ;;
  other)       run "rosnav: everything else" "$T" "${VENDORED[@]}" \
                 $(ignores "${EXPLORATION[@]}" "${NAVIGATION[@]}" "${TOOLS[@]}") ;;
  fault_inject) run "nav2_fault_inject" "$T/nav2_fault_inject" ;;
  slam_doctor)  run "slam_doctor" "$T/slam_doctor" ;;
  explore_eval) run "nav_explore_eval" "$T/nav_explore_eval" ;;
  *) echo "unknown group: $GROUP" >&2; exit 2 ;;
esac

if [ "$GROUP" = "all" ]; then
  # vendored standalone tools: each has its own conftest, so run them as separate sessions
  run "nav2_fault_inject" "$T/nav2_fault_inject"
  run "slam_doctor" "$T/slam_doctor"
  run "nav_explore_eval" "$T/nav_explore_eval"
fi

[ "$fail" -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit "$fail"
