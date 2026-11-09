#!/usr/bin/env bash
# Run every rosnav unit test (no simulator, no GPU, ~1 min) - the same command CI runs.
#
#   src/rosnav_bot/scripts/run_tests.sh              # everything
#   src/rosnav_bot/scripts/run_tests.sh -k frontier  # extra args go straight to pytest
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
fail=0

echo "== rosnav tests ($ROS_DISTRO) =="
python3 -m pytest "$T" "${COMMON[@]}" \
  --ignore="$T/nav2_fault_inject" --ignore="$T/slam_doctor" --ignore="$T/nav_explore_eval" "$@" || fail=1

# vendored standalone tools: each has its own conftest, so run them as separate sessions
for tool in nav2_fault_inject slam_doctor nav_explore_eval; do
  echo "== $tool =="
  python3 -m pytest "$T/$tool" "${COMMON[@]}" "$@" || { rc=$?; [ "$rc" -eq 5 ] || fail=1; }
done

[ "$fail" -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit "$fail"
