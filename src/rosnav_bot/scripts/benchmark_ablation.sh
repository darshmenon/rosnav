#!/usr/bin/env bash
# benchmark_ablation.sh — turn off one frontier_explorer feature at a time and
# measure the coverage cost, via benchmark_explore_trials.sh (N seeded trials/arm).
# Shows which pieces earn their complexity. Builtin explorer only.
#
# Usage: WORLD=maze TRIALS=4 ./scripts/benchmark_ablation.sh
# Extra arms: ABLATIONS_EXTRA='builtin,explorer_params:={"revisit_radius":0.0}'
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARMS=(
  'builtin'
  'builtin,explorer_params:={"hysteresis_gain":0.0}'
  'builtin,explorer_params:={"suspicious_frontier_penalty":0.0,"suspicious_frontier_hard_ratio":1000.0}'
  'builtin,explorer_params:={"frontier_clearance_radius":0.0}'
  'builtin,explorer_params:={"failed_frontier_cooldown":0.0}'
  'builtin,info_gain_mode:=fov'
)
# shellcheck disable=SC2086
ARMS="${ARMS[*]} ${ABLATIONS_EXTRA:-}" exec "$HERE/benchmark_explore_trials.sh"
