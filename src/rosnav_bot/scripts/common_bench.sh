# common_bench.sh — sourced helpers for the benchmark_*.sh / fault / smoke scripts.
# Isolated domain per run; kill ONLY processes carrying that ROS_DOMAIN_ID
# (shared machine: never pkill by ROS/Gazebo name).
#   bench_env_init   source ROS + install overlay
#   bench_domain     pick a random isolated ROS_DOMAIN_ID/GZ_PARTITION
#   domain_pids      list PIDs whose environ has our ROS_DOMAIN_ID
#   domain_kill_match PATTERN   SIGKILL domain procs whose cmdline matches PATTERN
#   domain_kill_all  tear everything in our domain down + clean Fast DDS shm
#   wait_topics T..  wait until each topic publishes once (BOOT_WAIT_S)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
bench_env_init() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
  # shellcheck disable=SC1091
  source "$ROOT/install/setup.bash"
  set -u
}
bench_domain() {
  local d=$((100 + RANDOM % 120))
  export ROS_DOMAIN_ID="$d" GZ_PARTITION="bench_$d" GZ_IP=127.0.0.1
}
domain_pids() {
  local p
  for p in $(ps -eo pid=); do
    [[ "$p" == "$$" ]] && continue
    [[ -r /proc/$p/environ ]] || continue
    tr '\0' '\n' </proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" && echo "$p"
  done
}
domain_kill_match() {
  local pat="$1" p n=0
  for p in $(domain_pids); do
    tr '\0' ' ' </proc/$p/cmdline 2>/dev/null | grep -qE -- "$pat" || continue
    kill -9 "$p" 2>/dev/null && n=$((n + 1))
  done
  echo "$n"
}
domain_kill_all() {
  local round p
  for round in 1 2; do
    for p in $(domain_pids); do
      case "$(tr '\0' ' ' </proc/$p/cmdline 2>/dev/null)" in *claude*|*common_bench*) continue ;; esac
      kill -9 "$p" 2>/dev/null
    done
    sleep 2
  done
  "$ROOT/src/rosnav_bot/scripts/clean_fastdds_shm.sh" --delete >/dev/null 2>&1 || true
}
wait_topics() {
  local deadline=$((SECONDS + ${BOOT_WAIT_S:-60})) t ok
  while ((SECONDS < deadline)); do
    ok=1
    for t in "$@"; do timeout 3 ros2 topic echo "$t" --once >/dev/null 2>&1 || { ok=0; break; }; done
    ((ok)) && return 0
    sleep 2
  done
  return 1
}
