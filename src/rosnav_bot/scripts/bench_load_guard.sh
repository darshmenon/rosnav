# bench_load_guard.sh — sourced by the benchmark_*.sh scripts.
# Timing-sensitive sim results are invalid on a busy shared machine (missed control cycles, TF
# freezes). Refuse to start when the 1-min load average is above BENCH_MAX_LOAD (default 8).
#   BENCH_LOAD_FORCE=1   run anyway (prints a WARNING)
# Note: one headless sim adds ~10 load by itself, so the check is made BEFORE launching anything.
bench_load_guard() {
  local max="${BENCH_MAX_LOAD:-8}" load
  load=$(cut -d' ' -f1 /proc/loadavg 2>/dev/null || echo 0)
  if awk -v l="$load" -v m="$max" 'BEGIN{exit !(l>m)}'; then
    if [ "${BENCH_LOAD_FORCE:-0}" = "1" ]; then
      echo "[bench] WARNING: load $load > $max - results may be invalid (BENCH_LOAD_FORCE=1)" >&2
    else
      echo "[bench] load $load > $max (another job is running): refusing to start. Re-run when idle," >&2
      echo "[bench] or set BENCH_LOAD_FORCE=1 to override (results will be unreliable)." >&2
      exit 3
    fi
  fi
}
