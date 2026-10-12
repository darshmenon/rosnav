#!/bin/bash
# clean_fastdds_shm.sh — remove Fast DDS shared-memory leftovers in /dev/shm that no process has open.
# kill -9 of ROS/Gazebo processes leaves fastrtps_* segments behind; thousands of them can build up
# (seen 2026-10-12: 4,295 stale entries, 537 MB) and may disturb discovery/delivery.
# Only files owned by you AND not open by any running process are touched, so other live ROS sessions are safe.
# Usage: clean_fastdds_shm.sh          (dry run — lists what would go)
#        clean_fastdds_shm.sh --delete (really delete)
set -u
open=$(mktemp); stale=$(mktemp)
lsof -F n +D /dev/shm 2>/dev/null | sed -n 's/^n//p' | sort -u > "$open"
find /dev/shm -maxdepth 1 \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \) -user "$(id -un)" | sort > "$stale.all"
comm -23 "$stale.all" "$open" > "$stale"
echo "fastrtps entries: $(wc -l < "$stale.all"), open by live processes: $(comm -12 "$stale.all" "$open" | wc -l), stale: $(wc -l < "$stale")"
if [ "${1:-}" = "--delete" ]; then xargs -a "$stale" rm -f -- ; echo "deleted. /dev/shm now $(du -sh /dev/shm | cut -f1)"; else echo "dry run (use --delete to remove)"; fi
rm -f "$open" "$stale" "$stale.all"
