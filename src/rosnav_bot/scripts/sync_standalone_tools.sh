#!/usr/bin/env bash
# Copy the standalone tool packages + tests from their repos into rosnav (vendored copies).
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
for t in nav2_fault_inject nav_explore_eval; do
  src="$HOME/$t"; [ -d "$src/$t" ] || { echo "missing $src" >&2; exit 1; }
  rm -rf "$here/tools/$t"; mkdir -p "$here/tools/$t"
  cp -r "$src/$t" "$here/tools/$t/$t"; find "$here/tools/$t" -name __pycache__ -prune -exec rm -rf {} +
  mkdir -p "$here/test/$t"; cp "$src"/test/*.py "$here/test/$t/"
  echo "synced $t"
done
