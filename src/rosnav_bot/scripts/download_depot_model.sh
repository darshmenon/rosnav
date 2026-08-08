#!/usr/bin/env bash
# Download OpenRobotics Depot via Gazebo Fuel (HTTP) — no git required.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$(cd "$SCRIPT_DIR/../models" && pwd)"
DST="$MODELS_DIR/Depot"
URL="https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot"

echo "[download_depot] Fuel download: $URL"
gz fuel download -u "$URL"

SRC_ROOT="$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/depot"
if [[ ! -d "$SRC_ROOT" ]]; then
  echo "[download_depot] ERROR: expected cache at $SRC_ROOT" >&2
  exit 1
fi

VER="$(ls "$SRC_ROOT" | sort -n | tail -1)"
echo "[download_depot] Using Fuel version $VER → $DST"
rm -rf "$DST"
cp -a "$SRC_ROOT/$VER" "$DST"
rm -rf "$DST/thumbnails"
# Warehouse include expects a static building
if grep -q '<static>false</static>' "$DST/model.sdf"; then
  sed -i 's/<static>false<\/static>/<static>true<\/static>/' "$DST/model.sdf"
fi

du -sh "$DST"
echo "[download_depot] Done. Rebuild: colcon build --packages-select rosnav_bot"
