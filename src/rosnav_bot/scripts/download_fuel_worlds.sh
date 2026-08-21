#!/usr/bin/env bash
# Download textured Gazebo Fuel models used by camera / RTAB-Map 3D SLAM worlds.
# Vendors into src/rosnav_bot/models/fuel/ (gitignored). No git clones.
#
#   bash src/rosnav_bot/scripts/download_fuel_worlds.sh          # all
#   bash src/rosnav_bot/scripts/download_fuel_worlds.sh cafe
#   bash src/rosnav_bot/scripts/download_fuel_worlds.sh lake_house aws_warehouse
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FUEL_DIR="$(cd "$SCRIPT_DIR/../models" && pwd)/fuel"
CACHE_ROOT="${HOME}/.gz/fuel"

mkdir -p "$FUEL_DIR"

# name|fuel_url
MODELS_LAKE_HOUSE=(
  'Lake House|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lake House'
  'Armchair|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Armchair'
  'Bathtub|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bathtub'
  'Bed|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bed'
  'Dining Table|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dining Table'
  'Dining Chair|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dining Chair'
  'Refrigerator|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Refrigerator'
  'Oven|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oven'
  'Standard Toilet|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standard Toilet'
  'Vanity|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Vanity'
  'Office Desk|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Office Desk'
  'Office Chair|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Office Chair'
  'Piano|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Piano'
  'Pendulum Sculpture|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pendulum Sculpture'
)

MODELS_AWS_WAREHOUSE=(
  'aws_robomaker_warehouse_ShelfF_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfF_01'
  'aws_robomaker_warehouse_WallB_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_WallB_01'
  'aws_robomaker_warehouse_ShelfE_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfE_01'
  'aws_robomaker_warehouse_ShelfD_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfD_01'
  'aws_robomaker_warehouse_GroundB_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_GroundB_01'
  'aws_robomaker_warehouse_Lamp_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_Lamp_01'
  'aws_robomaker_warehouse_Bucket_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_Bucket_01'
  'aws_robomaker_warehouse_ClutteringA_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ClutteringA_01'
  'aws_robomaker_warehouse_ClutteringC_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ClutteringC_01'
  'aws_robomaker_warehouse_ClutteringD_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ClutteringD_01'
  'aws_robomaker_warehouse_TrashCanC_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_TrashCanC_01'
  'aws_robomaker_warehouse_PalletJackB_01|https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_PalletJackB_01'
)

MODELS_TUGBOT_WAREHOUSE=(
  'Warehouse|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse'
  'Tugbot-charging-station|https://fuel.gazebosim.org/1.0/MovAi/models/Tugbot-charging-station'
  'cart_model_2|https://fuel.gazebosim.org/1.0/MovAi/models/cart_model_2'
  'shelf_big|https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big'
  'shelf|https://fuel.gazebosim.org/1.0/MovAi/models/shelf'
  'pallet_box_mobile|https://fuel.gazebosim.org/1.0/MovAi/models/pallet_box_mobile'
)

MODELS_CAFE=(
  'Cafe|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cafe'
  'Cafe table|https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cafe table'
)

slugify_url() {
  python3 -c 'import sys,urllib.parse; print(urllib.parse.unquote(sys.argv[1]).lower())' "$1"
}

find_cache_dir() {
  local url="$1"
  local owner model
  owner="$(python3 -c 'import sys,urllib.parse; p=urllib.parse.urlparse(sys.argv[1]).path.strip("/").split("/"); print(p[1].lower())' "$url")"
  model="$(python3 -c 'import sys,urllib.parse; p=urllib.parse.urlparse(sys.argv[1]).path.strip("/").split("/"); print(urllib.parse.unquote(p[3]).lower())' "$url")"
  local d
  for host in fuel.gazebosim.org fuel.ignitionrobotics.org; do
    d="${CACHE_ROOT}/${host}/${owner}/models/${model}"
    if [[ -d "$d" ]]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

vendor_model() {
  local name="$1"
  local url="$2"
  local dst="${FUEL_DIR}/${name}"
  if [[ -f "${dst}/model.sdf" || -f "${dst}/model.config" ]]; then
    echo "[fuel] skip (already vendored): ${name}"
    return 0
  fi

  echo "[fuel] download: ${url}"
  if ! gz fuel download -v 3 -u "$url"; then
    echo "[fuel] ERROR: gz fuel download failed for ${url}" >&2
    return 1
  fi

  local src_root ver
  if ! src_root="$(find_cache_dir "$url")"; then
    echo "[fuel] ERROR: cache not found after download: ${url}" >&2
    return 1
  fi
  ver="$(ls "$src_root" | grep -E '^[0-9]+$' | sort -n | tail -1)"
  echo "[fuel] ${name}  cache ${src_root}/${ver}  →  ${dst}"
  rm -rf "$dst"
  cp -a "${src_root}/${ver}" "$dst"
  rm -rf "${dst}/thumbnails"
  if grep -q '<static>false</static>' "${dst}/model.sdf" 2>/dev/null; then
    sed -i 's/<static>false<\/static>/<static>true<\/static>/' "${dst}/model.sdf"
  fi
  du -sh "$dst"
}

vendor_list() {
  local entry name url
  for entry in "$@"; do
    name="${entry%%|*}"
    url="${entry#*|}"
    vendor_model "$name" "$url"
  done
}

want_all=true
declare -a SELECTED=()
if [[ $# -gt 0 ]]; then
  want_all=false
  SELECTED=("$@")
fi

should_do() {
  local key="$1"
  if $want_all; then return 0; fi
  local s
  for s in "${SELECTED[@]}"; do
    [[ "$s" == "$key" || "$s" == all ]] && return 0
  done
  return 1
}

if should_do lake_house; then
  echo "[fuel] === lake_house (PBR house + furniture) ==="
  vendor_list "${MODELS_LAKE_HOUSE[@]}"
fi
if should_do aws_warehouse; then
  echo "[fuel] === aws_warehouse (AWS RoboMaker textured warehouse) ==="
  vendor_list "${MODELS_AWS_WAREHOUSE[@]}"
fi
if should_do tugbot_warehouse; then
  echo "[fuel] === tugbot_warehouse (OpenRobotics Warehouse + shelves) ==="
  vendor_list "${MODELS_TUGBOT_WAREHOUSE[@]}"
fi
if should_do cafe; then
  echo "[fuel] === cafe (textured cafe interior) ==="
  vendor_list "${MODELS_CAFE[@]}"
fi

echo
echo "[fuel] Vendored under ${FUEL_DIR}"
du -sh "$FUEL_DIR" 2>/dev/null || true
echo "[fuel] Rebuild: colcon build --packages-select rosnav_bot"
echo "[fuel] Then:"
echo "  ros2 launch rosnav_bot slam_nav.launch.py world_name:=lake_house enable_camera:=true lidar_type:=3d slam_algo:=3d"
echo "  ros2 launch rosnav_bot slam_nav.launch.py world_name:=aws_warehouse enable_camera:=true lidar_type:=3d slam_algo:=3d"
echo "  ros2 launch rosnav_bot slam_nav.launch.py world_name:=tugbot_warehouse enable_camera:=true lidar_type:=3d slam_algo:=3d"
echo "  ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe enable_camera:=true lidar_type:=3d slam_algo:=3d"
