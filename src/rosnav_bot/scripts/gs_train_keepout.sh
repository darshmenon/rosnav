#!/usr/bin/env bash
# gs_train_keepout.sh — capture → splatfacto train → export → keepout mask.
#
# Full pipeline (needs Gazebo for capture, nerfstudio venv for train/export):
#   bash src/rosnav_bot/scripts/gs_train_keepout.sh cafe
#
# Skip capture / use existing data:
#   SKIP_CAPTURE=1 DATA_DIR=$HOME/gs_data/cafe bash .../gs_train_keepout.sh cafe
#
# Rasterize an existing .npz only (no training):
#   MASK_ONLY=1 NPZ=$HOME/gs_data/cafe_points_final.npz \
#     ALIGN_TO=src/rosnav_bot/maps/map_hospital.yaml \
#     bash .../gs_train_keepout.sh cafe
#
# Then launch Nav2 with the mask:
#   ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe \
#     gs_keepout_mask:=src/rosnav_bot/maps/gs_keepout_cafe.yaml
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"

WORLD="${1:-cafe}"
DATA_DIR="${DATA_DIR:-$HOME/gs_data/${WORLD}}"
NS_VENV="${NS_VENV:-$HOME/venvs/nerfstudio}"
OUT_MASK="${OUT_MASK:-$ROOT/src/rosnav_bot/maps/gs_keepout_${WORLD}.yaml}"
ALIGN_TO="${ALIGN_TO:-}"
MAX_ITERS="${MAX_ITERS:-30000}"
SKIP_CAPTURE="${SKIP_CAPTURE:-0}"
MASK_ONLY="${MASK_ONLY:-0}"
NPZ="${NPZ:-}"
Z_MIN="${Z_MIN:-0.05}"
Z_MAX="${Z_MAX:-2.0}"
# densest (100 - pct)%% of nonzero cells → occupied (matches --density-percentile)
DENSITY_PERCENTILE="${DENSITY_PERCENTILE:-90}"

# ROS setup scripts reference unset vars; enable nounset only after sourcing.
# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/install/setup.bash"
fi
set -u
mask_from_npz() {
  local npz="$1"
  local align_args=()
  if [[ -n "$ALIGN_TO" ]]; then
    align_args=(--align-to "$ALIGN_TO")
  fi
  echo "[gs_train_keepout] rasterize $npz → $OUT_MASK"
  python3 "$ROOT/src/rosnav_bot/scripts/gs_mask_from_splat.py" \
    --npz "$npz" \
    --out "$OUT_MASK" \
    --z-min "$Z_MIN" --z-max "$Z_MAX" \
    --density-percentile "$DENSITY_PERCENTILE" \
    "${align_args[@]}"
  echo "[gs_train_keepout] done. Launch with:"
  echo "  ros2 launch rosnav_bot slam_nav.launch.py world_name:=${WORLD} gs_keepout_mask:=${OUT_MASK}"
}

if [[ "$MASK_ONLY" == "1" ]]; then
  if [[ -z "$NPZ" || ! -f "$NPZ" ]]; then
    echo "MASK_ONLY=1 requires NPZ=/path/to/points.npz" >&2
    exit 2
  fi
  mask_from_npz "$NPZ"
  exit 0
fi

if [[ "$SKIP_CAPTURE" != "1" ]]; then
  echo "[gs_train_keepout] launching capture (world=$WORLD) …"
  # User is expected to have Gazebo free; this is the documented capture path.
  ros2 launch rosnav_bot gs_capture.launch.py world_name:="$WORLD" &
  LAUNCH_PID=$!
  sleep 8
  ros2 run rosnav_bot gs_capture.py --ros-args \
    -p world_name:="$WORLD" -p out_dir:="$DATA_DIR" || true
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
else
  echo "[gs_train_keepout] SKIP_CAPTURE=1 — using $DATA_DIR"
fi

if [[ ! -f "$DATA_DIR/transforms.json" ]]; then
  echo "Missing $DATA_DIR/transforms.json — capture first or set DATA_DIR" >&2
  exit 2
fi

if [[ ! -x "$NS_VENV/bin/ns-train" ]]; then
  echo "nerfstudio venv not found at $NS_VENV (set NS_VENV=...)" >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$NS_VENV/bin/activate"

echo "[gs_train_keepout] ns-train splatfacto (max_num_iterations=$MAX_ITERS) …"
# Outputs land under $PWD/outputs/<…>/splatfacto/<timestamp>/
ns-train splatfacto \
  --data "$DATA_DIR" \
  --pipeline.model.random-init True \
  --max-num-iterations "$MAX_ITERS" \
  nerfstudio-data

CONFIG="$(find outputs -path '*/splatfacto/*/config.yml' -printf '%T@ %p\n' 2>/dev/null \
  | sort -nr | head -1 | cut -d' ' -f2- || true)"
if [[ -z "$CONFIG" || ! -f "$CONFIG" ]]; then
  echo "Could not locate splatfacto config.yml under outputs/" >&2
  exit 2
fi
echo "[gs_train_keepout] using config: $CONFIG"

EXPORT_DIR="$DATA_DIR/splat_export"
mkdir -p "$EXPORT_DIR"
ns-export gaussian-splat --load-config "$CONFIG" --output-dir "$EXPORT_DIR"

PLY="$EXPORT_DIR/splat.ply"
if [[ ! -f "$PLY" ]]; then
  # some nerfstudio versions nest the ply
  PLY="$(find "$EXPORT_DIR" -name 'splat.ply' | head -1 || true)"
fi
if [[ -z "$PLY" || ! -f "$PLY" ]]; then
  echo "splat.ply not found under $EXPORT_DIR" >&2
  exit 2
fi

NPZ_OUT="$DATA_DIR/points.npz"
python3 "$ROOT/src/rosnav_bot/scripts/gs_splat_to_pointcloud.py" "$PLY" "$NPZ_OUT"
deactivate || true

mask_from_npz "$NPZ_OUT"
