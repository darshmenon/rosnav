#!/usr/bin/env bash
# Bare-metal alternative to docker/orb_slam3/Dockerfile — same recipe, no
# Docker. Builds OpenCV 4.4.0 + Pangolin v0.9.1 + ORB-SLAM3 + the ROS2
# wrapper directly on this host, into a workspace separate from rosnav_bot's
# own (~/orb_slam3_ws, not this repo's src/) — same reasoning as the Docker
# image: ORB-SLAM3 is GPLv3 (rtabmap_slam is BSD) and has no ROS distro apt
# package, so it's kept out of the main colcon workspace either way.
#
# OpenCV/Pangolin install to $ORB_SLAM3_PREFIX (default ~/.local), not
# /usr/local — keeps the whole build sudo-free after the one-time apt
# install below, and avoids touching system-wide library paths other
# projects on a shared box might depend on.
#
# Only run this if the Docker sidecar (docker/orb_slam3/) isn't an option
# for you — it's the same multi-stage native build either way (~30-90 min),
# just without container isolation. Don't run this at the same time as
# `docker build ... docker/orb_slam3/Dockerfile` — both compile the exact
# same OpenCV/Pangolin/ORB-SLAM3 stack and will fight over CPU/RAM.
#
# Usage:
#   bash docker/orb_slam3/build_bare_metal.sh
#   # then:
#   source ~/orb_slam3_ws/install/setup.bash
#   ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py sensor_config:=rgbd
set -euo pipefail

WS="${ORB_SLAM3_WS:-$HOME/orb_slam3_ws}"
PREFIX="${ORB_SLAM3_PREFIX:-$HOME/.local}"
BUILD_JOBS="${BUILD_JOBS:-4}"   # see docker/orb_slam3/Dockerfile's comment on why not `nproc`
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

echo "[build_orb_slam3] workspace: $WS"
echo "[build_orb_slam3] install prefix: $PREFIX"
echo "[build_orb_slam3] parallel jobs: $BUILD_JOBS"

# Only touch apt if something's actually missing — this host already had
# nearly everything from other ROS projects when this was last run; no
# reason to demand sudo for packages already present. GTK/GStreamer dev
# headers are deliberately not in this list: they're only for OpenCV's own
# highgui (imshow, video file I/O), which ORB-SLAM3 doesn't use (its viewer
# is Pangolin) — OpenCV's cmake auto-disables that support and prints a
# status line if those headers are missing, it doesn't fail the build.
_apt_pkgs=(
  cmake build-essential git unzip pkg-config wget
  python3-dev python3-numpy python3-pip
  libgl1-mesa-dev libglew-dev libpython3-dev libeigen3-dev
  libavcodec-dev libavformat-dev libswscale-dev
  ros-humble-pcl-ros ros-humble-cv-bridge ros-humble-image-transport
  ros-humble-vision-opencv ros-humble-tf2-eigen
)
_missing=()
for p in "${_apt_pkgs[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || _missing+=("$p")
done
if [[ ${#_missing[@]} -gt 0 ]]; then
  echo "[build_orb_slam3] Missing apt packages: ${_missing[*]}"
  echo "[build_orb_slam3] Run this once (needs sudo), then re-run this script:"
  echo "  sudo apt-get update && sudo apt-get install -y --no-install-recommends ${_missing[*]}"
  exit 1
fi
echo "[build_orb_slam3] All apt deps already present, skipping."

mkdir -p "$PREFIX"
export CMAKE_PREFIX_PATH="$PREFIX:${CMAKE_PREFIX_PATH:-}"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="$PREFIX/lib:${LD_LIBRARY_PATH:-}"

# OpenCV 4.4.0 — same version pin as the Docker image, same reasoning
# (ORB-SLAM3's build is validated against it; newer OpenCV has broken this
# build before). Checked via its CMake package config, not pkg-config —
# OpenCV 4.4 doesn't generate a .pc file unless OPENCV_GENERATE_PKGCONFIG=ON
# is explicitly passed (confirmed: a pkg-config-based check here always
# missed an already-installed 4.4.0 and silently triggered a full rebuild).
if ! grep -q 'OpenCV_VERSION 4.4.0' "$PREFIX/lib/cmake/opencv4/OpenCVConfig-version.cmake" 2>/dev/null; then
  tmp_cv="$(mktemp -d)"
  git clone --depth 1 --branch 4.4.0 https://github.com/opencv/opencv.git "$tmp_cv/opencv"
  cmake -S "$tmp_cv/opencv" -B "$tmp_cv/opencv/build" \
    -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF \
    -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX="$PREFIX"
  cmake --build "$tmp_cv/opencv/build" -j"$BUILD_JOBS"
  cmake --install "$tmp_cv/opencv/build"
  rm -rf "$tmp_cv"
else
  echo "[build_orb_slam3] OpenCV 4.4.0 already installed, skipping"
fi

# Pangolin v0.9.1
if [[ ! -f "$PREFIX/lib/cmake/Pangolin/PangolinConfig.cmake" ]]; then
  tmp_pango="$(mktemp -d)"
  git clone --depth 1 --branch v0.9.1 https://github.com/stevenlovegrove/Pangolin "$tmp_pango/Pangolin"
  cmake -S "$tmp_pango/Pangolin" -B "$tmp_pango/Pangolin/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX="$PREFIX"
  cmake --build "$tmp_pango/Pangolin/build" -j"$BUILD_JOBS"
  cmake --install "$tmp_pango/Pangolin/build"
  rm -rf "$tmp_pango"
else
  echo "[build_orb_slam3] Pangolin v0.9.1 already installed, skipping"
fi

# ORB-SLAM3 core (suchetanrs' fork — see docker/orb_slam3/Dockerfile comment
# on why this fork, not upstream UZ-SLAMLab/ORB_SLAM3). build.sh installs its
# own Thirdparty libs (DBoW2/g2o/Sophus) under its own tree, not $PREFIX.
mkdir -p "$HOME/orb_slam3_src"
if [[ ! -d "$HOME/orb_slam3_src/ORB_SLAM3" ]]; then
  git clone --depth 1 https://github.com/suchetanrs/ORB_SLAM3.git "$HOME/orb_slam3_src/ORB_SLAM3"
fi
( set +u; source /opt/ros/humble/setup.bash; set -u
  cd "$HOME/orb_slam3_src/ORB_SLAM3" && chmod +x build.sh && ./build.sh )

# ROS2 wrapper (orb_slam3_ros2_wrapper, slam_msgs, orb_slam3_map_generator) —
# plain subdirectories of suchetanrs/ORB-SLAM3-ROS2-Docker, not standalone
# repos (see docker/orb_slam3/Dockerfile).
mkdir -p "$WS/src"
if [[ ! -d "$HOME/orb_slam3_src/ORB-SLAM3-ROS2-Docker" ]]; then
  git clone --depth 1 https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git \
    "$HOME/orb_slam3_src/ORB-SLAM3-ROS2-Docker"
fi
for pkg in orb_slam3_ros2_wrapper slam_msgs orb_slam3_map_generator; do
  ln -sfn "$HOME/orb_slam3_src/ORB-SLAM3-ROS2-Docker/$pkg" "$WS/src/$pkg"
done

# orb_slam3_ros2_wrapper/CMakeModules/FindORB_SLAM3.cmake hardcodes
# `set(ORB_SLAM3_ROOT_DIR "/home/orb/ORB_SLAM3")` — a plain set(), not an
# environment-variable read, despite the comment above it claiming
# ORB_SLAM3_ROOT_DIR is configurable via env var. That hardcode is correct
# for the Docker image (which places ORB_SLAM3 at exactly that path) but
# wrong here, where it's cloned to $HOME/orb_slam3_src/ORB_SLAM3 — patch it
# in our local clone (not upstream) so colcon build can find the library.
sed -i "s#/home/orb/ORB_SLAM3#$HOME/orb_slam3_src/ORB_SLAM3#" \
  "$HOME/orb_slam3_src/ORB-SLAM3-ROS2-Docker/orb_slam3_ros2_wrapper/CMakeModules/FindORB_SLAM3.cmake"

# Swap in rosnav_bot's own camera topics/frames/intrinsics — same two files
# the Docker image overwrites, see docker/orb_slam3/README.md.
cp "$REPO_ROOT/docker/orb_slam3/params/rosnav_rgbd.yaml" \
   "$WS/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/gazebo_rgbd.yaml"
cp "$REPO_ROOT/docker/orb_slam3/params/rosnav_rgbd_ros_params.yaml" \
   "$WS/src/orb_slam3_ros2_wrapper/params/ros_params/gazebo-rgbd-ros-params.yaml"

( set +u; source /opt/ros/humble/setup.bash; set -u
  cd "$WS" && colcon build --symlink-install --cmake-args \
    -DCMAKE_PREFIX_PATH="$PREFIX" )

echo
echo "[build_orb_slam3] Done. Run:"
echo "  export LD_LIBRARY_PATH=\"$PREFIX/lib:\$LD_LIBRARY_PATH\""
echo "  source $WS/install/setup.bash"
echo "  ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py sensor_config:=rgbd"
