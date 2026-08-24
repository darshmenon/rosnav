# ORB-SLAM3 (ROS 2 Humble) as a separate sidecar container — feature-based
# visual SLAM, for comparison against this repo's native RTAB-Map backends
# (slam_algo:=vslam|3d|multisensor, built via rosdep in the main Dockerfile).
#
# Not folded into the main colcon workspace: ORB-SLAM3 needs a from-source
# OpenCV/Pangolin/ORB-SLAM3 native build (no ROS distro apt package exists),
# is GPLv3 (rtabmap_slam is BSD), and the build alone takes ~30-60 min — a
# separate opt-in image keeps the main rosnav_bot image fast to build and
# license-clean. Recipe follows the working build in
# https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker (CPU target); this
# file re-derives it instead of vendoring that repo, and swaps in
# rosnav_bot's own camera topics/frames/intrinsics (see params/).
#
# Build:  docker build -t rosnav_orb_slam3:humble -f docker/orb_slam3/Dockerfile .
# Run:    docker compose up orb_slam3        (needs the Gazebo sim running
#                                              separately — see docker/orb_slam3/README.md)
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
# Parallel compile jobs for OpenCV/Pangolin/ORB-SLAM3 — default is
# conservative, not `nproc`, since a full-width build of all three at once
# is easy to OOM a memory-constrained box. Raise with
# `--build-arg BUILD_JOBS=$(nproc)` if you have RAM to spare.
ARG BUILD_JOBS=4

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake build-essential git unzip pkg-config wget \
    python3-dev python3-numpy python3-pip \
    libgl1-mesa-dev libglew-dev libpython3-dev libeigen3-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev \
    ca-certificates apt-transport-https software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# OpenCV 4.4.0 — matches the ORB-SLAM3 version this build.sh/CMake target
# is validated against (newer OpenCV's ABI has broken ORB-SLAM3's build in
# the past); skip the ~10min rebuild if you already trust system OpenCV.
RUN cd /tmp && git clone --depth 1 --branch 4.4.0 https://github.com/opencv/opencv.git && \
    cd opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF \
          -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j${BUILD_JOBS} && make install && ldconfig && \
    cd / && rm -rf /tmp/opencv

# Pangolin v0.9.1 — ORB-SLAM3's viewer. Headless boxes still need this to
# link; the viewer window itself only needs X11 (visualization:false in
# params/ to skip it if you have no display).
RUN cd /tmp && git clone --depth 1 --branch v0.9.1 https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j${BUILD_JOBS} && make install && ldconfig && \
    cd / && rm -rf /tmp/Pangolin

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-ros ros-humble-rviz2 ros-humble-cv-bridge \
    ros-humble-image-transport ros-humble-vision-opencv \
    ros-humble-tf2-eigen tmux \
    && rm -rf /var/lib/apt/lists/*

# ORB-SLAM3 core library (suchetanrs' fork — carries the CMake install
# target + threading fixes the ROS2 wrapper below builds against; upstream
# UZ-SLAMLab/ORB_SLAM3 doesn't export an installable target).
RUN git clone --depth 1 https://github.com/suchetanrs/ORB_SLAM3.git /home/orb/ORB_SLAM3

# The ROS2 wrapper (orb_slam3_ros2_wrapper, slam_msgs, orb_slam3_map_generator)
# lives as plain subdirectories in this repo, not standalone repos — clone it
# once and pull those three into the colcon workspace.
RUN git clone --depth 1 https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git /tmp/orb_upstream && \
    mkdir -p /root/colcon_ws/src && \
    mv /tmp/orb_upstream/orb_slam3_ros2_wrapper /root/colcon_ws/src/ && \
    mv /tmp/orb_upstream/slam_msgs /root/colcon_ws/src/ && \
    mv /tmp/orb_upstream/orb_slam3_map_generator /root/colcon_ws/src/ && \
    rm -rf /tmp/orb_upstream

# Swap in rosnav_bot's own camera topics/frames/intrinsics (see this repo's
# urdf/rgbd_camera.xacro) in place of the upstream demo robot's config —
# these overwrite the exact files rgbd.launch.py defaults to.
COPY docker/orb_slam3/params/rosnav_rgbd.yaml \
     /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/gazebo_rgbd.yaml
COPY docker/orb_slam3/params/rosnav_rgbd_ros_params.yaml \
     /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/ros_params/gazebo-rgbd-ros-params.yaml

RUN . /opt/ros/humble/setup.sh && \
    cd /home/orb/ORB_SLAM3 && chmod +x build.sh && ./build.sh && \
    . /opt/ros/humble/setup.sh && \
    cd /root/colcon_ws && colcon build --symlink-install

COPY docker/orb_slam3/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "orb_slam3_ros2_wrapper", "unirobot.launch.py", "sensor_config:=rgbd"]
