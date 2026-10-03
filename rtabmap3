# rosnav_bot dev image: ROS 2 Humble + Gazebo + Nav2 + SLAM stack (slam_toolbox,
# RTAB-Map, Cartographer), built the same way the bare-metal README does — via
# rosdep against package.xml — so this stays in sync automatically as
# dependencies change instead of drifting from a hand-maintained apt list.
#
# Build:
#   docker build -t rosnav_bot:humble .
# Run (GUI via X11, see docker-compose.yml for the full flags):
#   xhost +local:root
#   docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix rosnav_bot:humble \
#     ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital explore:=true
#
# Prefer `docker compose up rosnav` — it wires the X11/DISPLAY/net:=host flags
# for you and bind-mounts src/ so edits on the host rebuild without a full
# `docker build`.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy only package manifests first so rosdep/apt layers cache across
# src/ code changes (only busts when package.xml/CMakeLists.txt change).
COPY src/rosnav_bot/package.xml src/rosnav_bot/package.xml
COPY src/rosnav_bot/CMakeLists.txt src/rosnav_bot/CMakeLists.txt

# apt-get update is required here even though the earlier layer already ran
# it once — that layer's `rm -rf /var/lib/apt/lists/*` cleanup wipes the
# package index, so rosdep's own internal `apt-get install` calls would
# otherwise fail with "Unable to locate package" for every ros-humble-*
# dependency (confirmed: this is exactly what happened before this line
# was added).
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys "rmf_fleet_adapter_python rmf_traffic_ros2 rmf_task_ros2 rmf_task_msgs rmf_fleet_msgs" \
    && rm -rf /var/lib/apt/lists/*
# Open-RMF packages (skipped above) aren't in every rosdistro apt mirror —
# rmf_fleet.launch.py is optional/experimental (see concepts.md §11b); the
# rest of the stack (SLAM/Nav2/multi-robot/exploration) doesn't need them.
# Uncomment if your mirror has them and you want rmf_fleet.launch.py:
#   RUN apt-get update && apt-get install -y --no-install-recommends \
#         ros-humble-rmf-fleet-adapter ros-humble-rmf-task-ros2 \
#         ros-humble-rmf-fleet-msgs ros-humble-rmf-task-msgs || true

COPY . /workspace

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --packages-select rosnav_bot

# YOLO object detection (yolo_detector.py, enable_yolo:=true) — optional,
# heavy (~2GB with torch). Skip with `docker build --build-arg WITH_YOLO=false`.
# Installed AFTER colcon build, not before: ultralytics drags in a newer
# setuptools that breaks Humble's ament_cmake_python egg-info step
# (setuptools>=80's canonicalize_version() drops a kwarg the vendored
# distutils shim still passes) — confirmed this broke the build when it ran
# pre-colcon. colcon build itself never needs ultralytics, only the runtime
# yolo_detector.py node does, so ordering it after is a free fix, not a
# workaround.
ARG WITH_YOLO=true
RUN if [ "$WITH_YOLO" = "true" ]; then pip3 install --no-cache-dir ultralytics; fi

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
