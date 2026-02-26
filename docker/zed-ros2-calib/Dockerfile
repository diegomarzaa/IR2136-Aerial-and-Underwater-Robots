# ============================================================
# zed-ros2-calib:2026-02  —  ZED SDK + ROS 2 Humble + Calibration
# ============================================================
# Two-stage build:
#   builder  — pulls ROS2 from ros2-humble:humble (single source of truth),
#              adds the ROS2 apt repo for rosdep, compiles ZED ROS2 wrapper
#              and calibration binary.
#   runtime  — ZED SDK base + /opt/ros/humble + compiled artifacts only.
#              No build tools, no apt sources needed at runtime.
#
# To add a new ROS package to this image:
#   1. Add it to docker/ros2-humble/Dockerfile  (or your global humble image)
#   2. Rebuild ros2-humble:humble
#   3. Rebuild this image  ← picks it up automatically via COPY --from=ros2

ARG ZED_BASE=zed:humble
ARG ROS2_IMAGE=ros2-humble:humble

# ── Stage 1: builder ─────────────────────────────────────────
FROM ${ROS2_IMAGE} AS ros2

FROM ${ZED_BASE} AS builder

ARG ROS_DISTRO=humble
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Pull the ROS 2 installation from the ros2-humble image.
# Only /opt/ros/humble is needed — it contains all installed ROS packages.
COPY --from=ros2 /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}

# Add the ROS 2 apt repo so rosdep can install missing system deps at build time.
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    rm -rf /var/lib/apt/lists/*

# Build tools (only in this stage, NOT copied to runtime)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config git \
    python3-pip python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libglew-dev freeglut3-dev \
    libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    && rosdep init || true && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Build ZED ROS 2 workspace
WORKDIR /root/ros2_ws/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    git clone https://github.com/stereolabs/zed-ros2-interfaces.git

WORKDIR /root/ros2_ws
RUN /bin/bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --parallel-workers $(nproc) \
    --base-paths src \
    --cmake-args '-DCMAKE_BUILD_TYPE=Release' \
    '-DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    '-DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined' && \
    rm -rf /var/lib/apt/lists/*"

# Build calibration binary
COPY external/zed-opencv-calibration /build/zed-opencv-calibration
RUN mkdir -p /build/zed-opencv-calibration/build && \
    cd /build/zed-opencv-calibration/build && \
    cmake .. \
    -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
    -DCUDA_CUDA_LIBRARY=/usr/local/cuda/lib64/stubs/libcuda.so \
    -DCMAKE_EXE_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
    -DCMAKE_SHARED_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
    && make -j$(nproc)

# ── Stage 2: runtime ─────────────────────────────────────────
FROM ${ZED_BASE}

ARG ROS_DISTRO=humble
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Add ROS 2 apt repo and install system-level dependencies.
# ros-humble-ros-base pulls all the /usr/lib deps (libspdlog, etc.) that
# /opt/ros/humble packages depend on. The COPY below then OVERRIDES
# /opt/ros/humble with the full package set from ros2-humble:humble.
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    libglew2.2 freeglut3 \
    && rm -rf /var/lib/apt/lists/*

# Override /opt/ros/humble with the FULL package set from ros2-humble:humble
# (nav2, image-pipeline, everything). System libs from the apt step above remain.
COPY --from=ros2 /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}

# Compiled ZED ROS 2 workspace
COPY --from=builder /root/ros2_ws/install /root/ros2_ws/install

# Pre-built calibration binary
COPY --from=builder /build/zed-opencv-calibration/build/stereo_calibration/zed_stereo_calibration \
    /usr/local/bin/zed_stereo_calibration

# Calibration source (config files — no recompilation at runtime)
COPY external/zed-opencv-calibration /workspace/external/zed-opencv-calibration

# Entrypoint
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/%s/setup.bash\nsource /root/ros2_ws/install/local_setup.bash\nexport ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}\nexec "$@"\n' \
    "${ROS_DISTRO}" > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/local_setup.bash" >> /root/.bashrc

WORKDIR /workspace/external/zed-opencv-calibration
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
