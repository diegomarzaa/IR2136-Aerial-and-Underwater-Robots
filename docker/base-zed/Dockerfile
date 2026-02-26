# ============================================================
# zed:humble  —  ZED SDK base (Ubuntu 22.04)
# ============================================================
# Minimal ZED SDK foundation. Only runtime dependencies,
# no build tools, no ROS. Use as base for any ZED project.
#
# Change ZED_BASE_IMAGE at build time to pin a specific SDK version.
# Available Ubuntu 22.04 tags: https://hub.docker.com/r/stereolabs/zed/tags

ARG ZED_BASE_IMAGE=stereolabs/zed:5.2-gl-devel-cuda12.8-ubuntu22.04
FROM ${ZED_BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Madrid
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update && apt-get install -y --no-install-recommends \
    locales \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    python3 \
    python3-numpy \
    python3-opencv \
    libopencv-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility,graphics
