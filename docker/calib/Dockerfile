# Calibration image for zed-opencv-calibration
FROM stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    build-essential \
    pkg-config \
    python3 \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-opencv \
    usbutils \
    pciutils \
    libopencv-dev \
    libglew-dev \
    freeglut3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    && rm -rf /var/lib/apt/lists/*

# Some base images include /usr/local NumPy 2.x that breaks apt OpenCV wheels.
# Keep apt-provided NumPy/OpenCV as canonical runtime in this container.
RUN python3 -m pip uninstall -y --break-system-packages numpy || true

WORKDIR /workspace/Repos/zed-opencv-calibration
COPY Repos/zed-opencv-calibration /workspace/Repos/zed-opencv-calibration
COPY docker/calib/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
