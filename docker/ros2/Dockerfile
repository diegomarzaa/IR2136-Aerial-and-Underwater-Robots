# ZED ROS 2 Jazzy - Extends existing ZED SDK image
# Based on: stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

FROM stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

ARG ROS2_DIST=jazzy
ARG DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility,graphics

# Timezone and locale setup
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    locales \
    lsb-release \
    wget \
    curl \
    gnupg2 \
    software-properties-common \
    build-essential \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    git && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

############ Install ROS 2 Jazzy ############

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy base packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS2_DIST}-ros-base \
    ros-${ROS2_DIST}-rviz2 \
    ros-${ROS2_DIST}-camera-calibration \
    ros-${ROS2_DIST}-image-transport \
    ros-${ROS2_DIST}-image-transport-plugins \
    ros-${ROS2_DIST}-diagnostic-updater \
    ros-${ROS2_DIST}-xacro \
    ros-${ROS2_DIST}-robot-localization \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# Ensure OpenCV/NumPy compatibility for ROS Python nodes (camera_calibration imports cv2)
# Some base images ship pip NumPy 2.x in /usr/local, while apt cv2 may be built against NumPy 1.x.
RUN python3 -m pip uninstall -y --break-system-packages numpy || true && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-opencv && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

############ Build ZED ROS 2 Wrapper ############

ENV ROS_DISTRO=${ROS2_DIST}

# Create workspace and copy sources
WORKDIR /root/ros2_ws/src

# Clone the ZED ROS 2 wrapper (use master branch which supports Jazzy)
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

# Install zed_msgs
RUN git clone https://github.com/stereolabs/zed-ros2-interfaces.git

WORKDIR /root/ros2_ws

# Install dependencies and build
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --parallel-workers $(nproc) --symlink-install \
    --event-handlers console_direct+ --base-paths src \
    --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
    ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    ' -DCMAKE_CXX_FLAGS=\"-Wl,--allow-shlib-undefined\"' " && \
    rm -rf /var/lib/apt/lists/*

# Setup entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Setup ROS 2 environment\n\
source /opt/ros/'${ROS2_DIST}'/setup.bash\n\
source /root/ros2_ws/install/local_setup.bash\n\
\n\
# ROS Domain ID (change if needed)\n\
export ROS_DOMAIN_ID=0\n\
\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

WORKDIR /root/ros2_ws

# Auto-source ROS 2 and workspace in interactive shells (e.g. docker exec ... bash)
RUN echo "source /opt/ros/${ROS2_DIST}/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/local_setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
