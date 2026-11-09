#!/bin/bash
set -eo pipefail

# Bootstrap tools needed by this and later install steps
apt-get update
apt-get install -y --no-install-recommends curl ca-certificates gnupg lsb-release sudo

# Keep only Ubuntu's 4.5 OpenCV (for Jetson)
if dpkg-query -W libopencv >/dev/null 2>&1; then
    apt-get purge -y libopencv libopencv-dev libopencv-python libopencv-samples nvidia-opencv nvidia-opencv-dev \
        opencv-licenses opencv-samples-data
fi

# Install ROS 2 Humble if the image doesn't come with it
if [ ! -f /opt/ros/humble/setup.bash ]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list
    apt-get update
    apt-get install -y --no-install-recommends ros-humble-ros-base build-essential git python3-rosdep \
        python3-colcon-common-extensions python3-colcon-mixin
    rosdep init
fi
rm -rf /var/lib/apt/lists/*
