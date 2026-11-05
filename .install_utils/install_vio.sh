#!/bin/bash
set -eo pipefail

# Set up Realsense: librealsense + realsense2_camera ROS wrapper
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor > /etc/apt/keyrings/librealsenseai.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/librealsense.list
apt-get update
apt-get install -y librealsense2-utils librealsense2-dev "ros-humble-realsense2-*"
rm -rf /var/lib/apt/lists/*

# Set up Basalt: build basalt_ros into its own workspace /opt/basalt_ws due to an Eigen clash
apt-get update
apt-get install -y python3-vcstool libtbb-dev libfmt-dev ros-humble-ament-cmake-clang-format \
    libopencv-dev ros-humble-cv-bridge ros-humble-image-transport
rm -rf /var/lib/apt/lists/*

mkdir -p /opt/basalt_ws/src && cd /opt/basalt_ws
vcs import --shallow --workers 1 < /tmp/basalt.repos
BASALT_SRC=src/basalt_wrapper/basalt
git -C $BASALT_SRC submodule update --init --depth 1 thirdparty/basalt-headers thirdparty/magic_enum
git -c url."https://github.com/eigen-mirror/eigen.git".insteadOf="https://gitlab.com/libeigen/eigen.git" \
    -C $BASALT_SRC/thirdparty/basalt-headers submodule update --init --depth 1 thirdparty/eigen thirdparty/Sophus thirdparty/cereal

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --merge-install --symlink-install --cmake-args \
    -DCMAKE_CXX_FLAGS="-w" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_DISABLE_FIND_PACKAGE_realsense2=ON -DJUST_INSTALL_CEREAL=ON
rm -rf log
