#! /bin/bash
# Builds basalt_ros in an isolated workspace outside /home/$USER/ws, since that
# tree is bind-mounted over at container runtime and would hide anything built
# into it at image-build time. Runs at `docker compose build` (make build).
set -e

BASALT_REPO_URL=https://github.com/ForzaETH/basalt_ros.git
BASALT_WS=/opt/basalt_ws

apt update
apt install -y \
    python3-vcstool \
    libbz2-dev \
    ros-humble-ament-cmake-clang-format \
    libtbb-dev \
    libopencv-dev \
    libboost-all-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglew-dev \
    liblz4-dev \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    python3-numpy

mkdir -p "$BASALT_WS/src"
git clone "$BASALT_REPO_URL" "$BASALT_WS/src/basalt_ros"
cd "$BASALT_WS"
vcs import --recursive < "$BASALT_WS/src/basalt_ros/basalt_ros.repos"
vcs import --recursive < "$BASALT_WS/src/basalt_wrapper/basalt_wrapper.repos"


source /opt/ros/${ROS_DISTRO}/setup.bash
# One package at a time, with as many compile jobs as cores and available RAM allow
# (~2 GB per Eigen-heavy basalt TU): full parallelism otherwise freezes the host.
MEM_JOBS=$(( $(awk '/MemAvailable/ {print $2}' /proc/meminfo) / (2 * 1024 * 1024) ))
JOBS=$(( MEM_JOBS < $(nproc) ? MEM_JOBS : $(nproc) ))
JOBS=$(( JOBS > 0 ? JOBS : 1 ))
echo "Building basalt with $JOBS parallel jobs"
MAKEFLAGS="-j$JOBS" colcon build --merge-install --symlink-install \
    --executor sequential \
    --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
