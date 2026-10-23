#! /bin/bash
# Builds basalt_ros in an isolated workspace outside /home/$USER/ws, since that
# tree is bind-mounted over at container runtime and would hide anything built
# into it at image-build time. Runs at `docker compose build` (make build).
set -e

# Bootstrap tools (other repos still work, so this update succeeds)
if [ -f /etc/nv_tegra_release ] || grep -qiE "nvidia|tegra" /proc/device-tree/model 2>/dev/null; then

	grep -rlE "packages\.ros\.org" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null | xargs -r rm -f
	apt update
	apt install -y --no-install-recommends curl ca-certificates gnupg
	# Fresh key + fresh source entry
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
fi

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
BASALT_WS=/opt/basalt_ws

apt update
# Detect pre-installed OpenCV (e.g. dustynv Jetson images ship 4.8.1 with CUDA)
if pkg-config --exists opencv4 2>/dev/null; then
  echo "OpenCV $(pkg-config --modversion opencv4) already present, skipping apt OpenCV"
  OPENCV_PKGS=""
  BUILD_CV_BRIDGE_FROM_SOURCE=1
else
  OPENCV_PKGS="libopencv-dev ros-humble-cv-bridge ros-humble-image-transport"
  BUILD_CV_BRIDGE_FROM_SOURCE=0
fi

apt install -y \
    python3-vcstool \
    libtbb-dev \
    libfmt-dev \
    ros-humble-ament-cmake-clang-format \
    $OPENCV_PKGS

mkdir -p "$BASALT_WS/src"
cd "$BASALT_WS"

vcs import --shallow --workers 1 < "$SCRIPT_DIR/basalt.repos"

# Only the submodules the VIO library needs
BASALT_SRC="$BASALT_WS/src/basalt_wrapper/basalt"
git -C "$BASALT_SRC" submodule update --init --depth 1 \
    thirdparty/basalt-headers thirdparty/magic_enum
git -C "$BASALT_SRC/thirdparty/basalt-headers" submodule update --init --depth 1 \
    thirdparty/eigen thirdparty/Sophus thirdparty/cereal


if [ -f /opt/ros/${ROS_DISTRO}/install/setup.bash ]; then
  source /opt/ros/${ROS_DISTRO}/install/setup.bash
else
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi
colcon build --merge-install --symlink-install \
    --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_DISABLE_FIND_PACKAGE_realsense2=ON \
    -DJUST_INSTALL_CEREAL=ON  # basalt_ros' T265 tool needs a library the trimmed basalt no longer has; cereal's sandbox/docs need Boost
