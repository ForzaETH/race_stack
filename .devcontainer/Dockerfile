# syntax=docker/dockerfile:1

# Base image is ros:humble-ros-base or 
# nvcr.io/nvidia/l4t-jetpack:r36.3.0 on Jetson
ARG BASE_IMAGE=ros:humble-ros-base
ARG USE_VIO=false
ARG USER=race_crew
ARG UID=1000
ARG GID=1000

# ---------- base: ROS + user ----------
FROM ${BASE_IMAGE} AS base
ARG USER UID GID
ARG DEBIAN_FRONTEND=noninteractive
SHELL [ "/bin/bash", "-c" ]
ENV ROS_DISTRO=humble LANG=C.UTF-8 LC_ALL=C.UTF-8

RUN <<'EOF'
set -eo pipefail
apt-get update
apt-get install -y --no-install-recommends curl ca-certificates gnupg lsb-release sudo

# Install ROS 2 Humble if the image doesn't come with it. This is needed for the base image for Nvidia Jetsons.
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
EOF

# Create user with matching UID/GID so bind-mounted files stay owned by
# the host user. Also give access to serial (VESC) and USB devices.
RUN <<'EOF'
set -e
groupadd -fg ${GID} ${USER}
useradd --no-log-init -ms /bin/bash -u ${UID} -g ${GID} -G sudo,dialout,plugdev ${USER}
echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER}
echo 'Defaults env_keep += "DEBIAN_FRONTEND"' >> /etc/sudoers.d/${USER}
chmod 0440 /etc/sudoers.d/${USER}
EOF

# ---------- vio-false / vio-true: selected by USE_VIO ----------
# Placed before the requirements so changing those does not rebuild Basalt.
FROM base AS vio-false

FROM base AS vio-true
ARG DEBIAN_FRONTEND=noninteractive

# Set up Realsense: librealsense + realsense2_camera ROS wrapper
RUN <<'EOF'
set -eo pipefail
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor > /etc/apt/keyrings/librealsenseai.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/librealsense.list
apt-get update
apt-get install -y librealsense2-utils librealsense2-dev "ros-humble-realsense2-*"
rm -rf /var/lib/apt/lists/*
EOF

# Set up Basalt: build basalt_ros into its own workspace /opt/basalt_ws (symlink install: keep build/),
# separate from /ws as its Eigen clashes with the race_stack one. Sourced in the bashrc after /ws.
COPY .install_utils/basalt.repos /tmp/basalt.repos
RUN <<'EOF'
set -e
# Skip apt OpenCV if one is pre-installed (e.g. CUDA OpenCV on Jetson images)
pkg-config --exists opencv4 || OPENCV_PKGS="libopencv-dev ros-humble-cv-bridge ros-humble-image-transport"
apt-get update
apt-get install -y python3-vcstool libtbb-dev libfmt-dev ros-humble-ament-cmake-clang-format $OPENCV_PKGS
rm -rf /var/lib/apt/lists/*

mkdir -p /opt/basalt_ws/src && cd /opt/basalt_ws
vcs import --shallow --workers 1 < /tmp/basalt.repos
BASALT_SRC=src/basalt_wrapper/basalt
git -C $BASALT_SRC submodule update --init --depth 1 thirdparty/basalt-headers thirdparty/magic_enum
# basalt-headers pins Eigen on gitlab.com, which often rejects requests; use the GitHub mirror
git -c url."https://github.com/eigen-mirror/eigen.git".insteadOf="https://gitlab.com/libeigen/eigen.git" \
    -C $BASALT_SRC/thirdparty/basalt-headers submodule update --init --depth 1 thirdparty/eigen thirdparty/Sophus thirdparty/cereal

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --merge-install --symlink-install --cmake-args \
    -DCMAKE_CXX_FLAGS="-w" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_DISABLE_FIND_PACKAGE_realsense2=ON -DJUST_INSTALL_CEREAL=ON
rm -rf /tmp/basalt.repos log
EOF

# ---------- final: requirements + shell ----------
FROM vio-${USE_VIO} AS final
ARG USER UID GID
ARG DEBIAN_FRONTEND=noninteractive

# Install system (apt) dependencies
COPY .install_utils/requirements/linux_req.txt /tmp/linux_req.txt
RUN <<'EOF'
set -e
apt-get update
xargs apt-get install -y < /tmp/linux_req.txt
rm -rf /var/lib/apt/lists/* /tmp/linux_req.txt
EOF

# Install python dependencies
COPY --from=ghcr.io/astral-sh/uv:0.13.6 /uv /uvx /bin/
COPY .install_utils/requirements/python_req.txt .install_utils/requirements/python_overrides.txt /tmp/
RUN uv pip install --system --no-cache -r /tmp/python_req.txt --override /tmp/python_overrides.txt \
    && rm /tmp/python_req.txt /tmp/python_overrides.txt

# Bashrc: source ROS and workspaces, display, git branch in terminal
COPY .install_utils/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /home/${USER}/.bashrc && rm /tmp/bashrc

# Create mount points for persistent volumes (extensions, home storage, colcon workspace)
RUN mkdir -p /home/${USER}/.vscode-server/extensions /home/${USER}/.persist /ws/build /ws/install /ws/log \
    && chown -R ${UID}:${GID} /home/${USER}/.vscode-server /home/${USER}/.persist /ws

# Set runtime environment
USER ${USER}
WORKDIR /ws
CMD ["/bin/bash"]
