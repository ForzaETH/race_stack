# syntax=docker/dockerfile:1

# Base image is ros:humble-ros-base or 
# nvcr.io/nvidia/l4t-jetpack:r36.4.0 on Jetson
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

# Keep only Ubuntu's 4.5 OpenCV (for Jetson)
if dpkg-query -W libopencv >/dev/null 2>&1; then
    apt-get purge -y libopencv libopencv-dev libopencv-python libopencv-samples nvidia-opencv nvidia-opencv-dev \
        opencv-licenses opencv-samples-data
fi

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
useradd --no-log-init -ms /bin/bash -u ${UID} -g ${GID} -G sudo,dialout,plugdev,video ${USER}
echo "${USER} ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER}
echo 'Defaults env_keep += "DEBIAN_FRONTEND"' >> /etc/sudoers.d/${USER}
chmod 0440 /etc/sudoers.d/${USER}
EOF

# ---------- vio-false / vio-true: selected by USE_VIO ----------
# Placed before the requirements so changing those does not rebuild Basalt.
FROM base AS vio-false

FROM base AS vio-true
ARG DEBIAN_FRONTEND=noninteractive

# Install Realsense and build Basalt
COPY .install_utils/install_vio.sh .install_utils/basalt.repos /tmp/
RUN bash /tmp/install_vio.sh && rm /tmp/install_vio.sh /tmp/basalt.repos

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

# Install PyTorch with CUDA and torchvision, on Jetson only
COPY .install_utils/install_torch_jetson.sh /tmp/
RUN bash /tmp/install_torch_jetson.sh && rm /tmp/install_torch_jetson.sh

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
