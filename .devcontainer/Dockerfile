# syntax=docker/dockerfile:1
# Base image is ros:humble-ros-base or l4t-jetpack:r36.4.0 on Jetson
ARG BASE_IMAGE=ros:humble-ros-base
ARG USE_VIO=false USER=race_crew UID=1000 GID=1000

FROM ${BASE_IMAGE}
ARG USE_VIO USER UID GID
ARG DEBIAN_FRONTEND=noninteractive
SHELL [ "/bin/bash", "-c" ]
ENV ROS_DISTRO=humble LANG=C.UTF-8 LC_ALL=C.UTF-8

# Install ROS 2 (on Jetson)
COPY .install_utils/install_ros.sh /tmp/
RUN bash /tmp/install_ros.sh && rm /tmp/install_ros.sh

# Create user with matching UID/GID with serial and USB access
RUN groupadd -fg ${GID} ${USER} \
    && useradd --no-log-init -ms /bin/bash -u ${UID} -g ${GID} -G sudo,dialout,plugdev,video ${USER} \
    && echo "${USER} ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} \
    && echo 'Defaults env_keep += "DEBIAN_FRONTEND"' >> /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER}

# Install RealSense and build Basalt, only if USE_VIO=true
COPY .install_utils/install_vio.sh .install_utils/basalt.repos /tmp/
RUN if [[ "$USE_VIO" == "true" ]]; then bash /tmp/install_vio.sh; fi \
    && rm -f /tmp/install_vio.sh /tmp/basalt.repos

# Install system (apt) dependencies
COPY .install_utils/requirements/linux_req.txt /tmp/linux_req.txt
RUN apt-get update && xargs apt-get install -y < /tmp/linux_req.txt \
    && rm -rf /var/lib/apt/lists/* /tmp/linux_req.txt

# Install uv for python
COPY --from=ghcr.io/astral-sh/uv:0.13.6 /uv /uvx /bin/

# Install PyTorch with CUDA and torchvision, on Jetson only
COPY .install_utils/install_torch_jetson.sh /tmp/
RUN bash /tmp/install_torch_jetson.sh && rm /tmp/install_torch_jetson.sh

# Install python dependencies
COPY .install_utils/requirements/python_req.txt .install_utils/requirements/python_overrides.txt /tmp/
RUN uv pip install --system --no-cache -r /tmp/python_req.txt --override /tmp/python_overrides.txt \
    && rm /tmp/python_req.txt /tmp/python_overrides.txt

# Install rosdep dependencies
COPY --parents ./**/package.xml /tmp/rosdep/race_stack/
COPY .install_utils/dependencies.repos /tmp/dependencies.repos
RUN apt-get update && rosdep update && mkdir -p /tmp/rosdep/deps \
    && vcs import --recursive /tmp/rosdep/deps < /tmp/dependencies.repos \
    && rosdep install --from-paths /tmp/rosdep --ignore-src -y --skip-keys "python-transforms3d-pip" \
    && rm -rf /tmp/rosdep /tmp/dependencies.repos /root/.ros /var/lib/apt/lists/*

# Source ROS and workspaces, display and git completion in terminal
COPY .install_utils/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /home/${USER}/.bashrc && rm /tmp/bashrc

# Create mount points for persistent volumes (extensions, home storage, colcon workspace)
RUN mkdir -p /home/${USER}/.vscode-server/extensions /home/${USER}/.persist /ws \
    && chown -R ${UID}:${GID} /home/${USER}/.vscode-server /home/${USER}/.persist /ws

# Set runtime environment
USER ${USER}
WORKDIR /ws
CMD ["/bin/bash"]
