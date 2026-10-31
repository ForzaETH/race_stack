# Base image is ros:humble-ros-base or nvcr.io/nvidia/l4t-jetpack:r36.3.0 on Jetson
ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ARG USER=race_crew
ARG UID=1000
ARG GID=1000

SHELL [ "/bin/bash", "-c" ]
ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Humble if the image doesn't come with it. This is needed for the base image for Nvidia Jetsons.
RUN if [ ! -f /opt/ros/humble/setup.bash ]; then \
      apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates \
      && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
      && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
      && apt-get update && apt-get install -y --no-install-recommends \
           ros-humble-ros-base build-essential git python3-rosdep \
           python3-colcon-common-extensions python3-colcon-mixin \
      && rosdep init && rm -rf /var/lib/apt/lists/*; \
    fi
ENV ROS_DISTRO=humble LANG=C.UTF-8 LC_ALL=C.UTF-8

# Create user with matching UID/GID so bind-mounted files stay owned by
# the host user. Also give access to serial (VESC) and USB devices.
RUN apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd -fg ${GID} ${USER} \
    && useradd --no-log-init -ms /bin/bash -u ${UID} -g ${GID} -G sudo,dialout,plugdev ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} \
    && echo 'Defaults env_keep += "DEBIAN_FRONTEND"' >> /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER}

# Install system (apt) dependencies
COPY .install_utils/requirements/linux_req.txt /tmp/linux_req.txt
RUN apt-get update \
    && xargs apt-get install -y < /tmp/linux_req.txt \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/linux_req.txt

# Install python dependencies
COPY --from=ghcr.io/astral-sh/uv:0.13.6 /uv /uvx /bin/
COPY .install_utils/requirements/python_req.txt .install_utils/requirements/python_overrides.txt /tmp/
RUN uv pip install --system --no-cache -r /tmp/python_req.txt --override /tmp/python_overrides.txt \
    && rm /tmp/python_req.txt /tmp/python_overrides.txt

# Bashrc: source ROS and workspaces, git completion and branch in terminal
RUN echo '' >> /home/${USER}/.bashrc \
    && echo '# --- race_stack ---' >> /home/${USER}/.bashrc \
    && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/${USER}/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /home/${USER}/.bashrc \
    && echo 'export XDG_RUNTIME_DIR=/tmp/runtime-$(id -u) && mkdir -p -m 0700 $XDG_RUNTIME_DIR' >> /home/${USER}/.bashrc \
    && echo 'source /usr/lib/git-core/git-sh-prompt' >> /home/${USER}/.bashrc \
    && echo 'PS1="\[\e[01;32m\]\u@\h\[\e[00m\]:\[\e[01;34m\]\w\[\e[00m\]\$(__git_ps1 \" (%s)\")\$ "' >> /home/${USER}/.bashrc

# Create mount points for persistent volumes (extensions, home storage, colcon workspace)
RUN mkdir -p /home/${USER}/.vscode-server/extensions /home/${USER}/.persist \
        /ws/build /ws/install /ws/log \
    && chown -R ${UID}:${GID} /home/${USER}/.vscode-server /home/${USER}/.persist /ws

# Set runtime environment
USER ${USER}
WORKDIR /ws
CMD ["/bin/bash"]
