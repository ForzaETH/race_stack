ARG IMAGE=ros:humble-desktop
FROM ${IMAGE}

ARG USER=race_crew
ARG UID=1000
ARG GID=1000

SHELL [ "/bin/bash", "-c" ]
ARG DEBIAN_FRONTEND=noninteractive

# Create user and give permissions
RUN groupadd -fg ${GID} ${USER} \
    && useradd -ms /bin/bash -u ${UID} -g ${GID} -G sudo ${USER} \
    && apt-get update && apt-get install -y sudo \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER} \
    && usermod -aG dialout,plugdev ${USER}

# Copy linux and python requirement lists
COPY .install_utils/requirements/linux_req.txt /tmp/linux_req.txt
COPY .install_utils/requirements/python_req.txt /tmp/python_req.txt

# Install system dependencies & setup standalone UV
RUN apt-get update && xargs apt-get install -y < /tmp/linux_req.txt \
    && apt-get clean && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir uv

# Install python dependencies through UV (Lightning Fast)
RUN uv pip install --system -r /tmp/python_req.txt

# Place workspace sourcing into terminal user profile
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER}/.bashrc && \
    echo "source /ws/install/setup.bash" >> /home/${USER}/.bashrc

# Delete temporary requirement files
RUN rm /tmp/linux_req.txt /tmp/python_req.txt

# Set runtime environment
USER ${USER}
WORKDIR /ws
CMD ["/bin/bash"]
