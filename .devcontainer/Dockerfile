FROM osrf/ros:jazzy-desktop-full
ARG USERNAME
ARG UID
ARG GID
ARG SIM=1

ENV SHELL /bin/bash


# Create the user
RUN userdel ubuntu
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid $UID --gid $GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install packages
RUN apt-get update && apt-get upgrade -y

# install folders are added granularly to avoid unnecessary rebuilds when developing the Dockerfile
ADD .install_utils/linux_req/ /tmp/.install_utils/ 
RUN xargs apt-get install -y < /tmp/.install_utils/linux_req.txt
# if sim is set to 1, install sim dependencies, else car dep
RUN if [ "$SIM" = "1" ]; \
    then xargs apt-get install -y < /tmp/.install_utils/linux_req_sim.txt;\
    else xargs apt-get install -y < /tmp/.install_utils/linux_req_car.txt;\
    fi
# separate installation is needed for quadprog because of trajectory-planning-helpers
RUN apt-get remove python3-kiwisolver -y
ADD .install_utils/python_req.txt /tmp/.install_utils/python_req.txt
RUN pip3 install -r /tmp/.install_utils/python_req.txt --break-system-packages &&\
    pip3 install quadprog~=0.1.11 --break-system-packages

SHELL ["/bin/bash", "-c"] 
# install nav2 packages from source
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    mkdir -p /opt/nav2/src && cd /opt/nav2 &&\
    git clone https://github.com/ros-navigation/navigation2.git ./src/navigation2 &&\
    rosdep install -r -y \
    --from-paths ./src \
    --ignore-src \
    --skip-keys=turtlebot3_gazebo &&\
    colcon build \
    --symlink-install --packages-up-to nav2_map_server nav2_lifecycle_manager &&\
    rm -r /opt/nav2/src
# flag desktop-full package as manually installed (otherwise apt suggests to uninstall it)
RUN apt-get install -y ros-jazzy-desktop-full


# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

ADD .install_utils/bashrc/ /tmp/.install_utils/
RUN echo "export USERNAME=$USERNAME" >> /home/$USERNAME/.bashrc &&\
    cat /tmp/.install_utils/.bashrc_git >> /home/$USERNAME/.bashrc &&\
    cat /tmp/.install_utils/.bashrc_forzaeth >> /home/$USERNAME/.bashrc

USER root
# Clean up
RUN rm -rf /tmp/.install_utils
USER $USERNAME

CMD ["/bin/bash"]
