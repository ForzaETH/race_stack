ARG IMAGE=ros:humble
FROM ${IMAGE}
ARG PLATFORM=arm
ARG USER
ARG UID
ARG GID

RUN if [[ "$(uname -m)" == "aarch64" ]] && [[ "$PLATFORM" == "jet" ]]; then \
    echo "Building for Jetson ARM architecture..."; \
    if [[ "$IMAGE" != "dustynv/ros:humble-ros-base-l4t-r36.3.0" ]]; then \
    echo "Error: The selected base image is wrong."; \
    exit 1; \
    fi; \
    elif [[ "$(uname -m)" == "arm64" ]] && [[ "$PLATFORM" == "arm" ]]; then \
    echo "Building for Apple ARM architecture..."; \
    if [[ "$IMAGE" != "ros:humble-desktop" ]]; then \
    echo "Error: The selected base image is wrong."; \
    exit 1; \
    fi; \
    elif [[ "$(uname -m)" == "x86_64" ]] && [[ "$PLATFORM" == "x86" ]]; then \
    echo "Building for x86 architecture..."; \
    if [[ "$IMAGE" != "ros:humble-desktop" ]]; then \
    echo "Error: The selected base image is wrong."; \
    exit 1; \
    fi; \
    fi;



SHELL [ "/bin/bash", "-c" ]
ARG DEBIAN_FRONTEND=noninteractive

ADD ./.install_utils/basalt_ros_setup.sh /.install_utils/basalt_ros_setup.sh
RUN chmod +x /.install_utils/basalt_ros_setup.sh && /.install_utils/basalt_ros_setup.sh

ADD ./.install_utils/realsense_setup.sh /.install_utils/realsense_setup.sh
RUN chmod +x /.install_utils/realsense_setup.sh && /.install_utils/realsense_setup.sh


ADD ./.install_utils/linux_req/linux_req.txt /.install_utils/linux_req.txt
RUN apt-get update && xargs apt-get install -y < /.install_utils/linux_req.txt
ADD ./.install_utils/python_req.txt /tmp/.install_utils/python_req.txt
RUN pip3 install -r /tmp/.install_utils/python_req.txt &&\
    pip3 install quadprog~=0.1.11

ADD ./.install_utils /.install_utils
RUN useradd -ms /bin/bash -u ${UID} -G sudo ${USER} && groupadd -fg ${GID} ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USER

ADD .install_utils/bashrc/ /tmp/.install_utils/
RUN echo "export USERNAME=$USER" >> /home/$USER/.bashrc &&\
    cat /tmp/.install_utils/.bashrc_git >> /home/$USER/.bashrc &&\
    cat /tmp/.install_utils/.bashrc_forzaeth >> /home/$USER/.bashrc

USER root
# Clean up
RUN rm -rf /tmp/.install_utils

USER $USER
WORKDIR /home/$USER/ws
CMD ["/bin/bash"]