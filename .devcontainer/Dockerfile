ARG IMAGE=ros:humble
FROM ${IMAGE}
ARG PLATFORM=arm
ARG USER
ARG UID
ARG GID
ARG USE_VIO=false

SHELL [ "/bin/bash", "-c" ]
ARG DEBIAN_FRONTEND=noninteractive

# VIO (RealSense D455 + Basalt) is optional, controlled by USE_VIO in .env
ADD ./.install_utils/basalt_ros_setup.sh /.install_utils/basalt_ros_setup.sh
RUN if [[ "$USE_VIO" == "true" ]]; then chmod +x /.install_utils/basalt_ros_setup.sh && /.install_utils/basalt_ros_setup.sh; fi

ADD ./.install_utils/realsense_setup.sh /.install_utils/realsense_setup.sh
RUN if [[ "$USE_VIO" == "true" ]]; then chmod +x /.install_utils/realsense_setup.sh && /.install_utils/realsense_setup.sh; fi


ADD ./.install_utils/linux_req/linux_req.txt /.install_utils/linux_req.txt
RUN if [[ "$PLATFORM" == "jet" ]]; then \
      mkdir -p /tmp/ocvdummy/DEBIAN && \
      printf '%s\n' \
        'Package: libopencv-dev' \
        'Version: 4.8.1' \
        'Architecture: all' \
        'Maintainer: none <none@localhost>' \
        'Provides: libopencv-core-dev, libopencv-imgproc-dev, libopencv-imgcodecs-dev, libopencv-videoio-dev, libopencv-highgui-dev, libopencv-calib3d-dev, libopencv-features2d-dev, libopencv-flann-dev, libopencv-ml-dev, libopencv-objdetect-dev, libopencv-photo-dev, libopencv-video-dev, libopencv-dnn-dev, libopencv-contrib-dev, libopencv-stitching-dev, libopencv-shape-dev, libopencv-superres-dev, libopencv-videostab-dev, libopencv-viz-dev, opencv-data' \
        'Description: Dummy, OpenCV 4.8.1 is provided by the base image' \
        > /tmp/ocvdummy/DEBIAN/control && \
      dpkg-deb --build /tmp/ocvdummy /tmp/libopencv-dev_4.8.1_all.deb && \
      dpkg -i /tmp/libopencv-dev_4.8.1_all.deb && \
      rm -rf /tmp/ocvdummy /tmp/libopencv-dev_4.8.1_all.deb; \
    fi

RUN apt-get update && xargs apt-get install -y < /.install_utils/linux_req.txt
ADD ./.install_utils/python_req.txt /tmp/.install_utils/python_req.txt
RUN pip3 install --index-url https://pypi.org/simple -r /tmp/.install_utils/python_req.txt || true

RUN apt-get update && apt-get install -y --no-install-recommends build-essential python3-dev && \
    pip3 install --index-url https://pypi.org/simple "setuptools==58.2.0" wheel "cython<3" && \
    SETUPTOOLS_USE_DISTUTILS=stdlib \
    pip3 install --index-url https://pypi.org/simple --no-build-isolation "quadprog~=0.1.11"

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