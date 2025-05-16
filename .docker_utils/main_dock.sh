#! /bin/bash

# Script to launch the main docker instance for the pblf110 car
IMAGE=nuc_forzaeth_racestack_ros2 # for x86
# IMAGE=jet_forzaeth_racestack_ros2 # for ARM
FORZETH_DIR=~/iASL-MAGPie-2025

docker run --tty \
  --interactive \
  --network=host \
  --env DISPLAY=$DISPLAY \
  --env USER=$USER \
  --env XAUTHORITY=/home/$USER/.Xauthority \
  --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  --env ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY \
  --env ROS_VERSION=$ROS_VERSION \
  --env ROS_PYTHON_VERSION=$ROS_PYTHON_VERSION \
  --env ROS_DISTRO=$ROS_DISTRO \
  --volume $XAUTH_LOC:/home/$USER/.Xauthority \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $FORZETH_DIR/../cache/humble/build:/home/$USER/ws/build \
  --volume $FORZETH_DIR/../cache/humble/install:/home/$USER/ws/install \
  --volume $FORZETH_DIR/../cache/humble/log:/home/$USER/ws/log \
  --volume $FORZETH_DIR:/home/$USER/ws/src/race_stack \
  --privileged \
  --name forzaeth_racestack_ros2_humble \
  --entrypoint /bin/bash \
  ${IMAGE}:humble
