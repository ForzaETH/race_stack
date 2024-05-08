#! /bin/bash

# Script to launch the main docker instance for the pblf110 car
IMAGE=nuc_forzaeth_racestack_ros2
FORZETH_DIR=/home/forzapblnuc/ros2_devcontainer/src/race_stack

docker run --tty \
    --interactive \
    --network=host \
    --env DISPLAY=$DISPLAY \
    --env USER=$USER \
    --env XAUTHORITY=/home/$USER/.Xauthority \
    --volume $XAUTH_LOC:/home/$USER/.Xauthority \
    --volume /dev:/dev \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $FORZETH_DIR/../cache/humble/build:/home/$USER/ws/build \
    --volume $FORZETH_DIR/../cache/humble/install:/home/$USER/ws/install \
    --volume $FORZETH_DIR/../cache/humble/log:/home/$USER/ws/log \
    --volume $FORZETH_DIR:/home/$USER/ws/src/race_stack \
    --privileged \
    --name nuc_forzaeth_racestack_ros2 \
    --entrypoint /bin/bash \
    ${IMAGE}:latest
