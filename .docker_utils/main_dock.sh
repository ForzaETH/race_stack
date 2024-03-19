#! /bin/bash

# Script to launch the main docker instance for the pblf110 car
IMAGE=race_stack_sim_x86

docker run --tty \
    --interactive \
    --network=host \
    --env DISPLAY=$DISPLAY \
    --env USER=$USER \
    --env XAUTHORITY=/home/$USER/.Xauthority \
    --env ROS_HOSTNAME=$ROS_HOSTNAME \
    --volume $XAUTH_LOC:/home/$USER/.Xauthority \
    --volume /dev:/dev \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $RACE_STACK_ROOT:/home/$USER/catkin_ws/src/race_stack \
    --volume $RACE_STACK_ROOT/../../cache/noetic/build:/home/$USER/catkin_ws/build \
    --volume $RACE_STACK_ROOT/../../cache/noetic/devel:/home/$USER/catkin_ws/devel \
    --volume $RACE_STACK_ROOT/../../cache/noetic/logs:/home/$USER/catkin_ws/logs \
    --privileged \
    --name forzaeth_devcontainer \
    --entrypoint /bin/bash \
    ${IMAGE}:latest
