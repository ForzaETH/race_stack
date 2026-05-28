#! /bin/bash

# Script to create a Xauthority file for the docker container - copied from xauth_setup.sh to avoid issues
XAUTH=$HOME/.Xauthority
export XAUTH_LOC=$XAUTH
xhost +local:$USER

touch $XAUTH

# Script to launch the main docker instance for the pblf110 car
IMAGE=nuc_forzaeth_racestack_ros2 # for x86
# IMAGE=jet_forzaeth_racestack_ros2 # for ARM
FORZETH_DIR=/home/ilr/ROS2_ForzaETH_Race_Stack/race_stack

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
    --group-add $(getent group input | cut -d: -f3) \
    --group-add $(getent group dialout | cut -d: -f3) \
    --name forzaeth_racestack_ros2_humble \
    --entrypoint /bin/bash \
    ${IMAGE}:humble
