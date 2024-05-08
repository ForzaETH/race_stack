#! /bin/bash

# Script to create a Xauthority file for the docker container
XAUTH=$HOME/.Xauthority
export XAUTH_LOC=$XAUTH
xhost +local:$USER

touch $XAUTH
