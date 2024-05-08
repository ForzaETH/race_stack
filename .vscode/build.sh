#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ $1 = "Debug" ]; then
    BUILD_TYPE=$1
elif [ $1 = "Release" ]; then
    BUILD_TYPE=RelWithDebInfo
else
    echo "$1 not recognised. Choose 'Debug' or 'Release'."
    exit
fi

PACKAGES=""
if [ -z "$2" ]; then
    SYMLINK="--symlink-install"
elif [ $2 = "no-symlink" ]; then
    SYMLINK=""
    if [ -n "$3" ]; then
        PACKAGES="--packages-up-to $3"
    fi
else
    SYMLINK="--symlink-install"
    if [ -n "$2" ]; then
        PACKAGES="--packages-up-to $2"
    fi
fi

colcon build \
    $PACKAGES \
    --base-paths ~/ws \
    --packages-skip f110_gym \
    $SYMLINK \
    --continue-on-error \
    --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
