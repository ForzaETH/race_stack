#!/bin/bash
set -e

if [ -z "$1" ]; then
    TARGET="workspace"
else
    TARGET="packages --packages-up-to $1"
fi

colcon clean \
    $TARGET --yes \
    --build-base ~/ws/build --install-base ~/ws/install --log-base ~/ws/log \
    --clean-ignore .gitkeep f110_gym
