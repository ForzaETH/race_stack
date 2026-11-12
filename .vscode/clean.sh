#!/bin/bash
set -e

BASES="--base-paths $HOME/ws --build-base $HOME/ws/build --install-base $HOME/ws/install --log-base $HOME/ws/log"

if [ -z "$1" ]; then
    # all packages except f110_gym (installed separately via pip, its colcon marker must survive)
    colcon clean packages --yes $BASES --packages-skip f110_gym
    rm -rf "$HOME/ws/log"
else
    # whole build/install dirs of the selected packages are removed
    colcon clean packages --yes $BASES --packages-select "$@" --packages-skip f110_gym
fi
