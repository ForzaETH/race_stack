#!/bin/bash
set -e

BASES="--build-base $HOME/ws/build --install-base $HOME/ws/install --log-base $HOME/ws/log"

if [ -z "$1" ]; then
    # file-filter mode: keeps .gitkeep at the base roots and the f110_gym build, leaves empty dirs behind
    colcon clean workspace --yes $BASES --clean-ignore .gitkeep f110_gym
else
    # no filter: whole build/install dirs of the selected packages are removed
    colcon clean packages --yes $BASES --packages-select "$@" --packages-skip f110_gym
fi
