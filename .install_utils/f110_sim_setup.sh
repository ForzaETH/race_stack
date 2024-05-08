#! /bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WS_DIR=$(readlink -f "$SCRIPT_DIR/../../..")   # should end up in the workspace dir

# ubuntu packages dependencies
# xargs sudo apt-get install -y < "$SCRIPT_DIR/linux_req_sim.txt"

# python dependencies
# pip3 install -r "$SCRIPT_DIR/requirements.txt"

# setup f1tenth_gym
cd $SCRIPT_DIR && pip3 install -e ../base_system/f110_simulator/f1tenth_gym/
