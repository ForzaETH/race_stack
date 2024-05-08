#! /bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR && pip3 install -e ../planner/global_planner/global_planner/global_racetrajectory_optimization/