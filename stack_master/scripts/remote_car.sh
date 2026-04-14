#!/bin/bash

declare -A machines=(
    [NUC1]="192.168.192.1"
    [NUC2]="192.168.192.4"
)

set_local() {
    export ROS_DOMAIN_ID=42
    unset ZENOH_CONFIG_OVERRIDE
    export ROS_LOCALHOST_ONLY=1
}


# parse first argument to check which nuc has been selected
# check that first three characters are exactly "NUC"
if [ "${1:0:3}" != "NUC" ]; then
    echo "Please provide the nuc name as the first argument, e.g. NUC5, NUC6, etc."
    echo "ROS_DOMAIN_ID will be set to 42."
    set_local
# if the last character is not a number, and the input is not 4 characters long then it is invalid
elif [ "${1:3:1}" == "" ] || [ "${1:3:1}" == "0" ] || [ "${1:4:1}" != "" ] || [ -z "${machines[$1]}" ]; then
    echo "Invalid nuc name. Please provide the nuc name as the first argument, e.g. NUC5, NUC6, etc."
    echo "ROS_DOMAIN_ID will be set to 42."
    set_local
    unset ZENOH_CONFIG_OVERRIDE
else
    export ROS_DOMAIN_ID=42 #CHANGE ROS_DOMAIN_ID
    echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"
    IP=${machines[$1]}
    export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/'$IP':7447"]'
    export ROS_LOCALHOST_ONLY=0
fi

