#!/bin/bash

# parse first argument to check which nuc has been selected
# check that first three characters are exactly "NUC"
if [ "${1:0:3}" != "NUC" ]; then
    echo "Please provide the nuc name as the first argument, e.g. NUC5, NUC6, etc."
    echo "ROS_DOMAIN_ID will be set to 48."
    export ROS_DOMAIN_ID=48 # the default
# if the last character is not a number, and the input is not 4 characters long then it is invalid
elif [ "${1:3:1}" == "" ] || [ "${1:3:1}" == "0" ] || [ "${1:4:1}" != "" ]; then
    echo "Invalid nuc name. Please provide the nuc name as the first argument, e.g. NUC5, NUC6, etc."
    echo "ROS_DOMAIN_ID will be set to 48."
    export ROS_DOMAIN_ID=48 # the default
else
    # make the ros_domain_id 40+ nuc number
    export ROS_DOMAIN_ID=$((40 + ${1:3:1}))
    echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"
fi

export ROS_LOCALHOST_ONLY=0
