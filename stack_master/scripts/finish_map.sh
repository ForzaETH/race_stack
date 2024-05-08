#!/bin/bash

echo "Finish trajectory..."
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

echo "Save map under the name $1"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$1', include_unfinished_submaps: "true"}"