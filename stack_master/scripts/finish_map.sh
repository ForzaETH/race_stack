#!/bin/bash

echo "Finish trajectory..."
rosservice call /finish_trajectory 0
if [ "$1" = "" ]
then
	echo "Please enter a map name: "
	read map_name
	echo "Save map under the name $map_name.pbstream"
	rosservice call /write_state "{filename: '${HOME}/catkin_ws/src/icra_f110_stack/stack_master/maps/$map_name.pbstream', include_unfinished_submaps: "true"}"
else
	echo "Save map under the name $1.pbstream"
	rosservice call /write_state "{filename: '${HOME}/catkin_ws/src/icra_f110_stack/stack_master/maps/$1.pbstream', include_unfinished_submaps: "true"}"
fi
