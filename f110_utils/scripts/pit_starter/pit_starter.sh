#!/bin/bash

if [ $# -gt 0 ]
then

#export the ROS IP PARAMS to connect to nuc as master
export ROS_HOSTNAME=$1
export ROS_IP=$1
echo 'Using your input as ROS_HOSTNAME and ROS_IP: '
echo $1

	if [ $2 == 'NUC2' ]
	then 
		echo 'Exporting ROS params to access NUC2'
		export ROS_MASTER_URI=http://<TODO:SET_YOUR_CAR_FIXED_IP_HERE>:11311
		echo 'Exported your stuff...'
	# elif [ $2 == 'NUCX' ]
	# then 
	# 	echo 'Exporting ROS params to access NUCX'
	# 	export ROS_MASTER_URI=http://<another fixed IP>:11311
	# 	echo 'Exported your stuff...'
	else
		echo 'No valid car arg. Valid car args are NUC2 - NUC5.'
	fi
	
	if [ -z "$3" ]
	then
		echo "No additional argument supplied."
	elif [ $3 == 'rviz' ]
	then
		echo 'Starting rviz as well'
		#start configured RVIZ
		rosrun rviz rviz -d ~/catkin_ws/src/race_stack/f110_utils/scripts/pit_starter/f1.rviz
	elif [ $3 == 'rqt' ]
	then
		echo 'Starting rqt as well'
		rqt
	elif [ $3 == 'rvizqt' ]
	then
		echo 'Starting rviz and rqt as well'
		rosrun rviz rviz -d ~/catkin_ws/src/race_stack/f110_utils/scripts/pit_starter/f1.rviz &
		rqt
	else
		echo "Not a valid rviz arg. Exiting.\
		
		Valid arguments:
		[None]	| Simply export ROS params to access relevant NUC.
		rviz	| Launch RViz only.
		rqt		| Launch rqt only.
		rvizqt	| Simultaneously launch RViz and rqt."
	fi
fi
