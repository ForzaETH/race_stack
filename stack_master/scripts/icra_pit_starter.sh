#!/bin/bash

if [ $# -gt 0 ]
then

#export the ROS IP PARAMS to connect to nuc as master
export ROS_HOSTNAME=$1
export ROS_IP=$1
echo 'Using your input as ROS_HOSTNAME and ROS_IP: '
echo $1

	if [ $2 == 'NUC3' ]
	then 
		echo 'Exporting ROS params to access NUC3'
		export ROS_MASTER_URI=http://192.168.192.135:11311
		echo 'Exported your stuff...'
	elif [ $2 == 'NUC4' ]
	then 
		echo 'Exporting ROS params to access NUC4'
		export ROS_MASTER_URI=http://192.168.192.106:11311
		echo 'Exported your stuff...'
	else
		echo 'No valid car arg'
	fi
	
	if [ $3 == 'rviz' ]
	then
		echo 'Starting rviz as well'
		#start configured RVIZ
		rosrun rviz rviz -d ~/catkin_ws/src/icra_f110_stack/stack_master/config/rviz/f1.rviz
	else
		echo 'Not a valid rviz arg...'
	fi
fi
