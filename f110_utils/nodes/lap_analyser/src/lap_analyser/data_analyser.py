#!/usr/bin/env python3

import rospy
from lap_analyser import LapAnalyser
from vel_analyser import VelAnalyser

if __name__ == "__main__":
    """
    ROS node that modularly handles the data anlysis
    """
    
    rospy.init_node("data_analyser")
    
    # add new data analysers here
    lap_analyser = LapAnalyser()
    vel_analyser = VelAnalyser()
    # TODO actually remove this weird thing and just do separate launch files that can be joined together
    
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        vel_analyser.loop()
        rate.sleep()