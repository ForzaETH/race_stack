#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from ackermann_cmd_mux.cfg import dyn_tract_tunerConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_traction_tuner_node", anonymous=False)
    print('Dynamic Traction Server Launched...')
    srv = Server(dyn_tract_tunerConfig, callback)
    rospy.spin()

