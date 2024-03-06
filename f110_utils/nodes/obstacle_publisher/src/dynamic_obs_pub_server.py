#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from obstacle_publisher.cfg import dyn_obs_publisherConfig

def callback(config, level):
    config.speed_scaler = config.speed_scaler
    config.ampl_sin1 = config.ampl_sin1
    config.ampl_sin2 = config.ampl_sin2
    config.phase_sin1 = config.phase_sin1
    config.phase_sin2 = config.phase_sin2
    print("Hey there")
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_obstacle_publisher_node", anonymous=False)
    print('[Obs. Publisher] Dynamic Obstacle Publisher Server Launched...')
    srv = Server(dyn_obs_publisherConfig, callback)
    rospy.spin()