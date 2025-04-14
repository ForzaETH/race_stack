#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from perception.cfg import dyn_tracker_tunerConfig

def callback(config, level):
    # tracking
    config.dist_deletion = round(config.dist_deletion * 20) / 20
    config.dist_infront = round(config.dist_infront * 20) /20
    config.min_std = round(min(config.min_std, config.max_std) * 100) / 100
    config.max_std = round(max(config.min_std + 0.01, config.max_std) * 100) / 100
    config.vs_reset = round(config.vs_reset * 100) / 100
    config.aggro_multi = round(config.aggro_multi *10) / 10
    config.ratio_to_glob_path = round(config.ratio_to_glob_path * 10) / 10

    # detection
    config.max_obs_size = round(config.max_obs_size * 10) / 10
    config.max_viewing_distance = round(config.max_viewing_distance * 20) / 20
    config.boundaries_inflation = round(config.boundaries_inflation *100) / 100
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tracker_server", anonymous=False)
    print('[Opponent Tracking] Dynamic Tracker Server Launched...')
    srv = Server(dyn_tracker_tunerConfig, callback)
    rospy.spin()