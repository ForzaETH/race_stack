#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from spliner.cfg import dyn_spliner_tunerConfig

def callback(config, level):
    # Ensuring nice rounding by either 0.05 or 0.5
    config.evasion_dist = round(config.evasion_dist * 20) / 20
    config.obs_traj_tresh = round(config.obs_traj_tresh * 20) / 20
    config.spline_bound_mindist = round(config.spline_bound_mindist * 20) / 20

    config.pre_apex_dist0 = round(config.pre_apex_dist0 * 2) / 2
    # Ensuring that the pre_apex_dist1 is always greater than pre_apex_dist0
    config.pre_apex_dist1 = round(min(config.pre_apex_dist0 + 0.5, config.pre_apex_dist1)* 2) / 2
    config.pre_apex_dist2 = round(min(config.pre_apex_dist1 + 0.5, config.pre_apex_dist2)* 2) / 2
    config.post_apex_dist0 = round(config.post_apex_dist0 * 2) / 2
    config.post_apex_dist1 = round(max(config.post_apex_dist0 + 0.5, config.post_apex_dist1)* 2) / 2
    config.post_apex_dist2 = round(max(config.post_apex_dist1 + 0.5, config.post_apex_dist2)* 2) / 2
    config.kd_obs_pred = round(config.kd_obs_pred * 20) / 20
    config.fixed_pred_time = round(config.fixed_pred_time * 100) / 100
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_spline_tuner_node", anonymous=False)
    print('[Planner] Dynamic Spline Server Launched...')
    srv = Server(dyn_spliner_tunerConfig, callback)
    rospy.spin()

