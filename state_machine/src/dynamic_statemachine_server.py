#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from state_machine.cfg import dyn_statemachine_tunerConfig

def callback(config, level):
    # Ensuring nice rounding by either 0.05
    config.lateral_width_gb_m = round(config.lateral_width_gb_m * 20) / 20
    config.lateral_width_ot_m = round(config.lateral_width_ot_m * 20) / 20

    config.splini_hyst_timer_sec = round(config.splini_hyst_timer_sec * 20) / 20
    config.splini_ttl = round(config.splini_ttl * 20) / 20
    config.emergency_break_horizon = round(config.emergency_break_horizon, 2)

    config.ftg_speed_mps = round(config.ftg_speed_mps * 20) / 20
    config.ftg_timer_sec = round(config.ftg_timer_sec * 20) / 20

    if not config.ftg_active:
       rospy.logdebug_throttle_identical(30, "FTG IS NOT ACTIVE")

    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_statemachine_tuner_node", anonymous=False)
    print('[dynamic_statemachine_tuner_node] State Machine Parameter Server Launched')
    srv = Server(dyn_statemachine_tunerConfig, callback)
    rospy.spin()

