#!/usr/bin/env python3
import rospy
import rospkg
import yaml
from dynamic_reconfigure.server import Server
from controller.cfg import dyn_l1_params_tunerConfig

def get_racecar_version():
    racecar_version = rospy.get_param('/racecar_version') # NUCX or JETx
    return racecar_version

def get_yaml_values():
    # Get corresponding yaml file path
    yaml_path = rospkg.RosPack().get_path("stack_master")
    # Get and return data
    with open(yaml_path + "/config/" + get_racecar_version() + "/l1_params.yaml", "r") as file:
        data = yaml.safe_load(file)
    return data

def callback(config, level):
    rospy.loginfo("L1 Parameter Updated")
    # Ensuring nice rounding by 0.05
    config.t_clip_min = round(config.t_clip_min * 200) / 200 # round to 0.005
    config.t_clip_max = round(config.t_clip_max * 200) / 200
    config.m_l1 = round(config.m_l1 * 200) / 200
    config.q_l1 = round(config.q_l1 * 200) / 200
    config.speed_lookahead = round(config.speed_lookahead * 20) / 20
    config.lat_err_coeff = round(config.lat_err_coeff * 20) / 20
    config.acc_scaler_for_steer = round(config.acc_scaler_for_steer * 20) / 20
    config.dec_scaler_for_steer = round(config.dec_scaler_for_steer * 20) / 20
    config.start_scale_speed = round(config.start_scale_speed * 20) / 20
    config.end_scale_speed = round(config.end_scale_speed * 20) / 20
    config.downscale_factor = round(config.downscale_factor * 20) / 20
    config.speed_lookahead_for_steer = round(config.speed_lookahead_for_steer * 20) / 20
    #Trailing Controller
    config.prioritize_dyn = config.prioritize_dyn
    config.trailing_gap = round(config.trailing_gap * 20) / 20
    config.trailing_p_gain = round(config.trailing_p_gain * 100) / 100
    config.trailing_i_gain = round(config.trailing_i_gain * 100) / 100
    config.trailing_d_gain = round(config.trailing_d_gain * 100) / 100
    config.blind_trailing_speed = round(config.blind_trailing_speed * 10) / 10

    return config

if __name__ == "__main__":
    rospy.init_node("dyn_l1_params_tuner_server", anonymous=False)
    server = Server(dyn_l1_params_tunerConfig, callback)

    yaml_data = get_yaml_values()
 
    # Set l1 parameter values to the values of the corresponding .yaml file
    # float() is need as we want to ensure its type to access it in L1_controller with "params.doubles[idx].value"
    default_config = {
        ## L1 Controller Parameters
        "t_clip_min": float(yaml_data["t_clip_min"]),
        "t_clip_max": float(yaml_data["t_clip_max"]),
        "m_l1": float(yaml_data["m_l1"]),
        "q_l1": float(yaml_data["q_l1"]),
        "speed_lookahead": float(yaml_data["speed_lookahead"]),
        "lat_err_coeff": float(yaml_data["lat_err_coeff"]),
        "acc_scaler_for_steer": float(yaml_data["acc_scaler_for_steer"]),
        "dec_scaler_for_steer": float(yaml_data["dec_scaler_for_steer"]),
        "start_scale_speed": float(yaml_data["start_scale_speed"]),
        "end_scale_speed": float(yaml_data["end_scale_speed"]),
        "downscale_factor": float(yaml_data["downscale_factor"]),
        "speed_lookahead_for_steer": float(yaml_data["speed_lookahead_for_steer"]),
        ## Trailing Controller Parameters
        "prioritize_dyn": bool(yaml_data["prioritize_dyn"]),
        "trailing_gap": float(yaml_data["trailing_gap"]),
        "trailing_p_gain": float(yaml_data["trailing_p_gain"]),
        "trailing_i_gain": float(yaml_data["trailing_i_gain"]),
        "trailing_d_gain": float(yaml_data["trailing_d_gain"]),
        "blind_trailing_speed": float(yaml_data["blind_trailing_speed"])
    }

    server.update_configuration(default_config)
    rospy.spin()


