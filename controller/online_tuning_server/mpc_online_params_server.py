#!/usr/bin/env python3
import rospy
import rosparam
from typing import Union
from dynamic_reconfigure.server import Server
from controller.cfg import dyn_kmpc_params_tunerConfig, dyn_stmpc_params_tunerConfig
from pbl_config import STMPCConfig, load_STMPC_config_ros, STMPCConfigDyn, KMPCConfig, load_KMPC_config_ros, KMPCConfigDyn

class CallbackHandler:
    def __init__(self, mpc_version: str) -> None:
        self.version = mpc_version
        if mpc_version == "STMPC":
            self.dyn_config_keys = STMPCConfigDyn.model_fields.keys()
        elif mpc_version == "KMPC":
            self.dyn_config_keys = KMPCConfigDyn.model_fields.keys()
        else:
            raise ValueError("Invalid MPC version")


    def callback(self, config, level):
        rospy.loginfo(f"[{self.version} dynamic parameter server] MPC Parameters Updated")
        print(config)

        for k in self.dyn_config_keys:
            # Ensuring rounding to the second decimal
            setattr(config, k, round(getattr(config, k), 2))

        return config

def get_default_dict(mpc_version: str) -> dict:
    # get full config from yaml
    racecar_version = rospy.get_param('/racecar_version') # NUCX or JETx

    if mpc_version == "KMPC":
        config: KMPCConfig = load_KMPC_config_ros(racecar_version)
        dyn_config_keys = KMPCConfigDyn.model_fields.keys()
    elif mpc_version == "STMPC":
        config: STMPCConfig = load_STMPC_config_ros(racecar_version)
        dyn_config_keys = STMPCConfigDyn.model_fields.keys()

    # only iterate in possible keys of the dynamic reconfigure
    default_config = {key: getattr(config, key) for key in dyn_config_keys}

    return default_config


if __name__ == "__main__":
    rospy.init_node("mpc_dyn_rec_tuner_server", anonymous=False)
    mpc_version = rosparam.get_param("/mpc_version")

    cbh = CallbackHandler(mpc_version)
    if mpc_version == "STMPC":
        dyn_mpc_params_tunerConfig = dyn_stmpc_params_tunerConfig
    elif mpc_version == "KMPC":
        dyn_mpc_params_tunerConfig = dyn_kmpc_params_tunerConfig
    server = Server(dyn_mpc_params_tunerConfig, cbh.callback)

    default_dict = get_default_dict(mpc_version)

    server.update_configuration(default_dict)
    rospy.spin()
