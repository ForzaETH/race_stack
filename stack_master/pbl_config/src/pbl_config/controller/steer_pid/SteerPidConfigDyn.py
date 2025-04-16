
from typing import Tuple

import rospkg
import yaml
from dynamic_reconfigure.parameter_generator_catkin import ParameterGenerator
from pbl_config import SteerPidConfig, load_steer_pid_config
from pydantic import BaseModel, ConfigDict, ValidationError


class SteerPidConfigDyn(BaseModel):
    """Steer PID dynamic reconfigure configuration class"""
    model_config = ConfigDict(extra='forbid', use_attribute_docstrings=True)

    K_p: Tuple[float, float]
    """Kp of the steering PID controller"""
    K_i: Tuple[float, float]
    """Ki of the steering PID controller"""
    K_d: Tuple[float, float]
    """Kd of the steering PID controller"""


def create_steer_pid_dynamic_parameters(gen: ParameterGenerator):
    """Create dynamic reconfigure parameters for the kinematic MPC controller

    Args:
        gen (ParameterGenerator): The dynamic reconfigure parameter generator
    """
    default_car = "DEFAULT"
    # load default parameters from the yaml file
    SteerPidCfg: SteerPidConfig = load_steer_pid_config(default_car)
    # load the dynamic reconfigure parameters
    relative_path = f'/config/{default_car}/steer_pid_dyn_params.yaml'
    config_path = rospkg.RosPack().get_path('stack_master') + relative_path
    with open(config_path, 'r') as file:
        cfg_dict = yaml.safe_load(file)
    try:
        config = SteerPidConfigDyn(**cfg_dict)
    except ValidationError as e:
        for error in e.errors():
            if error["type"] == "missing":
                for missing_key in error["loc"]:
                    raise ValueError(f"Missing key <{missing_key}> in {config_path} file. Please add it.")
            elif error["type"] == "extra_forbidden":
                for extra_key in error["loc"]:
                    raise ValueError(f"Extra key <{extra_key}> in {config_path} file. Please remove it.")
            else:
                raise ValueError(f"Error loading the {config_path} file. Please contact support (edo) with this traceback.")
        return None

    # check dyn parameters are available in the normal config
    for key in config.model_dump().keys():
        if key not in SteerPidCfg.model_dump().keys():
            raise ValueError(f"Key <{key}> is not available in the normal config. Please only try to dynamically reconfigure only the available parameters.")

    # create the dynamic reconfigure parameters
    for key, value in config.model_dump().items():
        gen.add(name=key,
                paramtype='double',
                level=0,
                description=f"{config.model_fields[key].description}",
                default=SteerPidCfg.model_dump()[key],
                min=value[0],
                max=value[1],
        )

    return gen
