import logging
import os
from datetime import datetime

import rospkg
import yaml
from pydantic import BaseModel, ConfigDict, ValidationError


class LinearTireConfig(BaseModel):
    """Pacejka Tire Parameters configuration class"""
    model_config = ConfigDict(extra='forbid', use_attribute_docstrings=True)

    C_Sf: str
    """Cornering Stiffness of the front tire"""
    C_Sr: str
    """Cornering Stiffness of the rear tire"""
    floor: str
    """Floor indicating location of the track"""


def load_linear_tire_config_ros(racecar_version: str, floor: str) -> LinearTireConfig:
    """Loads the pacejka_tire config from the yaml file

    Args:
        racecar_version (str): a car name
    """

    relative_path = '/config/' + racecar_version + '/linear/' + floor + '/default.yaml'
    config_path = rospkg.RosPack().get_path('stack_master') + relative_path
    try:
        with open(config_path, 'r', encoding='utf-8') as file:
            cfg_dict = yaml.safe_load(file)
            try:
                config = LinearTireConfig(**cfg_dict)
            except ValidationError as e:
                for error in e.errors():
                    if error["type"] == "missing":
                        for missing_key in error["loc"]:
                            print(f"Missing key <{missing_key}> in {relative_path} file. Please add it.")
                    elif error["type"] == "extra_forbidden":
                        for extra_key in error["loc"]:
                            print(f"Extra key <{extra_key}> in {relative_path} file. Please remove it.")
                    else:
                        print(f"Error loading the {relative_path} file. Please contact support (edo) with this traceback.")
                        raise e
                return None
    except FileNotFoundError as e:
        # check if pacejka folder exists
        linear_path = rospkg.RosPack().get_path('stack_master') + '/config/' + racecar_version + '/linear/' + floor
        if os.path.exists(linear_path):
            print(f"Selected floor {floor} does not exist. Please initialize it with some sysid procedure.")
            raise NotExistingFloor()
        else:
            raise e

    return config

class NotExistingFloor(ValueError):
    pass