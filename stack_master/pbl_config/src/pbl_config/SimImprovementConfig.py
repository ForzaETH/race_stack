import os

import rospkg
import yaml
from pydantic import BaseModel, ConfigDict, ValidationError


class SimImprovementConfig(BaseModel):
    """Parameters for the Sim improvement configuration class"""
    model_config = ConfigDict(extra='forbid', use_attribute_docstrings=True)

    dataset_name: str
    """Name of the dataset"""
    racecar_version: str
    """Cornering Stiffness of the front tire"""
    floor: str
    """Floor indicating location of the track"""
    drivetrainMLP_mode: str
    """Mode of the drivetrain MLP[Acceleration, Velocity]"""
    bag_type: str
    """Type of the data, possible values are [f1tenth, gokart]"""
    dt: float
    """Time step of the simulation"""


def load_sim_improvement_config() -> SimImprovementConfig:
    """Loads the sim improvement config from the yaml file
    """

    relative_path = '/config/SimImprovement/sim_improvement_params.yaml'
    config_path = rospkg.RosPack().get_path('stack_master') + relative_path

    with open(config_path, 'r', encoding='utf-8') as file:
        cfg_dict = yaml.safe_load(file)
        try:
            config = SimImprovementConfig(**cfg_dict)
            if config.drivetrainMLP_mode not in ["acceleration", "velocity"]:
                print(f"Invalid drivetrainMLP_mode <{config.drivetrainMLP_mode}> in {relative_path} file. Please correct it.")
                raise WrongDrivetrainMode()
            if config.bag_type not in ["f1tenth", "gokart"]:
                print(f"Invalid bag_type <{config.bag_type}> in {relative_path} file. Please correct it.")
                raise WrongDrivetrainMode()
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

    return config

class WrongDrivetrainMode(ValueError):
    pass

class WrongDrivetrainMode(ValueError):
    pass