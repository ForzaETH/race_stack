import os
from pathlib import Path
import yaml
from pydantic import BaseModel, ConfigDict, ValidationError

from pbl_config.archive import write_archived
from pbl_config.paths import get_stack_master_path


class PacejkaTireConfig(BaseModel):
    """Pacejka Tire Parameters configuration class"""
    model_config = ConfigDict(extra='forbid', use_attribute_docstrings=True)

    friction_coeff: float
    """Friction coefficient of the tire"""
    Bf: float
    """Longitudinal stiffness coefficient (B) of the FRONT tire"""
    Cf: float
    """Cornering stiffness coefficient (C) of the FRONT tire"""
    Df: float
    """Peak value of the lateral force (D) of the FRONT tire"""
    Ef: float
    """Lateral stiffness coefficient (E) of the FRONT tire"""

    Br: float
    """Longitudinal stiffness coefficient (B) of the REAR tire"""
    Cr: float
    """Cornering stiffness coefficient (C) of the REAR tire"""
    Dr: float
    """Peak value of the lateral force (D) of the REAR tire"""
    Er: float
    """Lateral stiffness coefficient (E) of the REAR tire"""

    floor: str
    """Floor indicating location of the track"""

def load_pacejka_tire_config_ros(racecar_version: str, floor: str) -> PacejkaTireConfig:
    """Loads the pacejka_tire config from the yaml file

    Args:
        racecar_version (str): a car name
    """

    relative_path = 'config/' + racecar_version + '/pacejka/' + floor + '/default.yaml'
    config_path = get_stack_master_path(relative_path)
    try:
        with open(config_path, 'r', encoding='utf-8') as file:
            cfg_dict = yaml.safe_load(file)
            try:
                config = PacejkaTireConfig(**cfg_dict)
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
                raise
    except FileNotFoundError as e:
        # check if pacejka folder exists
        pacejka_path = get_stack_master_path('config/' + racecar_version + '/pacejka/')
        if os.path.exists(pacejka_path):
            print(f"Selected floor {floor} does not exist. Please initialize it with some sysid procedure.")
            raise NotExistingFloor()
        else:
            raise e

    return config


def save_pacejka_tire_config(
        config: PacejkaTireConfig,
        racecar_version: str,
        floor: str,
        update_latest: bool = True) -> Path:
    """Saves the pacejka_tire config to `config/<car>/pacejka/<floor>/archive/` and,
    if `update_latest`, points default.yaml at it.

    Args:
        config (PacejkaTireConfig): a PacejkaTireConfig
    """
    def write(path):
        with open(path, 'w', encoding='utf-8') as file:
            yaml.dump(config.model_dump(), file)

    return write_archived(get_stack_master_path('config/' + racecar_version + '/pacejka/' + floor),
                          'pacejka.yaml', write, update_latest)

class NotExistingFloor(ValueError):
    pass