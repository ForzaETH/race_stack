import os
import logging
from datetime import datetime
import yaml
import rospkg
from pydantic import BaseModel, ConfigDict, ValidationError


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

    relative_path = '/config/' + racecar_version + '/pacejka/' + floor + '/default.yaml'
    config_path = rospkg.RosPack().get_path('stack_master') + relative_path
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
                return None
    except FileNotFoundError as e:
        # check if pacejka folder exists
        pacejka_path = rospkg.RosPack().get_path('stack_master') + '/config/' + racecar_version + '/pacejka/'
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
        update_latest: bool = True) -> None:
    """Saves the pacejka_tire config to the yaml file

    Args:
        config (PacejkaTireConfig): a PacejkaTireConfig
    """
    path = rospkg.RosPack().get_path('stack_master') + '/config/'
    model_path = path + racecar_version + '/pacejka/' + floor + '/'
    timestamp = datetime.now().strftime("%m%d")
    archive_folder = model_path + 'archive/'
    if not os.path.exists(archive_folder):
        os.makedirs(archive_folder)
    filename_only = f'{timestamp}_pacejka.yaml'
    filename = archive_folder + filename_only

    # print warning if the file already exists
    if os.path.exists(filename):
        logging.warning("%s already exists and will be overwritten", filename)

    with open(filename, 'w', encoding='utf-8') as file:
        yaml.dump(config.model_dump(), file)
    logging.info("Saved pacejka_tire config of %s on floor %s to %s", racecar_version, floor, filename)

    if update_latest:
        # update  default.yaml to be symlink to the latest model
        latest_path = model_path + 'default.yaml'
        if os.path.exists(latest_path):
            os.remove(latest_path)
        symlink_path = f"./archive/{filename_only}"
        os.symlink(symlink_path, latest_path)
        logging.info("Updated latest pacejka_tire config symlink of %s on floor %s to %s", racecar_version, floor, filename)

class NotExistingFloor(ValueError):
    pass