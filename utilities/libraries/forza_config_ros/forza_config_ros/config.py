"""The race stack's layout on top of forza_config: car configs live in the stack_master
source tree, under ``config/<racecar_version>/``."""
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from forza_config import (CarConfig, PacejkaTireConfig, load_car_config, load_pacejka_tire_config,
                          load_steering_lut, save_pacejka_tire_config, save_steering_lut)


class NotExistingFloor(ValueError):
    """The car has identified tires, but not for this floor."""


def get_stack_master_path(subpath: str = '') -> Path:
    """Resolves a path relative to the stack_master package source tree.

    Goes via the installed share directory to find the workspace root, since
    stack_master's config folder is not fully mirrored into the install space.
    Results written here (sysid) land next to the code and can be committed.
    """
    return Path(get_package_share_directory('stack_master')).parents[3] / 'src/race_stack/stack_master' / subpath


def car_config_dir(racecar_version: str) -> Path:
    return get_stack_master_path(f'config/{racecar_version}')


def load_car_config_ros(racecar_version: str) -> CarConfig:
    """``config/<car>/car_model.yaml``"""
    return load_car_config(car_config_dir(racecar_version) / 'car_model.yaml')


def load_pacejka_tire_config_ros(racecar_version: str, floor: str) -> PacejkaTireConfig:
    """``config/<car>/pacejka/<floor>/default.yaml``

    Raises NotExistingFloor when the car has a ``pacejka/`` folder but not this floor,
    FileNotFoundError when it has none.
    """
    pacejka = car_config_dir(racecar_version) / 'pacejka'
    try:
        return load_pacejka_tire_config(pacejka / floor / 'default.yaml')
    except FileNotFoundError:
        if pacejka.is_dir():
            raise NotExistingFloor(f"{racecar_version} has no tires for floor '{floor}' in {pacejka}; "
                                   "initialize it with a sysid procedure") from None
        raise


def save_pacejka_tire_config_ros(config: PacejkaTireConfig, racecar_version: str, floor: str,
                                 update_latest: bool = True) -> Path:
    """New archive file in ``config/<car>/pacejka/<floor>/``; ``default.yaml`` points at it if ``update_latest``."""
    return save_pacejka_tire_config(config, car_config_dir(racecar_version) / 'pacejka' / floor, update_latest)


def load_steering_lut_ros(racecar_version: str, floor: str) -> np.ndarray:
    """``config/<car>/LUT/<floor>/default.csv``"""
    return load_steering_lut(car_config_dir(racecar_version) / 'LUT' / floor / 'default.csv')


def save_steering_lut_ros(table: np.ndarray, racecar_version: str, floor: str, update_latest: bool = True) -> Path:
    """New archive file in ``config/<car>/LUT/<floor>/``; ``default.csv`` points at it if ``update_latest``."""
    return save_steering_lut(table, car_config_dir(racecar_version) / 'LUT' / floor, update_latest)
