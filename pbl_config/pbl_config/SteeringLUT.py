from pathlib import Path

import numpy as np

from pbl_config.archive import write_archived
from pbl_config.paths import get_stack_master_path


def load_steering_lut(racecar_version: str, floor: str) -> np.ndarray:
    """Loads `config/<car>/LUT/<floor>/default.csv`.

    Row 0 holds the velocities, column 0 the steering angles, cells the steady-state
    lateral acceleration (NaN where there is none).
    """
    return np.loadtxt(get_stack_master_path(f'config/{racecar_version}/LUT/{floor}/default.csv'), delimiter=',')


def save_steering_lut(table: np.ndarray, racecar_version: str, floor: str, update_latest: bool = True) -> Path:
    """Saves a steering LUT to `config/<car>/LUT/<floor>/archive/` and, if `update_latest`, points default.csv at it."""
    return write_archived(get_stack_master_path(f'config/{racecar_version}/LUT/{floor}'), 'lut.csv',
                          lambda path: np.savetxt(path, table, delimiter=','), update_latest)
