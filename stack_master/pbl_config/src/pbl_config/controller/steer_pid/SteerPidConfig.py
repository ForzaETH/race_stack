
import rospkg
import yaml
from pydantic import BaseModel, ConfigDict, ValidationError


class SteerPidConfig(BaseModel):
    """Steer Pid Parameters configuration class"""
    model_config = ConfigDict(extra='forbid', use_attribute_docstrings=True)

    K_p: float
    """Kp of the steering PID controller"""
    K_i: float
    """Ki of the steering PID controller"""
    K_d: float
    """Kd of the steering PID controller"""


def load_steer_pid_config(racecar_version: str) -> SteerPidConfig:
    """Loads the pacejka_tire config from the yaml file

    Args:
        racecar_version (str): a car name
    """

    relative_path = '/config/' + racecar_version + '/steer_pid.yaml'
    config_path = rospkg.RosPack().get_path('stack_master') + relative_path
    with open(config_path, 'r', encoding='utf-8') as file:
        cfg_dict = yaml.safe_load(file)
        try:
            config = SteerPidConfig(**cfg_dict)
            return config
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