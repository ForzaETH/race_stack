import yaml
import rospkg
from pydantic import BaseModel, ConfigDict, ValidationError

class MultiOppSplinerConfig(BaseModel):
    """Car Parameters configuration class"""
    model_config = ConfigDict(extra='forbid', use_attribute_docstrings=True)

    evasion_dist: float
    """Orthogonal distance of the apex to the obstacle"""
    obs_traj_tresh: float
    """Threshold of the obstacle towards raceline to be considered for evasion"""
    lookahead_dist: float
    """Spline length and lookahead distance for obstacles to be considered in meters"""
    spline_bound_mindist: float
    """Splines may never be closer to the track bounds than this param in meters"""
    spline_resolution: float
    """Resolution of the spline in meters"""
    evasion_slowdown: float
    """Slowdown velocity for the evasion when use_sqp is disabled in meters per second"""
    pre_apex_dist0: float
    """First distance infront of the apex for smooth boundary encapsulation in meters"""
    pre_apex_dist1: float
    """Second distance infront of the apex for smooth boundary encapsulation in meters"""
    pre_apex_dist2: float
    """Third distance infront of the apex for smooth boundary encapsulation in meters"""
    post_apex_dist0: float
    """First distance after of the apex for smooth boundary encapsulation in meters"""
    post_apex_dist1: float
    """Second distance after of the apex for smooth boundary encapsulation in meters"""
    post_apex_dist2: float
    """Third distance after of the apex for smooth boundary encapsulation in meters"""
    parallel_segment: float
    """Length of the parallel segment for smooth boundary encapsulation in meters"""
    
    pre_apex_minimal: float
    """Distance infront of the apex for minimal boundary encapsulation in meters"""
    post_apex_minimal: float
    """Distance after the apex for minimal boundary encapsulation in meters"""
    parallel_segment_minimal: float
    """Length of the parallel segment for minimal boundary encapsulation in meters"""
    
    kd_obs_pred: float
    """Controls how fast the opponent is assumed to go back
    to our raceline"""
    fixed_pred_time: float
    """Fixed prediciton time for the opponent"""
    debug: bool
    """Enable debug visualization"""
    use_sqp: bool
    """Use sqp for the spline calculation"""

def load_multi_opp_spline_config_ros() -> MultiOppSplinerConfig:
    """Loads the car config from the yaml file

    Args:
        racecar_version (str): a car name
    """

    relative_path = '/config/Planning/multi_opp_spliner_config.yaml'
    config_path = rospkg.RosPack().get_path('stack_master') + relative_path
    with open(config_path, 'r') as file:
        cfg_dict = yaml.safe_load(file)
        try:
            config = MultiOppSplinerConfig(**cfg_dict)
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
