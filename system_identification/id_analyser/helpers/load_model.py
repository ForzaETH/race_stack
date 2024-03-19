import yaml
import os
from .dotdict import DotDict

def get_dict(model_name):
    model, tire = model_name.split("_")
    script_dir = os.path.dirname(os.path.realpath(__file__))
    model_file = os.path.join(script_dir, "../models", model, model_name + ".txt")
    if not os.path.isfile(model_file):
        raise ValueError(f"Model {model_name} does not exist")
    with open(model_file, 'rb') as f:
        params = yaml.load(f, Loader=yaml.Loader)
    
    return params

def get_dotdict(model_name):
    dict = get_dict(model_name)
    params = DotDict(dict)
    return params
