import yaml
import os
from .dotdict import DotDict

def save(model, overwrite_existing=True, verbose=False):
  script_dir = os.path.dirname(os.path.realpath(__file__))
  folder_path = os.path.join(script_dir, "../models", model['model_name'])
  if not os.path.exists(folder_path):
    os.makedirs(folder_path)
  file_path = os.path.join(folder_path, model['model_name'] +"_"+ model['tire_model'] + ".txt")
  if os.path.isfile(file_path):
    if (verbose): print("Model already exists")
    if overwrite_existing:
      if (verbose): print("Overwriting...")
    else:
      if (verbose): print("Not overwriting.")
      return 0

  # will only work if its dotdict, otherwise no need to convert to dict
  try:
    model = model.to_dict()
  except:
    model = model

  with open(file_path, "w") as f:
      yaml.dump(model, f, default_flow_style=False)
