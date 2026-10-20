# forza_config_ros

The ROS 2 layer for the stack's ROS-free configs. Those are the [`forza_config`](../forza_config) schemas (submodule), and also the config schemas of any other ROS-free package, e.g. the MPC controllers' own params. It adds what needs ROS:

- the stack's layout: car configs live in the stack_master **source** tree, `stack_master/config/<racecar_version>/`, so identification results land next to the code and can be committed;
- reading parameters of other nodes (`get_remote_parameter`, e.g. `racecar_version` and `floor` from `global_parameters`).

| Function | Reads / writes |
| --- | --- |
| `load_car_config_ros(car)` | `car_model.yaml` |
| `load_pacejka_tire_config_ros(car, floor)` | `pacejka/<floor>/default.yaml`; raises `NotExistingFloor` if the car has a `pacejka/` folder but not that floor |
| `save_pacejka_tire_config_ros(config, car, floor, update_latest=True)` | new `pacejka/<floor>/archive/YYYYMMDD_pacejka.yaml`; `default.yaml` points at it if `update_latest` |
| `load_steering_lut_ros(car, floor)` | `LUT/<floor>/default.csv` |
| `save_steering_lut_ros(table, car, floor, update_latest=True)` | new `LUT/<floor>/archive/YYYYMMDD_lut.csv`; `default.csv` points at it if `update_latest` |
| `load_model_ros(Model, car, *subpath)` | any pydantic schema from `config/<car>/<subpath...>` |
| `get_remote_parameter(node, remote_node, name, default=None)` | a parameter of another node |

Schemas (`CarConfig`, `PacejkaTireConfig`, ...) and errors come from `forza_config`: a schema mismatch raises a pydantic `ValidationError` naming the file.

## Other ROS-free configs

A ROS-free package (like the MPC controllers) defines its own pydantic schemas and path-based loaders, and keeps its
yaml values out of the package, as `forza_config` does. Its ROS 2 node then reads them here, by car:

```python
from forza_config_ros import load_model_ros
params = load_model_ros(STMPCConfig, racecar_version, 'mpc', 'single_track_mpc.yaml')
```

That gives the same strict loading as the car configs (no extra or repeated keys, errors naming the file). If
several nodes share a config, add a named `load_<x>_ros` here. Do not import the ROS-free package here if it
pulls heavy dependencies (e.g. acados): pass its schema in instead.

The path lookup assumes the workspace layout `<ws>/src/race_stack`.
