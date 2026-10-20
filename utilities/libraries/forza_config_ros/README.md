# forza_config_ros

The ROS 2 layer for the stack's ROS-free configs: the [`forza_config`](../forza_config) schemas (submodule), and the config schemas of any other ROS-free package. It adds what needs ROS:

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

A ROS-free package keeps its own pydantic schemas and path-based loaders, with its yaml values in
`stack_master/config/<car>/`, as `forza_config` does. Its ROS 2 node reads them with `load_model_ros`, which gives the
same strict loading as the car configs (no extra or repeated keys, errors naming the file). Do not import such a
package here if it pulls heavy dependencies: the node passes its schema in.

The path lookup assumes the workspace layout `<ws>/src/race_stack`.
