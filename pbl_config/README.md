# PBL Config

Pydantic-validated loaders and savers for the car-specific parameters in `stack_master/config`. Every node that needs car, tire or steering look-up data should go through this package instead of opening the files itself.

## Layout

```
stack_master/config/<racecar_version>/
  car_model.yaml                    CarConfig, hand-maintained
  pacejka/<floor>/default.yaml      PacejkaTireConfig, symlink into archive/
  pacejka/<floor>/archive/          YYYYMMDD_pacejka.yaml
  LUT/<floor>/default.csv           steering LUT, symlink into archive/
  LUT/<floor>/archive/              YYYYMMDD_lut.csv
```

## API

| Function | Reads / writes |
| --- | --- |
| `load_car_config_ros(car)` | `car_model.yaml` |
| `load_pacejka_tire_config_ros(car, floor)` | `pacejka/<floor>/default.yaml`; raises `NotExistingFloor` if the car has a `pacejka/` folder but not that floor |
| `save_pacejka_tire_config(config, car, floor, update_latest=True)` | new archive file; `default.yaml` points at it if `update_latest` |
| `load_steering_lut(car, floor)` | `LUT/<floor>/default.csv` as a numpy array (row 0: velocities, column 0: steering angles, cells: lateral acceleration) |
| `save_steering_lut(table, car, floor, update_latest=True)` | new archive file; `default.csv` points at it if `update_latest` |
| `get_remote_parameter(node, remote_node, name, default=None)` | a parameter of another ROS 2 node, e.g. `global_parameters` |

Loaders print which keys are missing or extra and then raise the `ValidationError`.

The paths resolve to the stack_master **source** tree on purpose (`paths.get_stack_master_path`): identification results land next to the code and can be committed. This assumes the workspace layout `<ws>/src/race_stack`.
