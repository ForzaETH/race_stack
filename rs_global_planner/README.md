# rs_global_planner

Computes the global plan of a map (centerline, raceline and shortest path) and serves it to the rest of the stack.

The two jobs are separate executables:

- **Planning is offline:** `global_planner` computes the plan once per map, stores it as `global_waypoints.json` in the map directory and exits. It publishes nothing.
- **Serving is for running the stack:** `global_plan_server` loads the stored plan and publishes it, latched. It is the only publisher of a plan.

## Nodes

| Executable | Launched by | Does |
|---|---|---|
| `global_planner` | `stack_master/launch/planner_launch.py` | map → plan → save `global_waypoints.json` → exit (offline, no topics) |
| `global_plan_server` | `stack_master/launch/base_system_launch.yaml`, and `planner_launch.py` once the planner succeeded | load `global_waypoints.json` → publish, keeps running so the latched topics stay available (fails if the map has no plan yet) |

```bash
ros2 launch stack_master planner_launch.py map:=<map name in $MAPS_DIR>
```

`planner_launch.py` runs the planner, then starts the server for checking the new plan and slicing its sectors (sector slicer). If the planner fails, the launch shuts down instead of serving the previous plan.

## How the plan is computed

1. Load the map PNG and YAML (`resolution`, `origin`, `initial_pose`) from `$MAPS_DIR/<map>`.
2. Skeletonize the free space; its closed loop is the centerline. Smooth it (`centerline_smoothing_window_m`) and resample it to 0.1 m.
3. Order the centerline in the driving direction given by the heading of `initial_pose`.
4. Extract the two track bounds (walls) from the map and the distances of the centerline to them.
5. Optimize the raceline (iterative minimum curvature, `safety_width`) with the
   [TUM global race trajectory optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
   (ForzaETH fork, pinned in `.install_utils/dependencies.txt`).
6. Optimize the shortest path (`safety_width_sp`) along the raceline as reference line.

The vehicle model of the optimizer (`racecar_f110.ini`, `veh_dyn_info/`) is in `stack_master/config/global_planner/`.

## Package layout

| Module | Contents |
|---|---|
| `planner_node.py` | `global_planner` node |
| `server_node.py` | `global_plan_server` node: latched topics of a plan, including the markers and sectors |
| `plan.py` | `GlobalPlan`: the plan and its JSON file |
| `track_geometry.py` | map image → centerline, track bounds, widths (numpy/OpenCV only, no ROS) |
| `parameters.yaml` | parameter definitions (generate_parameter_library) |

## Parameters of `global_planner`

Values are set in `stack_master/config/global_planner/global_planner_params.yaml`, `map_path` by the launch file.

| Parameter | Default | Description |
|---|---|---|
| `map_path` | (required) | map directory; the plan is written here |
| `reverse_mapping` | `false` | plan the track in reverse driving direction |
| `safety_width` | 0.6 m | width kept free around the raceline, incl. the car |
| `safety_width_sp` | 0.6 m | same for the shortest path |
| `centerline_smoothing_window_m` | 2.0 m | Savitzky-Golay window for smoothing the centerline |

`global_plan_server` only takes `map_path`.

## Topics of `global_plan_server` (latched)

| Topic | Type | Contents |
|---|---|---|
| `/global_plan/raceline` | `rs_msgs/WpntArray` | raceline |
| `/global_plan/shortest_path` | `rs_msgs/WpntArray` | shortest path |
| `/global_plan/centerline` | `rs_msgs/WpntArray` | centerline |
| `/global_plan/raceline/markers` | `visualization_msgs/MarkerArray` | raceline, line colored by speed |
| `/global_plan/shortest_path/markers` | `visualization_msgs/MarkerArray` | shortest path, line colored by speed |
| `/global_plan/centerline/markers` | `visualization_msgs/MarkerArray` | centerline line |
| `/global_plan/track_bounds/markers` | `visualization_msgs/MarkerArray` | right and left track bound lines |
| `/global_plan/raceline/track_length` | `std_msgs/Float64` | lap length of the raceline in meters |
| `/global_plan/speed_sectors` | `rs_msgs/SectorArray` | speed sectors from `speed_scaling.yaml` in the map directory; one whole-lap sector if it is missing or doesn't fit the raceline |

All markers are in the `map` frame. In Lichtblick, add them to a 3D panel with display frame `map`, together with `/map`.
