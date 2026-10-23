# rs_global_planner

Computes the global plan of a map (centerline, raceline and shortest path) and publishes it for the rest of the stack.

The plan is computed once per map and stored as `global_waypoints.json` in the map directory. Every run of the base system then only loads and publishes it.

## Nodes

| Executable | Launched by | Does |
|---|---|---|
| `global_planner` | `stack_master/launch/planner_launch.yaml` | map → plan → save `global_waypoints.json` → publish |
| `global_plan_publisher` | `stack_master/launch/base_system_launch.yaml` | load `global_waypoints.json` → publish (fails if the map has no plan yet) |

Both keep running after publishing: the topics are latched, so late subscribers still get the plan.

```bash
ros2 launch stack_master planner_launch.yaml map:=<map name in $MAPS_DIR>
```

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
| `publisher_node.py` | `global_plan_publisher` node |
| `plan.py` | `GlobalPlan`: the plan and its JSON file |
| `plan_publisher.py` | latched topics of a plan, including the markers |
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

`global_plan_publisher` only takes `map_path`.

## Published topics (latched)

| Topic | Type | Contents |
|---|---|---|
| `/global_waypoints` | `rs_msgs/WpntArray` | raceline |
| `/global_waypoints/shortest_path` | `rs_msgs/WpntArray` | shortest path |
| `/centerline_waypoints` | `rs_msgs/WpntArray` | centerline |
| `/global_waypoints/markers` | `visualization_msgs/MarkerArray` | raceline, line colored by speed |
| `/global_waypoints/shortest_path/markers` | `visualization_msgs/MarkerArray` | shortest path, line colored by speed |
| `/centerline_waypoints/markers` | `visualization_msgs/MarkerArray` | centerline line |
| `/trackbounds/markers` | `visualization_msgs/MarkerArray` | right and left track bound lines |

All markers are in the `map` frame. In Lichtblick, add them to a 3D panel with display frame `map`, together with `/map`.
