# Global Optimizer 

## Description
The Global Optimizer package is a ROS package designed to generate a global trajectory around a race track. It provides a node that subscribes to map and car pose information, processes this data, and publishes global waypoints, track boundaries and map informations and saves everything in a JSON file. This package is implemented in Python and uses the `rospy` library for ROS interactions. For the calculation of the global trajectory, it uses the [TUM Global Race Trajectory Optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) package, which was modified so that it was possible to integrate it in our race stack. The main node `global_planner_node.py` is launched with the `mapping.launch` file from the `stack_master` package or if the `map_editor` is used. There is also a node `global_trajectory_publisher.py` for publishing the global waypoints and markers and track bounds markers frequently after they have been calculated which is launched every time the base system is launched.

## Parameters
 - `rate`: Sets the rate in hertz at which the ROS node is running.
 - `test_on_car`: Defines if the global optimizer is running on the car `True` or in simulation `False`.
 - `safety_width`: Sets the width of the car in meters including a safety margin for the calculation of the global trajectory.
 - `safety_width_sp`: Sets the width of the car in meters including a safety margin for the calculation of the shortest path trajectory.
 - `occupancy_grid_threshold`: Sets the threshold above which a cell in the occupancy grid is considered occupied and below it is considered free.
 - `show_plots`: Show additional plots for debugging during generation of the global trajectory.
 - `map_editor`: `True` if the map editor is used.
 - `map_editor_mapping`: `True` if the map editor is used live on the car.
 - `map_name`: Sets the name of the map.
 - `map_dir`: Sets the directory of the map.
 - `create_map`: Create a map that can be used in the simulator and save a YAML file and a PNG of the map in the directory `map_dir`.
 - `reverse_mapping`: Generate a global trajectory in the reverse direction.
 - `required_laps`: Number of laps required before the generation of the global trajectory can be started.

## Input/Output Topic Signature
This node subscribes to:
- `/map`: Subscribes to the occupancy grid of the map.
- `/car_state/pose`: Reads the car's pose
    
The node publishes to:
- `/global_waypoints`: Publishes global waypoints.
- `/global_waypoints/markers`: Publishes global waypoint markers.
- `/global_waypoints/shortest_path`: Publishes waypoints of the shortest path trajectory (only used for visualization of the overtaking sectors).
- `/global_waypoints/shortest_path/markers`: Publishes waypoint markers of the shortest path trajectory (only used for visualization of the overtaking sectors).
- `/centerline_waypoints`: Publishes centerline waypoints.
- `/centerline_waypoints/markers`: Publishes centerline waypoint markers.
- `/trackbounds/markers`: Publishes markers for the track boundaries.
- `/map_infos`: Publishes map infos like estimated lap time and maximum speed.
- `/map_ready`: Publishes a boolean if the map is ready so that the global trajectory can be calculated.
- `/estimated_lap_time`: Publishes the estimated lap time.

## License
TODO

