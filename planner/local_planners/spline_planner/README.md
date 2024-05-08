# Spliner 

## Description
The Spliner package is a ROS package designed to perform splining around obstacles. It provides a node that subscribes to obstacle and waypoint information, processes this data, and publishes splined waypoints and spline markers. This package is implemented in Python and uses the `rospy` and `dynamic_reconfigure` libraries for ROS interactions and dynamic parameter tuning. The main node `spliner_node.py` is launched with the `headtohead.launch` file from the `stack_master` package.

## Parameters
 - `evasion_dist`: Sets the orthogonal distance of the apex to the obstacle, determining how far the spline will deviate from the obstacle.
 - `obs_traj_tresh`: Sets the threshold of the obstacle towards the raceline to be considered for evasion, determining which obstacles will be considered for splining around.
 - `spline_bound_mindist`: Sets the minimum distance that splines may be from the track bounds in meters, ensuring that the spline does not get too close to the track boundaries.
 - `pre_apex_0`: Sets the first distance in front of the apex in meters, used in the calculation of the spline.
 - `pre_apex_1`: Sets the second distance in front of the apex in meters, used in the calculation of the spline.
 - `pre_apex_2`: Sets the third distance in front of the apex in meters, used in the calculation of the spline.
 - `post_apex_0`: Sets the first distance behind the apex in meters, used in the calculation of the spline.
 - `post_apex_1`: Sets the second distance behind the apex in meters, used in the calculation of the spline.
 - `post_apex_2`: Sets the third distance behind the apex in meters, used in the calculation of the spline.
 - `kd_obs_pred`: Sets the gain for the obstacle prediction, used in the calculation of the obstacle prediction.
 - `fixed_pred_time`: Sets the fixed prediction time, used in the calculation of the obstacle prediction.

The parameters `kd_obs_pred`, `fixed_pred_time` are only used with some obstacle prediction methods, so they might not always affect the behaviour. 

## Input/Output Topic Signature
This node subscribes to:
- `/perception/obstacles`: Subscribes to the obstacle array.
- `/car_state/frenet/odom`: Reads the car's state
- `/global_waypoints`: Subscribes to global waypoints.
- `/global_waypoints_scaled`: Subscribes to the scaled global waypoints to obtain the current target velocities.
    
The node publishes to:
- `/planner/avoidance/markers`: Publishes spline markers.
- `/planner/avoidance/otwpnts`: Publishes splined waypoints.
- `/planner/avoidance/considered_OBS`: Publishes markers for the closest obstacle.
- `/planner/avoidance/propagated_obs`: Publishes markers for the propagated obstacle.
- `/planner/avoidance/latency`: Publishes the latency of the spliner node. (only if measuring is enabled)

## License
TODO