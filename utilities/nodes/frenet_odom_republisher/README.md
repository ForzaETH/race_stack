# Frenet Odom Republisher

Publishes the car state in the Frenet frame of the global trajectory.

Pose is read from TF (`map -> car_state/base_link`), velocity from `/state_estimation/odom`. Both simulation and the physical car provide these, so the node is launched unconditionally by `stack_master/launch/base_system_launch.xml`.

```
ros2 launch frenet_odom_republisher frenet_odom_republisher_launch.xml use_sim_time:=<True/False>
```

## Input/Output topic signature
The node subscribes to:
- `/global_waypoints`: Global trajectory, used to build the Frenet converter.
- `/state_estimation/odom`: Car velocity (twist, `base_link` frame). Each message triggers one publish.
- TF `map -> car_state/base_link`: Car pose.

The node publishes to:
- `/car_state/frenet/odom`: `nav_msgs/Odometry` with `pose.position.x = s`, `pose.position.y = d`, `twist.linear.x = vs`, `twist.linear.y = vd`, `child_frame_id` = index of the closest global waypoint.
