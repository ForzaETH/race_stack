# F110_Utils

Utils repo for PBL F110. Messages, services and library functions are located inside `/libs`, rosnodes are located inside `/nodes`, and scripts are located inside `/scripts`.

## Current structure
  - `libs`
    - [`f110_msgs`](./libs/f110_msgs/README.md)
    - [`frenet_conversion`](./libs/frenet_conversion/README.md)
  - `nodes`
    - [`bayesopt4ros`](./nodes/bayesopt4ros/README.md): external submodules for bayesian optimization in ROS.
    - [`car_to_car_sync`](./nodes/car_to_car_sync/README.md): contains the node and instructions to stream one car's state estimate to the other.
    - [`frenet_conversion_server`](./nodes/frenet_conversion_server/README.md)
    - [`frenet_odom_republisher`](./nodes/frenet_odom_republisher/README.md): contains the node and instructions to publish the position and velocity of the racecar in frenet coordinates.
    - [`gb_traj_publisher`](./nodes/gb_traj_publisher/README.md): contains the node and instructions to publish the trajectory of the car. 
    - [`lap_analyser`](./nodes/lap_analyser/README.md): contains the node and instructions to analyse the lap times and the performance of the car.
    - [`map_editor`](./nodes/map_editor/README.md): contains the node and instructions to map when an map defined by physical boundaries is not availble.
    - [`obstacle_publisher`](./nodes/obstacle_publisher/README.md): contains the node and instructions to publish fictitious obstacles in the simulator.
    - [`overtaking_sector_tuner`](./nodes/overtaking_sector_tuner/README.md): describes the overtaking sector slicer and server, used to define the overtaking sectors and the corresponding scaling factors.
    - [`param_optimizer`](./nodes/param_optimizer/README.md): contains the node and instructions to use bayesian optimization (with the `bayesopt4ros` package) to get optimal MAP parameters and/or sector scalers.
    - [`random_obstacle_publisher`](./nodes/random_obstacle_publisher/README.md)
    - [`sector_tuner`](./nodes/sector_tuner/README.md): contains the node and instructions to slice the racing line in sectors and then publishes the sectors and the corresponding scaled trajectories and scaling parameters once the system is running.
    - [`set_pose`](./nodes/set_pose/README.md): contains the node and instructions to set the pose of the car in the simulator.
    - [`slam_tuner`](./nodes/slam_tuner/README.md): contains a node and instructions to tune the localization algorithms.
    - [`tf_transformer`](./nodes/tf_transformer/README.md)
    - [`transform_broadcaster`](./nodes/transform_broadcaster/README.md)
  - `scripts`
    - [`pit_starter`](./scripts/pit_starter/README.md): contains the script and instructions to source the NUCX and runs the rviz with the correct config.

---
[Go back to the main README](../README.md)
