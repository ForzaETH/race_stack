# Controller
![Controller Architecture](./misc/controller_arch.png)

This controller package implements the following controllers, with more details in the respective READMEs:
- [MAP controller](./map/README.md)
- [Pure Pursuit controller](./pp/README.md)
- [Follow the Gap controller](./ftg/README.md)

The `control_node` implemented in `controller_manager.py` initializes the needed controllers. It runs at a specified loop rate where in each cycle the next control inputs are calculated via the choosen controller.

## Input/Output Topic Signature
This nodes subscribes to:
- `/perception/obstacles`: Subscribes to the obstacle array.
- `/car_state/odom`: Reads the car's state
- `/car_state/pose`: Reads the car's state
- `/car_state/odom_frenet`: Reads the car's state
- `/local_waypoints`: Subscribes to local waypoints.
- `/vesc/sensors/imu/raw`: Reads the IMU measurements.
- `/scan`: Reads the LiDAR scans.
- `/state_machine`: Listens to the state of the state machine.
- `/l1_param_tuner/parameter_updates`: Listens to the L1 parameters.


The node publishes to:
- `/vesc/high_level/ackermann_cmd_mux/input/nav_1`: Publishes the control commands.
- `/lookahead_point`: Publishes lookahead point marker.
- `/trailing_opponent_marker`: Published trailing opponend marker.
- `/my_waypoints`: Publishes marker array of the received waypoints.
- `/l1_distance`: Publishes the L1 distance.
- `/trailing/gap_data`: Publishes the PID data for trailing.
- `/controller/latency`: Publishes the latency of the controller.

