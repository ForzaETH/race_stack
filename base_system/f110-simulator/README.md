## If you need a F1TENTH simulation in ROS, official F1TENTH has moved to a containerized ROS 2 simulation here: [https://github.com/f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)

# F1TENTH Racecar Simulator

This is a lightweight 2D simulator of the F1TENTH Racecar.
It can be built with ROS, or it can be used as a standalone C++ library.

https://f1tenth.readthedocs.io/en/latest/going_forward/simulator/index.html

## Quick Start

To run the simulator on its own, run:

    roslaunch f1tenth_simulator simulator.launch

This will launch everything you need for a full simulation; roscore, the simulator, a preselected map and a model of the car.

### RVIZ Visualization

With the simulator running, open rviz.
In the left panel at the bottom click the "Add" button, then in the "By topic" tab add the ```/map``` topic and the ```/scan``` topic.
Then in the "By display type" tab add the RobotModel type.
In the left panel under the newly added LaserScan section, change the size to 0.1 meters for a clearer visualization of the lidar (shown in rainbow).

You can place the car manually by clicking the "2D Pose Estimate button" on the top of the screen and dragging your mouse on the desired pose.

There is also a preset config file for rviz, you can find it in `<race stack folder>/f110_utils/scripts/pit_starter/f1.rviz`. In RViz it can be opened with the graphical interface, or by passing it as a command-line argument.

### ROS API

The simulator was set up with two main objectives in mind- similitude to the real car and fast prototyping of racing algorithms. The *simulator* node was written such that it can be swapped out with the F1/10 car itself, and if all topic names remain the same, the same exact code can be run to drive the car. The rest of the ROS nodes are organized so that new planning algorithms can be added quickly and toggled between during driving.

![Simplified graph of ROS nodes](https://github.com/f1tenth/f1tenth_simulator/blob/master/media/sim_graph_public.png)

Each planner can listen to the sensor data published by the *simulator* and then publish [AckermannDrive](http://docs.ros.org/noetic/api/ackermann_msgs/html/msg/AckermannDrive.html) messages to the `/vesc/high_level/ackermann_cmd_mux/input/nav_1`.
We at PBL cut through most of the mux's duties, and just feed the command directly. The relevant parameters of the ```AckermannDriveStamped``` are ```msg.drive.speed, msg.drive.steering_angle and msg.drive.acceleration```. Here either an Acceleration or a Speed can be commanded. Where the acceleration has a higher priority than speed. If the acceleration is a nonzero value, the velocity command is ignored.

Therefore, the suggested way to operate is to build controllers and manage them with a `state_machine` node, then feeding control inputs directly to `/vesc/high_level/ackermann_cmd_mux/input/nav_1`.
TODO: 
Ideally we could publish messages to `.../nav_2`,  `.../nav_3`, etc. and they should be considered in order of number, with `nav_1` being highest priority and `nav_5` being lowest. This behaviour was however never tested.

To instantly move the car to a new state publish [Pose](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/Pose.html) messages to the ```/pose``` topic. This can be useful for scripting the car through a series of automated tests.

The simulated lidar is published to the ```/scan``` topic as [LaserScan](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/LaserScan.html) messages.

The pose of the car is broadcast as a transformation between the ```map``` frame and the ```base_link``` frame. ```base_link``` is the center of the rear axis. The ```laser``` frame defines the frame from which the lidar scan is taken and another transform is broadcast between it and ```base_link```.

### Adding a planning node

There are several steps that necessary to adding a new planning node. 
Ideally one would go through the common *perception->planning->control* steps, building nodes in the appropriate ros packages (`perception`, `planner`, `controller`).
Then a state_machine node can be built, in the `state_machine` package, which sends commands to the `/vesc/high_level/ackermann_cmd_mux/input/nav_1` topic.


Particularly useful can then be also the `f110_utils` folder, which contains all the utilities packages, and the the `stack_master` package, which is usually used to handle the launch of the whole system in a centralised way.

### Parameters

The parameters listed below can be modified in the ```params.yaml``` file.

#### Topics

```drive_topic```: The topic to listen to for autonomous driving.

```joy_topic```: The topic to listen to for joystick commands.

```map_topic```: The topic to listen to for maps to use for the simulated scan.

```pose_topic```: The topic to listen to for instantly setting the position of the car.

```pose_rviz_topic```: The topic to listen to for instantly setting the position of the car with Rviz's "2D Pose Estimate" tool.

```scan_topic```: The topic to publish the simulated scan to.

```distance_transform_topic```: The topic to publish a distance transform to for visualization (see the implementation section below).


#### Frames

```base_link```: The frame of the car, specifically the center of the rear axle.

```scan_frame```: The frame of the lidar.

```map_frame```: The frame of the map.

#### Simulator Parameters

```update_pose_rate```: The rate at which the simulator will publish the pose of the car and simulated scan, measured in seconds. Since the dynamics of the system are evaluated analytically, this won't effect the dynamics of the system, however it will effect how often the pose of the car reflects a change in the control input.

#### Car Parameters

```wheelbase```: The distance between the front and rear axle of the racecar, measured in meters. As this distance grows the minimum turning radius of the car increases.

```width```: Width of car in meters

```max_speed```: The maximum speed of the car in meters per second.

```max_steering_angle```: The maximum steering angle of the car in radians.

```max_accel```: The maximum acceleration of the car in meters per second squared.

```max_steering_vel```: The maximum steering angle velocity of the car in radians per second.

```friction_coeff```: Coefficient of friction between wheels and ground

```mass```: Mass of car in kilograms

#### Lidar Parameters

```scan_beams```: The number of beams in the scan.

```scan_field_of_view```: The field of view of the lidar, measured in radians. The beams are distributed uniformly throughout this field of view with the first beam being at ```-scan_field_of_view``` and the last beam being at ```scan_field_of_view```. The center of the field of view is direction the racecar is facing.

```scan_distance_to_base_link```: The distance from the lidar to the center of the rear axle (base_link), measured in meters.

```scan_std_dev```: The ammount of noise applied to the lidar measuredments. The noise is gaussian and centered around the correct measurement with standard deviation ```scan_std_dev```, measured in meters.

```map_free_threshold```: The probability threshold for points in the map to be considered "free". This parameter is used to determine what points the simulated scan hits and what points it passes through.

#### Joystick Parameters

```joy```: This boolean parameter enables the joystick if true.

```joy_speed_axis```: The index of the joystick axis used to control the speed of the car. To determine this parameter it may be useful to print out the joystick messages with ```rostopic echo /joy```.

```joy_angle_axis```: The index of the joystick axis used to control the angle of the car.  To determine this parameter it may be useful to print out the joystick messages with ```rostopic echo /joy```.

```joy_button_idx```: The index of the joystick button used to turn on/off joystick driving.

