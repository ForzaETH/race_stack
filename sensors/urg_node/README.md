urg_node
===================

### Branches

 - ROS1:
   - Melodic & Noetic: [melodic-devel](https://github.com/ros-drivers/urg_node/tree/melodic-devel)
   - Kinetic: [kinetic-devel](https://github.com/ros-drivers/urg_node/tree/kinetic-devel)
- ROS2:
   - Galactic, Humble, Rolling: [ros2-devel](https://github.com/ros-drivers/urg_node/tree/ros2-devel)

### LaserScan Visualization

Until the launch API is sorted out, there are two ways to view the laserscan in RViz:

1) use the static_transform_publisher tool from tf2_ros to publish a static transform for a fixed frame simultaneously with rviz and urg_node.

```
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id laser
```

where "laser" is the frame id, which you can set in your urg_node yaml file

2) Run the robot_state_publisher along with a urdf file so that a fixed frame can be loaded in rviz

```
ros2 run robot_state_publisher robot_state_publisher <path to urdf file>
```

A urdf file is already included and if you have succesfully ran an `colcon build`, the path to urdf file will be:

```
<path to ros2_ws>/install/share/urg_node/launch/hokuyo_laser.urdf
```

Now run RViz:

```
ros2 run rviz2 rviz2
```
or simply `rviz2`

Add the LaserScan topic and change the global fixed frame from 'map' to 'world' to display the laser scan.


### Parameters

A YAML file example is included in the launch folder, all the available parameters are listed in it.
For example (note that the serial_port is commented because you can't set a param with an empty string):

```
urg_node:
  ros__parameters:
    ip_address: "192.168.0.10"
    ip_port: 10940
    #serial_port: ""
    serial_baud: 115200
    laser_frame_id: laser
    angle_max: 3.14
    angle_min: -3.14
    publish_intensity: false
    publish_multiecho: false
    calibrate_time: false
    default_user_latency: 0
    diagnostics_tolerance: 0.05
    diagnostics_window_time: 5.0
    error_limit: 4
    get_detailed_status: false
    cluster: 0
    skip: 1
```

To give parameters to urg_node :

```
ros2 run urg_node urg_node_driver --ros-args --params-file path/to/my/file.yaml
```

You can reconfigure parameters while the node is launched.
For now, you can only reconfigure the following parameters:

```
laser_frame_id
error_limit
default_user_latency
angle_max
angle_min
cluster
skip
```

For example to reconfigure the cluster parameter using command line :
```
ros2 param set /urg_node cluster 1
```

### How to use the ust-20lx (and other Ethernet-based laser)

To use ust-20lx you need to be on the same subnet as the laser.
The ust-20lx default IP address is 192.168.0.10, so you might need to change your IP to something on the same subnet.

On Ubuntu :
- Network settings
- Wired connection
- Under IPv4, change the IPv4 method from automatic to Manual and under Addresses, set the values:

```
    Address : 192.168.0.15
    Netmask : 255.255.255.0
    Gateway : 192.168.0.1
```

- Turn the connection off and on again to apply the change
- To check if it worked, open a terminal and type

```
ip --color address
```
or simply `ip -c a`

- Under eth0 (or maybe something like enpxxxxx), your IP should be 192.168.0.15.
You should now be able to ping the ust-20lx at its address (by default 192.168.0.10)

```
ping 192.168.0.10
```

If you don't receive any answer, you might have a connection problem or the IP of your laser might have changed. If the IP was changed, you can either change your computer's IP to the same subnet or reset the laser to factory settings.

- Once you can ping the laser, launch the urg_node_driver:

```
ros2 run urg_node urg_node_driver --ros-args --params-file path/to/my/file.yaml
```
