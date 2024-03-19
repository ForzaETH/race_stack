# Sector Tuner
small package for handling the sector tuning.
It takes care of providing the GUI for slicing the racing line in sectors and then publishes the sectors and the corresponding scaled trajectories and scaling parameters once the system is running. 

## Sector slicer
### Basic usage

An example usage is in `<race_stack folder>/base_system/f1tenth_system/racecar/racecar/launch/pbl_teleop.launch`, where the overtaking sector slicer is launched as follows:
```xml
<!-- Launch Sector Slicer/Tuner from Utils -->
<node pkg="sector_tuner" type="sector_slicing.py" name="sector_node" output="screen">
    <param name="save_dir" value="$(find stack_master)/maps/$(arg map_name)"/>
</node>
```
This specifically provides the functionality to slice the sectors.

### How does it work

One may launch the simulator with mapping enabled:
```
roslaunch f1tenth_simulator pbl_sim.launch mapping_bool:=True map_name:=berlin
```
Then the sector slicer will open a GUI for defining the sectors and generate `sector_scaling.yaml` within the maps folder. 

Additionally it will copy said `yaml` into the `sector_tuner` package and rebuild the package for the dynamic reconf build [here](./cfg/dyn_sect_tuner.cfg).
**You will need to use a freshly sourced roscore from here on.**

Now you can launch the sim without the mapping bool and launch `rqt > Plugins > Configuration > Dynamic Reconfigure`. Enjoy dynamic reconfigure from here on.

## Sector server

### Basic usage
An example is in `<race_stack folder>/base_system/f1tenth_system/racecar/racecar/launch/pbl_teleop.launch`, where the sector server is launched as follows:
```xml
<!--  launch dynamic reconfigure for the sectors -->
<node pkg="sector_tuner" type="sector_server.py" name="dyn_sector_server" output="screen"/>
<rosparam ns="map_params" command="load" file="$(find stack_master)/maps/$(arg map_name)/speed_scaling.yaml"/>
<!--  launch Velocity Interpolator for dynamic sector -->
<include file="$(find sector_tuner)/launch/velocity_scaler.launch">
    <arg name="debug_plot" value="False" />
</include>
```
The first launched node provides the dynamic reconfigurable parameters, i.e. the sectors starts and end plus the velocity scaling.
Then the second launch file provides the scaled trajectory, with smooth inerpolation between the sectors.

The trajectory (as an [`f110_msgs/WpntArray`](../../../f110_utils/libs/f110_msgs/msg/WpntArray.msg)) is published at the following topic
```
/global_waypoints_scaled
```
The shortest path trajectory is also published at the following topic
```
/global_waypoints_scaled/shortest_path
```

### How does it work

The dynamic parameters server provides the info needed at either the following parameter:
```
/dyn_sector_server
```
or at the following topic (message type: [`dynamic_reconfigure/Config`](http://docs.ros.org/en/noetic/api/dynamic_reconfigure/html/msg/Config.html))
```
/dyn_sector_server/parameter_updates
```
The velocity scaler then listens to the parameters' update and publishes the scaled trajectory by linearly interpolating between sectors, starting `hl_points` points before the end of a sector and finishing `hl_points` points after the beginning of the new sectors (last time I checked `hl_points` == 10).

ROS parameters:
  -  `debug_plot` : it activates a matplotlib plot to visualise the the velocity scaling across the whole track. Unfortunately it breaks down the publishing of the scaled waypoints, therefore it is only for debugging purposes.

---
[Go back to the utils list](../../README.md)