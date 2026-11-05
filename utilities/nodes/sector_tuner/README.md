# Sector Tuner
Small package for handling the sector tuning (i.e. speed scaling and overtaking sectors).
It takes care of providing the GUIs for slicing the racing line in sectors (done for speed scaling and overtaking separately) and then publishes the sectors and the corresponding scaled trajectories and scaling parameters once the system is running. 

## (OT) Sector Slicer
### Basic usage

An example usage is in `<race_stack folder>/stack_master/launch/planner_launch.xml`, where the sector slicers are launched as follows:
```xml
<!-- Launch Sector Slicer from Utils -->
<node pkg="sector_tuner" exec="sector_slicer" name="sector_slicer" output="screen">
    <param name="map_path" value="$(var map_path)" />
</node>

<!-- Launch Overtaking Sector Slicer from Utils -->
<node pkg="sector_tuner" exec="ot_sector_slicer" name="ot_sector_slicer" output="screen">
    <param name="map_path" value="$(var map_path)" />
</node>
```
This specifically provides the functionality to slice the sectors.

### How does it work

One may launch the global planner:
```
ros2 launch stack_master planner_launch.xml map:=test
```
Then the sector slicers will open GUIs for defining the sectors and generating a `speed_scaling.yaml` and a `ot_sectors.yaml`, respectively, within the maps folder. 

Now you can launch the sim and `rqt > Plugins > Configuration > Dynamic Reconfigure`. Enjoy dynamic reconfigure from here on.

## Sector Tuner and OT Interpolator

### Basic usage
An example is in `<race_stack folder>/stack_master/launch/base_system_launch.xml`, where the sector tuner and ot interpolator are launched as follows:
```xml
<!-- launch sector tuner -->
<node pkg="sector_tuner" name="sector_tuner" exec="sector_tuner">
    <param from="$(var map_path)/speed_scaling.yaml" />
</node>

<!-- launch ot sector tuner -->
<node pkg="sector_tuner" name="ot_interpolator" exec="ot_interpolator">
    <param from="$(var map_path)/ot_sectors.yaml" />
</node>
```
Each launched node provides the dynamic reconfigurable parameters, plus the scaled trajectory, with smooth interpolation between the sectors for speed scaling and overtaking respectively.

The trajectory (as an [`f110_msgs/WpntArray`](../../../utilities/libraries/f110_msgs/msg/WpntArray.msg)) is published at the following topic
```
/global_waypoints_scaled
```
The shortest path trajectory is also published at the following topic
```
/global_waypoints_scaled/shortest_path
```

### How does it work

The parameters of the two nodes can be accessed either via the following service servers:
```
/sector_tuner/{get,list,set}_parameters
/ot_interpolator/{get,list,set}_parameters
```
or at the general `/parameter_events` topic (see also [Parameter API design in ROS](https://design.ros2.org/articles/ros_parameters.html))
```
/parameter_events
```
The (speed) sector tuner then listens to the parameters' update and publishes the scaled trajectories by linearly interpolating between sectors, starting `hl_points` points before the end of a sector and finishing `hl_points` points after the beginning of the new sectors (last time I checked `hl_points` == 10).

The nodes provide additional visualizations markers of the scaled trajectories to be displayed in `rviz`.
