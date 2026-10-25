# rs_sectors
Small package for handling the sector tuning (i.e. speed scaling and overtaking sectors).
It takes care of providing the GUIs for slicing the racing line in sectors (done for speed scaling and overtaking separately) and then publishes the sectors and the corresponding scaled trajectories and scaling parameters once the system is running. 

## Sector Slicer
`sector_slicer` splits the raceline into speed sectors and overtaking sectors. It is started by
`stack_master/launch/planner_launch.py` and controlled from the Lichtblick sector slicer panel (one panel per
sector set, "Sector set" setting `speed` or `overtaking`).

Per sector set `<set>` (`speed`, `overtaking`):
- `/sector_slicer/<set>/cursor_s` (`std_msgs/Float64`, from the panel): slider position in meters along the raceline
- `/sector_slicer/<set>/add_boundary` (`rs_msgs/AddBoundary`, `s_m`): add a sector boundary there
- `/sector_slicer/<set>/save` (`std_srvs/Trigger`): write `speed_scaling.yaml` / `ot_sectors.yaml` into the map folder
- `/sector_slicer/<set>/sectors` (`visualization_msgs/MarkerArray`): boundaries and slider position for the 3D panel
  (speed blue, overtaking red)

Boundaries snap to the nearest raceline waypoint. Saving overwrites the file with default `scaling`/`ot_flag` values.
Once both sets are saved the node exits, which ends the planner launch.

## Sector Tuner and OT Interpolator

### Basic usage
An example is in `<race_stack folder>/stack_master/launch/base_system_launch.xml`, where the sector tuner and ot interpolator are launched as follows:
```xml
<!-- launch sector tuner -->
<node pkg="rs_sectors" name="sector_tuner" exec="sector_tuner">
    <param name="map_name" value="$(var map_name)" />
    <param from="$(find-pkg-share stack_master)/maps/$(var map_name)/speed_scaling.yaml" />
</node>

<!-- launch ot sector tuner -->
<node pkg="rs_sectors" name="ot_interpolator" exec="ot_interpolator">
    <param name="map_name" value="$(var map_name)" />
    <param from="$(find-pkg-share stack_master)/maps/$(var map_name)/ot_sectors.yaml" />
</node>
```
Each launched node provides the dynamic reconfigurable parameters, plus the scaled trajectory, with smooth interpolation between the sectors for speed scaling and overtaking respectively.

The trajectory (as an [`rs_msgs/WpntArray`](../../../rs_msgs/msg/WpntArray.msg)) is published at the following topic
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
