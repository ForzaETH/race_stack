# Overtaking Sector Tuner
Basically like the Sector Tuner but for the overtaking line. 

It takes care of providing the GUI for slicing the overtaking line in sectors and then publishes the sectors and the corresponding scaling factors once the system is running. 

## Overtaking sector slicer
An example usage is in `<race_stack folder>/base_system/pbl_f110_system/racecar/racecar/launch/pbl_teleop.launch`, where the overtaking sector slicer is launched as follows:
```xml
<!-- Launch Overtaking Sector Slicer/Tuner from Utils -->
<node pkg="overtaking_sector_tuner" type="ot_sector_slicing.py" name="ot_sector_node" output="screen">
    <param name="save_dir" value="$(find stack_master)/maps/$(arg map_name)"/>
</node>
```
This specifically provides the functionality to slice the sectors.

## Overtaking sector server
An example usage is in `<race_stack folder>/base_system/pbl_f110_system/racecar/racecar/launch/pbl_teleop.launch`, where the overtaking sector server is launched as follows:
```xml
<!--  Launch Overtaking Sector Server -->
<node pkg="overtaking_sector_tuner" type="ot_sector_server.py" name="ot_dyn_sector_server" output="screen"/>
<rosparam ns="ot_map_params" command="load" file="$(find stack_master)/maps/$(arg map_name)/ot_sectors.yaml"/>
<!--  Launch Overtaking Interpolator -->
<include file="$(find overtaking_sector_tuner)/launch/ot_interpolator.launch">
    <arg name="yeet_factor" value="1.25" />
</include>
```
This specifically provides the functionality to publish the sectors and publish the overtaking with the velocity smoothly interpolated between sectors.
The trajectory is published at the following topic
```
/global_waypoints/overtaking
```
