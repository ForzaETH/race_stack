# Set Pose

This package provides the capacities to change the position estimate of the real car.
It is particularly useful at startup, when the localization system has not converged and therefore automatic driving cannot be launched right away. 
This package can however be also useful in other circumstances when the car localization system has not properly converged, e.g. after a crash.

## Basic usage
The package inclusion in a launch file can be seen in the stack master [`base_system`](../../../stack_master/launch/base_system.launch) launch file.
```xml
<!--  launch slam pose setter -->
<node pkg="set_pose" type="set_pose_node.py" name="set_slam_pose_node" output="screen">
    <!-- Set the config dir of SLAM -->
    <param name="config_dir" value="$(find stack_master)/config/$(arg racecar_version)/slam"/>
    <param name="config_base" value="f110_2d_loc.lua"/>
</node>
```

## How does it works
The pose can be set easily with the "2d Pose Estimate" button in the RViz interface. 

---
[Go back to the utils list](../../README.md)