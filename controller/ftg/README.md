# Follow The Gap

## Description
The Follow-The-Gap (FTG) Controller is a simple reactive controller which uses directly the LiDAR scans and the car's pose to compute and publish the velocity and steering angle of the car. In our race stack, the FTG controller is mainly used for mapping, so that we do not have to drive manually around the track. However, even though never used in practice anymore, there is the possibility to set a sector in the state machine as `FTGONLY` and then the FTG controller managed by `controller_manager.py` is used during these sectors.

 ## Parameters
 - `track_width`: Approximate width of the track.
 - `max_speed`: Maximum speed allowed.
 - `safety_radius`: Safety radius to reduce cutting corners.
 - `max_lidar_dist`: Maximum possible scan distance of the LiDAR.
 - `range_offset`: Range of LiDAR points to be considered, only consider `[range_offset,-range_offset]`.
 - `debug`: If set to `True`, the best gap, best point and preprocessed LiDAR scans are published as Markers.
