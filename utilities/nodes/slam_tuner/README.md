# Slam Tuner
This package provides functionalities to tune Localization packages.

## Launch Procedure

Start by getting your favorite rosbag file (.db3) and put it in the `data` folder of the `slam_tuner` folder. Then run `colcon build` from the `ws` directory so that the data files are transferred. 

You can edit SLAM parameters as explained below and rerun the localization/mapping algorithms. After changing parameters, make sure to `colcon build`.

Then run the `tuner_launch.xml` launch file, for example:

```bash
ros2 launch slam_tuner tuner_launch.xml bag:=mapping map_name:=GLC_smile_small racecar_version:=NUC6 bag_ekf:=False rviz:=True rate:=1.0 mapping_bool:=True
```

This launch file has a few parameters:
- `bag`: Name of the rosbag folder within `slam_tuner/data`
- `map_name`: Map you wish to get from `stack_master/maps`
- `racecar_version`: Choose the relevant NUC lookup tables
- `bag_ekf`: Whether or not to use the EKF estimates from the bag (early fusion; set to True) or alternatively to run the EKF live (set to False)
- `rviz`: Whether or not to visualize
- `rate`: The rate at which the rosbag is replayed (reduce load)
- `mapping_bool`: Whether to run in mapping mode (otherwise localization mode is used)

The launch file should replay the Rosbag and show the mapping/localization process in Rviz. It's then up to you to tune the relevant parameters in `slam_tuner/config`.

## Tuning Parameters
### `Cartographer`
When mapping, edit the `f110_2d.lua` (for Mapping) or `f110_2d_loc.lua` (for Localization) file.

Based on our trust on Odometry values:
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight`
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight`

These weight the cost of rotating/translating incoming scans (from the odometry reference). If we trust our odometry, we can weight these values a little higher, but if not (our IMU and `vesc/odom` values aren't great at the moment) the default values are `0.2`.

- `POSE_GRAPH.optimization_problem.odometry_rotation_weight`
- `POSE_GRAPH.optimization_problem.odometry_translation_weight`

These weight how much the pose graph weights odometry (over scan matching). Similarly to above, but when performing localization of the car. Default values are `0`.

Based on our trust of LIDAR scans:
- `TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability`
`TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability`

These weight the probability of a hit or miss in the occupancy grid.
- Hit probability: The trust in cells where we get range data
- Miss probability: The trust in cells where we observe free space

If we don't trust the lidar (eg. black tubes), we should reduce the miss probability (free space may mean undetected things).

