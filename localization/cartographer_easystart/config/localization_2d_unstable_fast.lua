include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_tracked_pose = true,
}

-- Set the map builder to use localization only
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 6
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
-- Disable submap creation for localization
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10 --10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.001 --0.01
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.0005 --25

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- Adjust for better localization
TRAJECTORY_BUILDER_2D.min_range = 0.12
-- TRAJECTORY_BUILDER_2D.max_range = 3.5

TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 8.0
TRAJECTORY_BUILDER_2D.max_range = 50.0

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- Adjust pose graph settings for localization
-- POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.min_score = 0.2
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

POSE_GRAPH.constraint_builder.sampling_ratio = 0.01
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.constraint_builder.loop_closure_translation_weight =  3e5 --3e5
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e6 --1e6
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 5 -- 15
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight =  5 --10.  --0.01
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight =  0.1 

return options