-- Shared pure-localization overrides. Include after base.lua.

MAP_BUILDER.num_background_threads = 4.0
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 5,
}

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.global_sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05

--POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1.25 * POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight
