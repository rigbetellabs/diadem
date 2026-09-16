include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder          = MAP_BUILDER,
  trajectory_builder   = TRAJECTORY_BUILDER,
  map_frame            = "odom",
  tracking_frame       = "base_link",
  published_frame      = "base_link",
  odom_frame           = "odom",
  provide_odom_frame   = false,
  publish_tracked_pose = true,
  publish_frame_projected_to_2d = true,
  use_odometry         = false,
  use_nav_sat          = false,
  use_landmarks        = false,
  num_laser_scans      = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds     = 0,

  lookup_transform_timeout_sec  = 0.2,
  submap_publish_period_sec     = 0.3,
  pose_publish_period_sec       = 5e-3,  
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio    = 1.0,
  odometry_sampling_ratio       = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio            = 1.0,
  landmarks_sampling_ratio      = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range                  = 0.15
TRAJECTORY_BUILDER_2D.max_range                  = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length    = 12.5
TRAJECTORY_BUILDER_2D.use_imu_data              = false
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window        = 0.20
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window       = math.rad(35.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight    = 1e-2

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight  = 25.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight     = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight        = 1.0 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 25

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.05
TRAJECTORY_BUILDER_2D.num_accumulated_range_data        = 1

TRAJECTORY_BUILDER_2D.submaps.num_range_data          = 30
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type  = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimize_every_n_nodes         = 0
POSE_GRAPH.max_num_final_iterations       = 1
POSE_GRAPH.global_sampling_ratio          = 1e-9
POSE_GRAPH.constraint_builder.sampling_ratio                  = 1e-9
POSE_GRAPH.constraint_builder.min_score                       = 1.0
POSE_GRAPH.constraint_builder.global_localization_min_score   = 1.0

return options
