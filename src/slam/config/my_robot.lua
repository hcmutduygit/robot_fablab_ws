-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",

  -- DÙNG IMU LÀM TRACKING FRAME
  tracking_frame = "imu",
  published_frame = "base_footprint",
  odom_frame = "odom",

  -- Cartographer tự publish odom -> base_footprint
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  -- DÙNG ENCODER
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ===== TRAJECTORY BUILDER 2D =====
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Nếu lidar <= 15Hz → 1, nếu >= 20Hz → 2
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.min_range = 0.25
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03

-- ===== POSE GRAPH =====
POSE_GRAPH.optimize_every_n_nodes = 30

-- ENCODER CHỈ LÀ HỖ TRỢ, KHÔNG ĐƯỢC BỎ
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

-- ===== MAPPING MODE =====
TRAJECTORY_BUILDER.pure_localization = false

return options
