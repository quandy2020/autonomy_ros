-- Copyright 2025 The Openbot Authors (duyongquan)
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

include "autonomy.lua"

-- all lua config
options = {
    autonomy = AUTONOMY,
    map_frame = "map",
    base_frame = "base_footprint",
    odom_frame = "odom",
    use_imu_data = true,
    use_odometry = true,
    use_nav_sat = false,
    use_landmarks = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 1,
    num_subdivisions_per_laser_scan = 10,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,
    global_plan_publish_period_sec = 0.2,
    rangefinder_sampling_ratio = 1.,
    odometry_sampling_ratio = 1.,
    fixed_frame_pose_sampling_ratio = 1.,
    imu_sampling_ratio = 1.,
    landmarks_sampling_ratio = 1.,
    show_configuration_contents = true
}

return options