/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy_ros/node_options.hpp"

#include <vector>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/system/common/system_interface.hpp"
#include "autonomy/common/json_util.hpp"
#include "autonomy/system/system.hpp"
#include "glog/logging.h"

namespace autonomy_ros {

bool CreateNodeOptions(::autonomy::common::LuaParameterDictionary* lua_parameter_dictionary, NodeOptions& node_options)
{
    node_options.autonomy_options = ::autonomy::system::common::LoadOptions(
        lua_parameter_dictionary->GetDictionary("autonomy").get());
    node_options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    node_options.base_frame= lua_parameter_dictionary->GetString("base_frame");
    node_options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
    node_options.use_imu_data = lua_parameter_dictionary->GetBool("use_imu_data");
    node_options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
    node_options.use_nav_sat = lua_parameter_dictionary->GetBool("use_nav_sat");
    node_options.use_landmarks = lua_parameter_dictionary->GetBool("use_landmarks");
    node_options.num_laser_scans = lua_parameter_dictionary->GetInt("num_laser_scans");
    node_options.num_multi_echo_laser_scans = lua_parameter_dictionary->GetInt("num_multi_echo_laser_scans");
    node_options.num_subdivisions_per_laser_scan = lua_parameter_dictionary->GetInt("num_subdivisions_per_laser_scan");
    node_options.num_point_clouds = lua_parameter_dictionary->GetInt("num_point_clouds");

    node_options.global_plan_publish_period_sec = lua_parameter_dictionary->GetDouble("global_plan_publish_period_sec");
    node_options.lookup_transform_timeout_sec = lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
    node_options.rangefinder_sampling_ratio = lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
    node_options.odometry_sampling_ratio = lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
    node_options.fixed_frame_pose_sampling_ratio = lua_parameter_dictionary->GetDouble("fixed_frame_pose_sampling_ratio");
    node_options.imu_sampling_ratio = lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
    node_options.landmarks_sampling_ratio = lua_parameter_dictionary->GetDouble("landmarks_sampling_ratio");

    if (lua_parameter_dictionary->GetBool("show_configuration_contents")) {
        LOG(INFO) << "autonomy options: " 
                  << autonomy::common::JsonUtil::ProtoToJson(node_options.autonomy_options);
    }
    return true;
}

NodeOptions LoadOptions(const std::string& configuration_directory, const std::string& configuration_basename)
{
    auto file_resolver = std::make_unique<::autonomy::common::ConfigurationFileResolver>(std::vector<std::string>{configuration_directory});
    const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
    ::autonomy::common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));

    NodeOptions options;
    bool success = CreateNodeOptions(&lua_parameter_dictionary, options);
    if (!success) {
        LOG(ERROR) << "Create nodes option error.";
    }
    return options;
}

}  // namespace autonomy_ros