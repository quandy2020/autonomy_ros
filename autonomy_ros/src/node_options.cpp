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
#include "autonomy/common/json_util.hpp"
#include "autonomy/system/system.hpp"
#include "glog/logging.h"

namespace autonomy_ros {

bool CreateNodeOptions(::autonomy::common::LuaParameterDictionary* lua_parameter_dictionary, NodeOptions& node_options)
{
    node_options.autonomy_options =
        ::autonomy::system::CreateAutonomyOptions(
            lua_parameter_dictionary->GetDictionary("autonomy").get());

    node_options.tracking_frame = lua_parameter_dictionary->GetString("tracking_frame");
    node_options.published_frame = lua_parameter_dictionary->GetString("published_frame");
    node_options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
    node_options.map_frame = lua_parameter_dictionary->GetString("map_frame");

    LOG(INFO) << "autonomy options: " << autonomy::common::JsonUtil::ProtoToJson(node_options.autonomy_options);
    return true;
}

NodeOptions LoadOptions(const std::string& configuration_directory, const std::string& configuration_basename)
{

    LOG(INFO) << "Run LoadOptions tfcuntion.";
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