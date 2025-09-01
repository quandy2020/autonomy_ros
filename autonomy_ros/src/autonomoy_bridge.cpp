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

#include "autonomy_ros/autonomoy_bridge.hpp"

namespace autonomy_ros {


AutonomyBridge::AutonomyBridge(
    const NodeOptions& node_options,
    tf2_ros::Buffer* tf_buffer)
    : node_options_{node_options},
      tf_buffer_{tf_buffer}
{
  // autonomy_builder_ = ::autonomy::system::CreateAutonomyBuilder(node_options.autonomy_options);
}

SensorBridge* AutonomyBridge::sensor_bridge()
{
    return sensor_bridges_.get();
}

::autonomy::system::AutonomyNode* AutonomyBridge::AutonomySystemNode()
{
    return autonomy_builder_.get();
}

}  // namespace autonomy_ros