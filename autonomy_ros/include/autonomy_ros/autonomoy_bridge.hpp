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


#pragma once 

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

// #include "absl/synchronization/mutex.h"

#include "autonomy/system/system.hpp"
#include "autonomy_ros/node_options.hpp"
#include "autonomy_ros/sensor_bridge.hpp"
#include "autonomy_ros/tf_bridge.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


namespace autonomy_ros {


class AutonomyBridge
{
public:
    AutonomyBridge(
        const NodeOptions& node_options,
        std::unique_ptr<::autonomy::system::AutonomyNode> autonomy,
        tf2_ros::Buffer* tf_buffer);

    AutonomyBridge(const AutonomyBridge&) = delete;
    AutonomyBridge& operator=(const AutonomyBridge&) = delete;

    // SensorBridge* sensor_bridge();

private:

    const NodeOptions node_options_;

    std::unique_ptr<::autonomy::system::AutonomyNode> autonomy_builder_{nullptr};
    tf2_ros::Buffer* const tf_buffer_{nullptr};
};

}  // namespace autonomy_ros