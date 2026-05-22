// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_ROS__CONVERSIONS__CONVERSIONS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__CONVERSIONS_HPP_

/// @file conversions.hpp
/// @brief Umbrella header for all autonomy_ros ROS ↔ commsgs conversions.
///
/// This header pulls in every package-specific conversions/*.hpp module. Each
/// module mirrors a protobuf package under autonomy/commsgs/proto/*.proto and
/// declares overloaded fromRos() / toRos() functions in namespace
/// autonomy_ros::conversions.
///
/// @par What is converted here
/// - ROS 2 messages (rclcpp types under *::msg::*) ↔ C++ commsgs structs
///   (autonomy::commsgs::*).
/// - Field-wise copies only; no coordinate transforms, no TF math.
///
/// @par What is NOT converted here
/// Protobuf wire encoding uses the core library, e.g.
/// autonomy::commsgs::geometry_msgs::ToProto() and FromProto(). Do not add
/// toProto/fromProto helpers in autonomy_ros.
///
/// @par Usage (typical node)
/// 1. Include this file once in bridges, nodes, or adapters.
/// 2. On subscription: convert incoming ROS messages to commsgs before calling
///    autonomy core APIs.
/// 3. On publication: convert commsgs results back to ROS before publish().
///
/// @par Subscribe example
/// @code
/// #include "autonomy_ros/conversions/conversions.hpp"
/// void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
///   auto goal = autonomy_ros::conversions::fromRos(*msg);
///   // pass goal to planner; optional: autonomy::commsgs::geometry_msgs::ToProto(goal)
/// }
/// @endcode
///
/// @par Publish example
/// @code
/// #include "autonomy_ros/conversions/conversions.hpp"
/// void publish_plan(const autonomy::commsgs::planning_msgs::Path & path) {
///   pub_->publish(autonomy_ros::conversions::toRos(path));
/// }
/// @endcode
///
/// @par Narrow includes
/// For faster builds, include only the package you need, e.g.
/// autonomy_ros/conversions/sensor_msgs.hpp.
///
/// See also: share/autonomy_ros/docs/conversions.md

#include "autonomy_ros/conversions/builtin_interfaces.hpp"
#include "autonomy_ros/conversions/diagnostic_msgs.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/map_msgs.hpp"
#include "autonomy_ros/conversions/pcl_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"
#include "autonomy_ros/conversions/shape_msgs.hpp"
#include "autonomy_ros/conversions/std_msgs.hpp"
#include "autonomy_ros/conversions/stereo_msgs.hpp"
#include "autonomy_ros/conversions/tf2_msgs.hpp"
#include "autonomy_ros/conversions/trajectory_msgs.hpp"
#include "autonomy_ros/conversions/vision_msgs.hpp"
#include "autonomy_ros/conversions/visualization_msgs.hpp"

#endif  // AUTONOMY_ROS__CONVERSIONS__CONVERSIONS_HPP_
