/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 * email: quandy2020@126.com
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

// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__PLANNING_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__PLANNING_MSGS_HPP_

/// @file planning_msgs.hpp
/// @brief Converts automsgs planning_msgs to/from ROS nav_msgs where applicable.
///
/// Proto schema: automsgs/msgs/nav_msgs/ (Path, Odometry)
///
/// @par Implemented mappings
/// - Odometry ↔ nav_msgs/Odometry (shared by localization and planning stacks).
/// - Path ↔ nav_msgs/Path (e.g. Autonomy publishes /plan).
///
/// @par Not implemented (no standard ROS 2 message)
/// Path2D, Point2D, Twist2D, Pose2DStamped, Goals, CostmapFilterInfo.
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp). Path conversion
/// depends on geometry_msgs PoseStamped helpers (included transitively).
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/planning_msgs.hpp"
/// void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
///   auto odom = autonomy_ros::fromRos(*msg);
/// }
/// pub_plan->publish(autonomy_ros::toRos(core_path));
/// @endcode

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace autonomy_ros
{

using Odometry = ::automsgs::msgs::nav_msgs::Odometry;
using Path = ::automsgs::msgs::nav_msgs::Path;

/**
 * @brief Bidirectional conversion between ROS nav_msgs::msg::Odometry and automsgs Odometry.
 *
 * @par fromRos
 * @param from Input ROS message (nav_msgs::msg::Odometry). Fields are copied without coordinate transforms.
 * @return automsgs Odometry for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Odometry from planners, bridges, or drivers.
 * @return ROS nav_msgs::msg::Odometry ready for rclcpp publish() or subscribe() adapters.
 */
Odometry fromRos(const nav_msgs::msg::Odometry & from);
nav_msgs::msg::Odometry toRos(const Odometry & from);

/**
 * @brief Bidirectional conversion between ROS nav_msgs::msg::Path and automsgs Path.
 *
 * @par fromRos
 * @param from Input ROS message (nav_msgs::msg::Path). Fields are copied without coordinate transforms.
 * @return automsgs Path for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Path from planners, bridges, or drivers.
 * @return ROS nav_msgs::msg::Path ready for rclcpp publish() or subscribe() adapters.
 */
Path fromRos(const nav_msgs::msg::Path & from);
nav_msgs::msg::Path toRos(const Path & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__PLANNING_MSGS_HPP_
