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

#ifndef AUTONOMY_ROS__CONVERSIONS__BUILTIN_INTERFACES_HPP_
#define AUTONOMY_ROS__CONVERSIONS__BUILTIN_INTERFACES_HPP_

/// @file builtin_interfaces.hpp
/// @brief Converts ROS 2 builtin_interfaces Time/Duration and automsgs equivalents.
///
/// Proto schema: automsgs/msgs/builtin_interfaces/
///
/// @par Overview
/// Time and Duration are copied as integer sec and nanosec fields. These types
/// are embedded by std_msgs/Header and many stamped messages.
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) and call overloads in
/// autonomy_ros::conversions:
/// - fromRos(ros_msg) returns automsgs::msgs::builtin_interfaces::* by value.
/// - toRos(automsgs_msg) builds builtin_interfaces::msg::* for ROS I/O.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/builtin_interfaces.hpp"
/// builtin_interfaces::msg::Time ros_t = ...;
/// auto core_t = autonomy_ros::fromRos(ros_t);
/// auto back = autonomy_ros::toRos(core_t);
/// @endcode

#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace autonomy_ros
{

using Time = ::automsgs::msgs::builtin_interfaces::Time;
using Duration = ::automsgs::msgs::builtin_interfaces::Duration;

/**
 * @brief Bidirectional conversion between ROS builtin_interfaces::msg::Time and automsgs Time.
 *
 * @par fromRos
 * @param from Input ROS message (builtin_interfaces::msg::Time). Fields are copied without coordinate transforms.
 * @return automsgs Time for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Time from planners, bridges, or drivers.
 * @return ROS builtin_interfaces::msg::Time ready for rclcpp publish() or subscribe() adapters.
 */
Time fromRos(const builtin_interfaces::msg::Time & from);
builtin_interfaces::msg::Time toRos(const Time & from);

/**
 * @brief Bidirectional conversion between ROS builtin_interfaces::msg::Duration and automsgs Duration.
 *
 * @par fromRos
 * @param from Input ROS message (builtin_interfaces::msg::Duration). Fields are copied without coordinate transforms.
 * @return automsgs Duration for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Duration from planners, bridges, or drivers.
 * @return ROS builtin_interfaces::msg::Duration ready for rclcpp publish() or subscribe() adapters.
 */
Duration fromRos(const builtin_interfaces::msg::Duration & from);
builtin_interfaces::msg::Duration toRos(const Duration & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__BUILTIN_INTERFACES_HPP_
