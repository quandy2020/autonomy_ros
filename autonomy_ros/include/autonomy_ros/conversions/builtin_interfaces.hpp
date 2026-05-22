// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__BUILTIN_INTERFACES_HPP_
#define AUTONOMY_ROS__CONVERSIONS__BUILTIN_INTERFACES_HPP_

/// @file builtin_interfaces.hpp
/// @brief Converts ROS 2 builtin_interfaces Time/Duration and commsgs equivalents.
///
/// Proto schema: autonomy/commsgs/proto/builtin_interfaces.proto
///
/// @par Overview
/// Time and Duration are copied as integer sec and nanosec fields. These types
/// are embedded by std_msgs/Header and many stamped messages.
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) and call overloads in
/// autonomy_ros::conversions:
/// - fromRos(ros_msg) returns autonomy::commsgs::builtin_interfaces::* by value.
/// - toRos(commsgs_msg) builds builtin_interfaces::msg::* for ROS I/O.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/builtin_interfaces.hpp"
/// builtin_interfaces::msg::Time ros_t = ...;
/// auto core_t = autonomy_ros::conversions::fromRos(ros_t);
/// auto back = autonomy_ros::conversions::toRos(core_t);
/// @endcode

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace autonomy_ros::conversions
{

using Time = ::autonomy::commsgs::builtin_interfaces::Time;
using Duration = ::autonomy::commsgs::builtin_interfaces::Duration;

/**
 * @brief Bidirectional conversion between ROS builtin_interfaces::msg::Time and commsgs Time.
 *
 * @par fromRos
 * @param from Input ROS message (builtin_interfaces::msg::Time). Fields are copied without coordinate transforms.
 * @return commsgs Time for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Time from planners, bridges, or drivers.
 * @return ROS builtin_interfaces::msg::Time ready for rclcpp publish() or subscribe() adapters.
 */
Time fromRos(const builtin_interfaces::msg::Time & from);
builtin_interfaces::msg::Time toRos(const Time & from);

/**
 * @brief Bidirectional conversion between ROS builtin_interfaces::msg::Duration and commsgs Duration.
 *
 * @par fromRos
 * @param from Input ROS message (builtin_interfaces::msg::Duration). Fields are copied without coordinate transforms.
 * @return commsgs Duration for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Duration from planners, bridges, or drivers.
 * @return ROS builtin_interfaces::msg::Duration ready for rclcpp publish() or subscribe() adapters.
 */
Duration fromRos(const builtin_interfaces::msg::Duration & from);
builtin_interfaces::msg::Duration toRos(const Duration & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__BUILTIN_INTERFACES_HPP_
