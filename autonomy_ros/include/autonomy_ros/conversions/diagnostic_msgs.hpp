// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__DIAGNOSTIC_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__DIAGNOSTIC_MSGS_HPP_

/// @file diagnostic_msgs.hpp
/// @brief Converts ROS 2 diagnostic_msgs and autonomy::commsgs::diagnostic_msgs.
///
/// Proto schema: autonomy/commsgs/proto/diagnostic_msgs.proto
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) when forwarding /diagnostics
/// or robot monitor output into the autonomy stack:
/// - fromRos(diagnostic_msgs::msg::DiagnosticArray) for incoming ROS monitors.
/// - toRos() to republish commsgs diagnostic state to ROS tools.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/diagnostic_msgs.hpp"
/// void on_diag(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
///   auto arr = autonomy_ros::conversions::fromRos(*msg);
/// }
/// @endcode

#include "autonomy/commsgs/diagnostic_msgs.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace autonomy_ros::conversions
{

using KeyValue = ::autonomy::commsgs::diagnostic_msgs::KeyValue;
using DiagnosticStatus = ::autonomy::commsgs::diagnostic_msgs::DiagnosticStatus;
using DiagnosticArray = ::autonomy::commsgs::diagnostic_msgs::DiagnosticArray;

/**
 * @brief Bidirectional conversion between ROS diagnostic_msgs::msg::KeyValue and commsgs KeyValue.
 *
 * @par fromRos
 * @param from Input ROS message (diagnostic_msgs::msg::KeyValue). Fields are copied without coordinate transforms.
 * @return commsgs KeyValue for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs KeyValue from planners, bridges, or drivers.
 * @return ROS diagnostic_msgs::msg::KeyValue ready for rclcpp publish() or subscribe() adapters.
 */
KeyValue fromRos(const diagnostic_msgs::msg::KeyValue & from);
diagnostic_msgs::msg::KeyValue toRos(const KeyValue & from);

/**
 * @brief Bidirectional conversion between ROS diagnostic_msgs::msg::DiagnosticStatus and commsgs DiagnosticStatus.
 *
 * @par fromRos
 * @param from Input ROS message (diagnostic_msgs::msg::DiagnosticStatus). Fields are copied without coordinate transforms.
 * @return commsgs DiagnosticStatus for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs DiagnosticStatus from planners, bridges, or drivers.
 * @return ROS diagnostic_msgs::msg::DiagnosticStatus ready for rclcpp publish() or subscribe() adapters.
 */
DiagnosticStatus fromRos(const diagnostic_msgs::msg::DiagnosticStatus & from);
diagnostic_msgs::msg::DiagnosticStatus toRos(const DiagnosticStatus & from);

/**
 * @brief Bidirectional conversion between ROS diagnostic_msgs::msg::DiagnosticArray and commsgs DiagnosticArray.
 *
 * @par fromRos
 * @param from Input ROS message (diagnostic_msgs::msg::DiagnosticArray). Fields are copied without coordinate transforms.
 * @return commsgs DiagnosticArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs DiagnosticArray from planners, bridges, or drivers.
 * @return ROS diagnostic_msgs::msg::DiagnosticArray ready for rclcpp publish() or subscribe() adapters.
 */
DiagnosticArray fromRos(const diagnostic_msgs::msg::DiagnosticArray & from);
diagnostic_msgs::msg::DiagnosticArray toRos(const DiagnosticArray & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__DIAGNOSTIC_MSGS_HPP_
