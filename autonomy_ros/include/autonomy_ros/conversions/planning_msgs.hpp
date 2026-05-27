// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__PLANNING_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__PLANNING_MSGS_HPP_

/// @file planning_msgs.hpp
/// @brief Converts commsgs planning_msgs to/from ROS nav_msgs where applicable.
///
/// Proto schema: autonomy/commsgs/proto/planning_msgs.proto
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
///   auto odom = autonomy_ros::conversions::fromRos(*msg);
/// }
/// pub_plan->publish(autonomy_ros::conversions::toRos(core_path));
/// @endcode

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace autonomy_ros::conversions
{

using Odometry = ::autonomy::commsgs::planning_msgs::Odometry;
using Path = ::autonomy::commsgs::planning_msgs::Path;

/**
 * @brief Bidirectional conversion between ROS nav_msgs::msg::Odometry and commsgs Odometry.
 *
 * @par fromRos
 * @param from Input ROS message (nav_msgs::msg::Odometry). Fields are copied without coordinate transforms.
 * @return commsgs Odometry for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Odometry from planners, bridges, or drivers.
 * @return ROS nav_msgs::msg::Odometry ready for rclcpp publish() or subscribe() adapters.
 */
Odometry fromRos(const nav_msgs::msg::Odometry & from);
nav_msgs::msg::Odometry toRos(const Odometry & from);

/**
 * @brief Bidirectional conversion between ROS nav_msgs::msg::Path and commsgs Path.
 *
 * @par fromRos
 * @param from Input ROS message (nav_msgs::msg::Path). Fields are copied without coordinate transforms.
 * @return commsgs Path for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Path from planners, bridges, or drivers.
 * @return ROS nav_msgs::msg::Path ready for rclcpp publish() or subscribe() adapters.
 */
Path fromRos(const nav_msgs::msg::Path & from);
nav_msgs::msg::Path toRos(const Path & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__PLANNING_MSGS_HPP_
