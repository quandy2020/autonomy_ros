// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__DETAIL_HPP_
#define AUTONOMY_ROS__CONVERSIONS__DETAIL_HPP_

/// @file detail.hpp
/// @brief Internal field-copy helpers for autonomy_ros conversions (not public API).
///
/// @par Usage
/// Do not include from application code. Implementation files under
/// src/conversions/*.cpp use autonomy_ros::conversions::detail::copy* to share
/// logic between fromRos() and toRos() overloads. Prefer the public headers
/// (e.g. geometry_msgs.hpp) and fromRos / toRos at call sites.

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/inertia.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"

namespace autonomy_ros::conversions::detail
{

void copyTime(
  const builtin_interfaces::msg::Time & from,
  ::autonomy::commsgs::builtin_interfaces::Time & to);

void copyTime(
  const rclcpp::Time & from,
  ::autonomy::commsgs::builtin_interfaces::Time & to);

builtin_interfaces::msg::Time toRosTime(
  const ::autonomy::commsgs::builtin_interfaces::Time & from);

void copyHeader(
  const std_msgs::msg::Header & from,
  ::autonomy::commsgs::std_msgs::Header & to);

void copyHeader(
  const ::autonomy::commsgs::std_msgs::Header & from,
  std_msgs::msg::Header & to);

void copyQuaternion(
  const geometry_msgs::msg::Quaternion & from,
  ::autonomy::commsgs::geometry_msgs::Quaternion & to);

void copyQuaternion(
  const ::autonomy::commsgs::geometry_msgs::Quaternion & from,
  geometry_msgs::msg::Quaternion & to);

void copyPoint(
  const geometry_msgs::msg::Point & from,
  ::autonomy::commsgs::geometry_msgs::Point & to);

void copyPoint(
  const ::autonomy::commsgs::geometry_msgs::Point & from,
  geometry_msgs::msg::Point & to);

void copyPoint32(
  const geometry_msgs::msg::Point32 & from,
  ::autonomy::commsgs::geometry_msgs::Point32 & to);

void copyPoint32(
  const ::autonomy::commsgs::geometry_msgs::Point32 & from,
  geometry_msgs::msg::Point32 & to);

void copyVector3(
  const geometry_msgs::msg::Vector3 & from,
  ::autonomy::commsgs::geometry_msgs::Vector3 & to);

void copyVector3(
  const ::autonomy::commsgs::geometry_msgs::Vector3 & from,
  geometry_msgs::msg::Vector3 & to);

void copyPose2D(
  const geometry_msgs::msg::Pose2D & from,
  ::autonomy::commsgs::geometry_msgs::Pose2D & to);

void copyPose2D(
  const ::autonomy::commsgs::geometry_msgs::Pose2D & from,
  geometry_msgs::msg::Pose2D & to);

void copyPose(
  const geometry_msgs::msg::Pose & from,
  ::autonomy::commsgs::geometry_msgs::Pose & to);

void copyPose(
  const ::autonomy::commsgs::geometry_msgs::Pose & from,
  geometry_msgs::msg::Pose & to);

void copyTwist(
  const geometry_msgs::msg::Twist & from,
  ::autonomy::commsgs::geometry_msgs::Twist & to);

void copyTwist(
  const ::autonomy::commsgs::geometry_msgs::Twist & from,
  geometry_msgs::msg::Twist & to);

void copyAccel(
  const geometry_msgs::msg::Accel & from,
  ::autonomy::commsgs::geometry_msgs::Accel & to);

void copyAccel(
  const ::autonomy::commsgs::geometry_msgs::Accel & from,
  geometry_msgs::msg::Accel & to);

void copyTransform(
  const geometry_msgs::msg::Transform & from,
  ::autonomy::commsgs::geometry_msgs::Transform & to);

void copyTransform(
  const ::autonomy::commsgs::geometry_msgs::Transform & from,
  geometry_msgs::msg::Transform & to);

void copyWrench(
  const geometry_msgs::msg::Wrench & from,
  ::autonomy::commsgs::geometry_msgs::Wrench & to);

void copyWrench(
  const ::autonomy::commsgs::geometry_msgs::Wrench & from,
  geometry_msgs::msg::Wrench & to);

void copyInertia(
  const geometry_msgs::msg::Inertia & from,
  ::autonomy::commsgs::geometry_msgs::Inertia & to);

void copyInertia(
  const ::autonomy::commsgs::geometry_msgs::Inertia & from,
  geometry_msgs::msg::Inertia & to);

void copyPolygon(
  const geometry_msgs::msg::Polygon & from,
  ::autonomy::commsgs::geometry_msgs::Polygon & to);

void copyPolygon(
  const ::autonomy::commsgs::geometry_msgs::Polygon & from,
  geometry_msgs::msg::Polygon & to);

void copyDuration(
  const builtin_interfaces::msg::Duration & from,
  ::autonomy::commsgs::builtin_interfaces::Duration & to);

void copyDuration(
  const ::autonomy::commsgs::builtin_interfaces::Duration & from,
  builtin_interfaces::msg::Duration & to);

void copyColorRGBA(
  const std_msgs::msg::ColorRGBA & from,
  ::autonomy::commsgs::std_msgs::ColorRGBA & to);

void copyColorRGBA(
  const ::autonomy::commsgs::std_msgs::ColorRGBA & from,
  std_msgs::msg::ColorRGBA & to);

void copyCovariance6(
  const std::array<double, 36> & from,
  std::vector<double> & to);

void copyCovariance6(
  const std::vector<double> & from,
  std::array<double, 36> & to);

}  // namespace autonomy_ros::conversions::detail

#endif  // AUTONOMY_ROS__CONVERSIONS__DETAIL_HPP_
