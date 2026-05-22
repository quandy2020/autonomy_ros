// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/planning_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"

namespace autonomy_ros::conversions
{

Odometry fromRos(const nav_msgs::msg::Odometry & from)
{
  Odometry to;
  detail::copyHeader(from.header, to.header);
  to.child_frame_id = from.child_frame_id;
  detail::copyPose(from.pose.pose, to.pose.pose);
  detail::copyCovariance6(from.pose.covariance, to.pose.covariance);
  to.twist.twist.linear.x = static_cast<float>(from.twist.twist.linear.x);
  to.twist.twist.linear.y = static_cast<float>(from.twist.twist.linear.y);
  to.twist.twist.linear.z = static_cast<float>(from.twist.twist.linear.z);
  to.twist.twist.angular.x = static_cast<float>(from.twist.twist.angular.x);
  to.twist.twist.angular.y = static_cast<float>(from.twist.twist.angular.y);
  to.twist.twist.angular.z = static_cast<float>(from.twist.twist.angular.z);
  detail::copyCovariance6(from.twist.covariance, to.twist.covariance);
  return to;
}

nav_msgs::msg::Odometry toRos(const Odometry & from)
{
  nav_msgs::msg::Odometry to;
  detail::copyHeader(from.header, to.header);
  to.child_frame_id = from.child_frame_id;
  detail::copyPose(from.pose.pose, to.pose.pose);
  detail::copyCovariance6(from.pose.covariance, to.pose.covariance);
  to.twist.twist.linear.x = from.twist.twist.linear.x;
  to.twist.twist.linear.y = from.twist.twist.linear.y;
  to.twist.twist.linear.z = from.twist.twist.linear.z;
  to.twist.twist.angular.x = from.twist.twist.angular.x;
  to.twist.twist.angular.y = from.twist.twist.angular.y;
  to.twist.twist.angular.z = from.twist.twist.angular.z;
  detail::copyCovariance6(from.twist.covariance, to.twist.covariance);
  return to;
}

nav_msgs::msg::Path toRos(const Path & from)
{
  nav_msgs::msg::Path to;
  detail::copyHeader(from.header, to.header);
  to.poses.reserve(from.poses.size());
  for (const auto & pose : from.poses) {
    to.poses.push_back(toRos(pose));
  }
  return to;
}

Path fromRos(const nav_msgs::msg::Path & from)
{
  Path to;
  detail::copyHeader(from.header, to.header);
  to.poses.reserve(from.poses.size());
  for (const auto & pose : from.poses) {
    to.poses.push_back(fromRos(pose));
  }
  return to;
}

}  // namespace autonomy_ros::conversions
