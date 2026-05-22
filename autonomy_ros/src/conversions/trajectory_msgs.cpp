// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/trajectory_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"

namespace autonomy_ros::conversions
{

JointTrajectoryPoint fromRos(const trajectory_msgs::msg::JointTrajectoryPoint & from)
{
  JointTrajectoryPoint to;
  to.positions = from.positions;
  to.velocities = from.velocities;
  to.accelerations = from.accelerations;
  to.effort = from.effort;
  detail::copyDuration(from.time_from_start, to.time_from_start);
  return to;
}

trajectory_msgs::msg::JointTrajectoryPoint toRos(const JointTrajectoryPoint & from)
{
  trajectory_msgs::msg::JointTrajectoryPoint to;
  to.positions = from.positions;
  to.velocities = from.velocities;
  to.accelerations = from.accelerations;
  to.effort = from.effort;
  detail::copyDuration(from.time_from_start, to.time_from_start);
  return to;
}

JointTrajectory fromRos(const trajectory_msgs::msg::JointTrajectory & from)
{
  JointTrajectory to;
  detail::copyHeader(from.header, to.header);
  to.joint_names = from.joint_names;
  to.points.reserve(from.points.size());
  for (const auto & pt : from.points) {
    to.points.push_back(fromRos(pt));
  }
  return to;
}

trajectory_msgs::msg::JointTrajectory toRos(const JointTrajectory & from)
{
  trajectory_msgs::msg::JointTrajectory to;
  detail::copyHeader(from.header, to.header);
  to.joint_names = from.joint_names;
  to.points.reserve(from.points.size());
  for (const auto & pt : from.points) {
    to.points.push_back(toRos(pt));
  }
  return to;
}

MultiDOFJointTrajectoryPoint fromRos(
  const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & from)
{
  MultiDOFJointTrajectoryPoint to;
  to.transforms.reserve(from.transforms.size());
  for (const auto & t : from.transforms) {
    to.transforms.push_back(fromRos(t));
  }
  to.velocities.reserve(from.velocities.size());
  for (const auto & v : from.velocities) {
    to.velocities.push_back(fromRos(v));
  }
  to.accelerations.reserve(from.accelerations.size());
  for (const auto & a : from.accelerations) {
    to.accelerations.push_back(fromRos(a));
  }
  detail::copyDuration(from.time_from_start, to.time_from_start);
  return to;
}

trajectory_msgs::msg::MultiDOFJointTrajectoryPoint toRos(
  const MultiDOFJointTrajectoryPoint & from)
{
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint to;
  to.transforms.reserve(from.transforms.size());
  for (const auto & t : from.transforms) {
    to.transforms.push_back(toRos(t));
  }
  to.velocities.reserve(from.velocities.size());
  for (const auto & v : from.velocities) {
    to.velocities.push_back(toRos(v));
  }
  to.accelerations.reserve(from.accelerations.size());
  for (const auto & a : from.accelerations) {
    to.accelerations.push_back(toRos(a));
  }
  detail::copyDuration(from.time_from_start, to.time_from_start);
  return to;
}

MultiDOFJointTrajectory fromRos(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & from)
{
  MultiDOFJointTrajectory to;
  detail::copyHeader(from.header, to.header);
  to.joint_names = from.joint_names;
  to.points.reserve(from.points.size());
  for (const auto & pt : from.points) {
    to.points.push_back(fromRos(pt));
  }
  return to;
}

trajectory_msgs::msg::MultiDOFJointTrajectory toRos(
  const MultiDOFJointTrajectory & from)
{
  trajectory_msgs::msg::MultiDOFJointTrajectory to;
  detail::copyHeader(from.header, to.header);
  to.joint_names = from.joint_names;
  to.points.reserve(from.points.size());
  for (const auto & pt : from.points) {
    to.points.push_back(toRos(pt));
  }
  return to;
}

}  // namespace autonomy_ros::conversions
