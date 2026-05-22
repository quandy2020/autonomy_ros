// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__TRAJECTORY_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__TRAJECTORY_MSGS_HPP_

/// @file trajectory_msgs.hpp
/// @brief Converts ROS 2 trajectory_msgs and autonomy::commsgs::trajectory_msgs.
///
/// Proto schema: autonomy/commsgs/proto/trajectory_msgs.proto
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) for arm/manipulator
/// trajectory I/O. Each type has paired fromRos() / toRos() overloads.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/trajectory_msgs.hpp"
/// auto traj = autonomy_ros::conversions::fromRos(*ros_joint_traj);
/// traj_pub_->publish(autonomy_ros::conversions::toRos(core_traj));
/// @endcode

#include "autonomy/commsgs/trajectory_msgs.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"

namespace autonomy_ros::conversions
{

using JointTrajectoryPoint = ::autonomy::commsgs::trajectory_msgs::JointTrajectoryPoint;
using JointTrajectory = ::autonomy::commsgs::trajectory_msgs::JointTrajectory;
using MultiDOFJointTrajectoryPoint =
  ::autonomy::commsgs::trajectory_msgs::MultiDOFJointTrajectoryPoint;
using MultiDOFJointTrajectory =
  ::autonomy::commsgs::trajectory_msgs::MultiDOFJointTrajectory;

/**
 * @brief Bidirectional conversion between ROS trajectory_msgs::msg::JointTrajectoryPoint and commsgs JointTrajectoryPoint.
 *
 * @par fromRos
 * @param from Input ROS message (trajectory_msgs::msg::JointTrajectoryPoint). Fields are copied without coordinate transforms.
 * @return commsgs JointTrajectoryPoint for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs JointTrajectoryPoint from planners, bridges, or drivers.
 * @return ROS trajectory_msgs::msg::JointTrajectoryPoint ready for rclcpp publish() or subscribe() adapters.
 */
JointTrajectoryPoint fromRos(const trajectory_msgs::msg::JointTrajectoryPoint & from);
trajectory_msgs::msg::JointTrajectoryPoint toRos(const JointTrajectoryPoint & from);

/**
 * @brief Bidirectional conversion between ROS trajectory_msgs::msg::JointTrajectory and commsgs JointTrajectory.
 *
 * @par fromRos
 * @param from Input ROS message (trajectory_msgs::msg::JointTrajectory). Fields are copied without coordinate transforms.
 * @return commsgs JointTrajectory for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs JointTrajectory from planners, bridges, or drivers.
 * @return ROS trajectory_msgs::msg::JointTrajectory ready for rclcpp publish() or subscribe() adapters.
 */
JointTrajectory fromRos(const trajectory_msgs::msg::JointTrajectory & from);
trajectory_msgs::msg::JointTrajectory toRos(const JointTrajectory & from);

/**
 * @brief Bidirectional conversion between ROS trajectory_msgs::msg::MultiDOFJointTrajectoryPoint and commsgs MultiDOFJointTrajectoryPoint.
 *
 * @par fromRos
 * @param from Input ROS message (trajectory_msgs::msg::MultiDOFJointTrajectoryPoint). Fields are copied without coordinate transforms.
 * @return commsgs MultiDOFJointTrajectoryPoint for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MultiDOFJointTrajectoryPoint from planners, bridges, or drivers.
 * @return ROS trajectory_msgs::msg::MultiDOFJointTrajectoryPoint ready for rclcpp publish() or subscribe() adapters.
 */
MultiDOFJointTrajectoryPoint fromRos(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & from);
trajectory_msgs::msg::MultiDOFJointTrajectoryPoint toRos(const MultiDOFJointTrajectoryPoint & from);

/**
 * @brief Bidirectional conversion between ROS trajectory_msgs::msg::MultiDOFJointTrajectory and commsgs MultiDOFJointTrajectory.
 *
 * @par fromRos
 * @param from Input ROS message (trajectory_msgs::msg::MultiDOFJointTrajectory). Fields are copied without coordinate transforms.
 * @return commsgs MultiDOFJointTrajectory for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MultiDOFJointTrajectory from planners, bridges, or drivers.
 * @return ROS trajectory_msgs::msg::MultiDOFJointTrajectory ready for rclcpp publish() or subscribe() adapters.
 */
MultiDOFJointTrajectory fromRos(const trajectory_msgs::msg::MultiDOFJointTrajectory & from);
trajectory_msgs::msg::MultiDOFJointTrajectory toRos(const MultiDOFJointTrajectory & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__TRAJECTORY_MSGS_HPP_
