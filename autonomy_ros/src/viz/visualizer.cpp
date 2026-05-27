// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/viz/visualizer.hpp"

#include "autonomy_ros/system/constants.hpp"

namespace autonomy_ros::viz
{

Visualizer::Visualizer(rclcpp::Node & node)
: node_(node)
{
  node_.declare_parameter("visualization.frame_id", frame_id_);
  frame_id_ = node_.get_parameter("visualization.frame_id").as_string();

  plan_publisher_ = node_.create_publisher<nav_msgs::msg::Path>(
    constants::topics::kPlan, constants::defaults::kQueueDepth);
  goal_publisher_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    constants::topics::kNavigationGoal, constants::defaults::kQueueDepth);
  robot_pose_publisher_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    constants::topics::kRobotPose, constants::defaults::kQueueDepth);

  RCLCPP_INFO(
    node_.get_logger(),
    "[visualization] plan=%s goal=%s robot_pose=%s (frame=%s)",
    constants::topics::kPlan, constants::topics::kNavigationGoal,
    constants::topics::kRobotPose, frame_id_.c_str());
}

void Visualizer::onGlobalPath(const nav_msgs::msg::Path & path)
{
  if (plan_publisher_) {
    plan_publisher_->publish(path);
  }
}

void Visualizer::onNavigationGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (!goal_publisher_) {
    return;
  }
  geometry_msgs::msg::PoseStamped stamped = goal;
  if (stamped.header.frame_id.empty()) {
    stamped.header.frame_id = frame_id_;
  }
  if (stamped.header.stamp.sec == 0 && stamped.header.stamp.nanosec == 0) {
    stamped.header.stamp = node_.now();
  }
  goal_publisher_->publish(stamped);
}

void Visualizer::onRobotPose(const nav_msgs::msg::Odometry & odom)
{
  if (!robot_pose_publisher_) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = frame_id_;
  }
  pose.pose = odom.pose.pose;
  robot_pose_publisher_->publish(pose);
}

}  // namespace autonomy_ros::viz
