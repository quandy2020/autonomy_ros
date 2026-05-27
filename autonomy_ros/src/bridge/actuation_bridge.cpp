// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/bridge/actuation_bridge.hpp"

#include "autonomy_ros/system/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros::bridge
{

ActuationBridge::ActuationBridge(
  rclcpp::Node & node, const system::AutonomyRosOptions & ros_options,
  const system::AutonomyCoreOptions & core_options)
: node_(node),
  base_frame_(core_options.base_frame),
  max_linear_vel_(core_options.max_linear_vel)
{
  cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
    ros_options.cmd_vel_topic, constants::defaults::kQueueDepth);
  RCLCPP_INFO(
    node_.get_logger(), "[actuation_bridge] cmd_vel=%s base=%s",
    ros_options.cmd_vel_topic.c_str(), base_frame_.c_str());
}

ActuationBridge::~ActuationBridge()
{
  stopControlLoop();
  cmd_vel_pub_.reset();
}

void ActuationBridge::startControlLoop(int period_ms, std::function<void()> tick)
{
  stopControlLoop();
  if (!tick || period_ms <= 0) {
    return;
  }
  control_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(period_ms), std::move(tick));
}

void ActuationBridge::stopControlLoop()
{
  control_timer_.reset();
}

void ActuationBridge::publish(
  const ::autonomy::commsgs::geometry_msgs::TwistStamped & cmd)
{
  if (!cmd_vel_pub_) {
    return;
  }
  auto ros_cmd = conversions::toRos(cmd);
  ros_cmd.header.frame_id = base_frame_;
  if (max_linear_vel_ > 0.0) {
    if (ros_cmd.twist.linear.x > max_linear_vel_) {
      ros_cmd.twist.linear.x = max_linear_vel_;
    }
    if (ros_cmd.twist.linear.x < -max_linear_vel_) {
      ros_cmd.twist.linear.x = -max_linear_vel_;
    }
  }
  cmd_vel_pub_->publish(ros_cmd);
}

void ActuationBridge::publishZero()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = node_.now();
  stop.header.frame_id = base_frame_;
  cmd_vel_pub_->publish(stop);
}

}  // namespace autonomy_ros::bridge
