// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/bridge/platform_bridge.hpp"

#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros::bridge
{

PlatformBridge::PlatformBridge(rclcpp::Node & node)
: node_(node)
{
  node_.declare_parameter<std::string>("autonomy.odom_topic", odom_topic_);
  node_.declare_parameter<std::string>("autonomy.cmd_vel_topic", cmd_vel_topic_);
  node_.declare_parameter<std::string>("autonomy.controller.base_frame", base_frame_);
  node_.declare_parameter<double>("autonomy.controller.max_linear_vel", max_linear_vel_);

  odom_topic_ = node_.get_parameter("autonomy.odom_topic").as_string();
  cmd_vel_topic_ = node_.get_parameter("autonomy.cmd_vel_topic").as_string();
  base_frame_ = node_.get_parameter("autonomy.controller.base_frame").as_string();
  max_linear_vel_ = node_.get_parameter("autonomy.controller.max_linear_vel").as_double();
}

void PlatformBridge::start(
  std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)> odom_handler)
{
  odom_handler_ = std::move(odom_handler);
  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (odom_handler_) {
        odom_handler_(msg);
      }
    });
  cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic_, 10);
  RCLCPP_INFO(
    node_.get_logger(), "[platform] odom=%s cmd_vel=%s base=%s",
    odom_topic_.c_str(), cmd_vel_topic_.c_str(), base_frame_.c_str());
}

void PlatformBridge::stop()
{
  odom_sub_.reset();
  cmd_vel_pub_.reset();
  odom_handler_ = nullptr;
}

void PlatformBridge::publishCmdVel(const ::autonomy::commsgs::geometry_msgs::TwistStamped & cmd)
{
  if (!cmd_vel_pub_) {
    return;
  }
  auto ros_cmd = conversions::toRos(cmd);
  ros_cmd.header.frame_id = base_frame_;
  if (max_linear_override_ > 0.0) {
    const double cap = max_linear_override_;
    if (ros_cmd.twist.linear.x > cap) {
      ros_cmd.twist.linear.x = cap;
    }
    if (ros_cmd.twist.linear.x < -cap) {
      ros_cmd.twist.linear.x = -cap;
    }
  } else if (max_linear_vel_ > 0.0) {
    if (ros_cmd.twist.linear.x > max_linear_vel_) {
      ros_cmd.twist.linear.x = max_linear_vel_;
    }
    if (ros_cmd.twist.linear.x < -max_linear_vel_) {
      ros_cmd.twist.linear.x = -max_linear_vel_;
    }
  }
  cmd_vel_pub_->publish(ros_cmd);
}

void PlatformBridge::publishZeroCmdVel()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = node_.now();
  stop.header.frame_id = base_frame_;
  if (cmd_vel_pub_) {
    cmd_vel_pub_->publish(stop);
  }
}

void PlatformBridge::setMaxLinearVel(double max_linear)
{
  max_linear_override_ = max_linear > 0.0 ? max_linear : 0.0;
}

}  // namespace autonomy_ros::bridge
