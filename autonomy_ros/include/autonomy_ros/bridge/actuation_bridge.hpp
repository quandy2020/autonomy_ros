// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__BRIDGE__ACTUATION_BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE__ACTUATION_BRIDGE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy_ros/options.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::bridge
{

/**
 * @brief ROS egress for cmd_vel and optional periodic control ticks (e.g. TickFollowPath).
 */
class ActuationBridge
{
public:
  ActuationBridge(
    rclcpp::Node & node, const AutonomyRosOptions & ros_options,
    const AutonomyCoreOptions & core_options);
  ~ActuationBridge();

  void publish(const ::autonomy::commsgs::geometry_msgs::TwistStamped & cmd);
  void publishZero();

  /** @brief Wall timer invoking @p tick at @p period_ms until destroyed or reset. */
  void startControlLoop(int period_ms, std::function<void()> tick);
  void stopControlLoop();

private:
  rclcpp::Node & node_;
  std::string base_frame_;
  double max_linear_vel_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace autonomy_ros::bridge

#endif  // AUTONOMY_ROS__BRIDGE__ACTUATION_BRIDGE_HPP_
