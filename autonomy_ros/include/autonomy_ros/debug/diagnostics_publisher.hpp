// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__DEBUG__DIAGNOSTICS_PUBLISHER_HPP_
#define AUTONOMY_ROS__DEBUG__DIAGNOSTICS_PUBLISHER_HPP_

#include <functional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::debug
{

struct DiagnosticsSnapshot
{
  bool core_running{false};
  bool odometry_received{false};
  bool static_map_loaded{false};
  bool controller_enabled{false};
  bool navigation_active{false};
};

/**
 * @brief Publishes /diagnostics for autonomy_ros runtime health.
 */
class DiagnosticsPublisher
{
public:
  using SnapshotProvider = std::function<DiagnosticsSnapshot()>;

  DiagnosticsPublisher(
    rclcpp::Node & node, SnapshotProvider snapshot_provider,
    int period_sec = 1);

private:
  void publish();

  rclcpp::Node & node_;
  SnapshotProvider snapshot_provider_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy_ros::debug

#endif  // AUTONOMY_ROS__DEBUG__DIAGNOSTICS_PUBLISHER_HPP_
