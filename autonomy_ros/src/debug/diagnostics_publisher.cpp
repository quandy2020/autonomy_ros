// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/debug/diagnostics_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "autonomy_ros/system/constants.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace autonomy_ros::debug
{

DiagnosticsPublisher::DiagnosticsPublisher(
  rclcpp::Node & node, SnapshotProvider snapshot_provider, const int period_sec)
: node_(node), snapshot_provider_(std::move(snapshot_provider))
{
  pub_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    constants::topics::kDiagnostics, constants::defaults::kQueueDepth);
  const int period = std::max(period_sec, 1);
  timer_ = node_.create_wall_timer(
    std::chrono::seconds(period), std::bind(&DiagnosticsPublisher::publish, this));
  RCLCPP_INFO(node_.get_logger(), "[diagnostics] publishing %s every %ds",
    constants::topics::kDiagnostics, period);
}

void DiagnosticsPublisher::publish()
{
  if (!snapshot_provider_ || !pub_) {
    return;
  }
  const DiagnosticsSnapshot snap = snapshot_provider_();

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_.now();

  auto add_status = [&](const char * name, uint8_t level, const char * message) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.level = level;
    status.message = message;
    array.status.push_back(status);
  };

  add_status(
    constants::msg::kDiagCore,
    snap.core_running ? diagnostic_msgs::msg::DiagnosticStatus::OK
                      : diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    snap.core_running ? constants::msg::kDiagCoreRunning
                      : constants::msg::kDiagCoreNotRunning);

  add_status(
    constants::msg::kDiagOdom,
    snap.odometry_received ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN,
    snap.odometry_received ? constants::msg::kDiagOdomAvailable
                           : constants::msg::kDiagOdomMissing);

  add_status(
    constants::msg::kDiagMap,
    snap.static_map_loaded ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN,
    snap.static_map_loaded ? constants::msg::kDiagMapLoaded
                           : constants::msg::kDiagMapMissing);

  add_status(
    constants::msg::kDiagController,
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    snap.controller_enabled ? constants::msg::kDiagControllerEnabled
                            : constants::msg::kDiagControllerIdle);

  pub_->publish(array);
}

}  // namespace autonomy_ros::debug
