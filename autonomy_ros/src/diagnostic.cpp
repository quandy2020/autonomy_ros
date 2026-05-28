/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 * email: quandy2020@126.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy_ros/diagnostic.hpp"

#include <algorithm>
#include <chrono>

#include "autonomy_ros/constants.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace autonomy_ros
{

DiagnosticPublisher::DiagnosticPublisher(
  rclcpp::Node & node,
  SnapshotProvider snapshot_provider,
  const int publish_period_seconds)
: node_(node)
, snapshot_provider_(std::move(snapshot_provider))
{
  publisher_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    kDiagnosticsTopic, 10);
  const int period_seconds = std::max(publish_period_seconds, 1);
  timer_ = node_.create_wall_timer(
    std::chrono::seconds(period_seconds),
    std::bind(&DiagnosticPublisher::Publish, this));
  RCLCPP_INFO(
    node_.get_logger(), "[diagnostics] publishing %s every %d s",
    kDiagnosticsTopic, period_seconds);
}

void DiagnosticPublisher::Publish()
{
  if (!snapshot_provider_ || !publisher_) {
    return;
  }
  const SystemHealthSnapshot snapshot = snapshot_provider_();

  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = node_.now();

  auto append_status = [&](const char * name, uint8_t level, const char * text) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.level = level;
    status.message = text;
    message.status.push_back(status);
  };

  using diagnostic_msgs::msg::DiagnosticStatus;

  append_status(
    "autonomy_ros/core",
    snapshot.core_running ? DiagnosticStatus::OK : DiagnosticStatus::ERROR,
    snapshot.core_running ? "core running" : "core not running");

  append_status(
    "autonomy_ros/odometry",
    snapshot.odometry_received ? DiagnosticStatus::OK : DiagnosticStatus::WARN,
    snapshot.odometry_received ? "odometry available" : "no odometry");

  append_status(
    "autonomy_ros/static_map",
    snapshot.static_map_loaded ? DiagnosticStatus::OK : DiagnosticStatus::WARN,
    snapshot.static_map_loaded ? "static map loaded" : "static map missing");

  append_status(
    "autonomy_ros/controller",
    DiagnosticStatus::OK,
    snapshot.controller_enabled ? "controller enabled" : "controller idle");

  publisher_->publish(message);
}

}  // namespace autonomy_ros
