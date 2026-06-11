// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// SPDX-License-Identifier: Apache-2.0

#include "autonomy_pedestrian/pedestrian_node.hpp"

#include <functional>

namespace autonomy_pedestrian
{

PedestrianNode::PedestrianNode()
: Node("pedestrian_node")
{
  loadParameters();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, qos);

  const auto period_ms = static_cast<int>(1000.0 / update_rate_hz_);
  update_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PedestrianNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "[autonomy_pedestrian] frame=%s markers=%s rate=%.1f Hz",
    frame_id_.c_str(), marker_topic_.c_str(), update_rate_hz_);
}

void PedestrianNode::loadParameters()
{
  declare_parameter<std::string>("frame_id", frame_id_);
  declare_parameter<std::string>("marker_topic", marker_topic_);
  declare_parameter<double>("update_rate_hz", update_rate_hz_);

  frame_id_ = get_parameter("frame_id").as_string();
  marker_topic_ = get_parameter("marker_topic").as_string();
  update_rate_hz_ = get_parameter("update_rate_hz").as_double();
}

void PedestrianNode::onTimer()
{
  visualization_msgs::msg::MarkerArray markers;
  marker_pub_->publish(markers);
}

}  // namespace autonomy_pedestrian

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_pedestrian::PedestrianNode>());
  rclcpp::shutdown();
  return 0;
}
