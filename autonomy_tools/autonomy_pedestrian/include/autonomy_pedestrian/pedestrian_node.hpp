// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// SPDX-License-Identifier: Apache-2.0

#ifndef AUTONOMY_PEDESTRIAN__PEDESTRIAN_NODE_HPP_
#define AUTONOMY_PEDESTRIAN__PEDESTRIAN_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_pedestrian
{

/**
 * @brief Pedestrian simulation node for autonomy_ros tools.
 */
class PedestrianNode : public rclcpp::Node
{
public:
  PedestrianNode();

private:
  void loadParameters();
  void onTimer();

  std::string frame_id_;
  std::string marker_topic_;
  double update_rate_hz_{10.0};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace autonomy_pedestrian

#endif  // AUTONOMY_PEDESTRIAN__PEDESTRIAN_NODE_HPP_
