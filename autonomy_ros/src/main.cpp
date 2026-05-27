// Copyright 2026 autonomy_ros contributors (duyongquan)
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <memory>
#include <tuple>

#include "autonomy_ros/options.hpp"
#include "autonomy_ros/ros_autonomy_system.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

void run()
{
  auto node = std::make_shared<rclcpp::Node>("autonomy_node");

  AutonomyOptions options;
  std::tie(options.core_options, options.ros_options) = createOptions(*node);

  auto system = std::make_unique<RosAutonomySystem>(
    *node, std::move(options.core_options), std::move(options.ros_options));

  RCLCPP_INFO(
    node->get_logger(),
    "autonomy_ros: core=%s",
    system->isRunning() ? "running" : "failed");

  rclcpp::spin(node);
}

}  // namespace autonomy_ros

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  autonomy_ros::run();
  rclcpp::shutdown();
  return 0;
}
