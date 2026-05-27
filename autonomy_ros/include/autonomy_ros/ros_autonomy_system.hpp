// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__ROS_AUTONOMY_SYSTEM_HPP_
#define AUTONOMY_ROS__ROS_AUTONOMY_SYSTEM_HPP_

#include <memory>

#include "autonomy/system/autonomy.hpp"
#include "autonomy_ros/options.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

namespace bridge
{
class ActuationBridge;
class CostmapBridge;
class MapBridge;
class SensorBridge;
class TfBridge;
}  // namespace bridge

namespace command
{
class CommandInterface;
}

namespace debug
{
class DiagnosticsPublisher;
}

namespace visualization
{
class Visualizer;
}

/**
 * @brief ROS 2 integration for autonomy::system::Autonomy.
 *
 * Bridges sensors, maps, costmaps, and cmd_vel; exposes navigation via
 * CommandInterface. Core logic stays in the autonomy library.
 */
class RosAutonomySystem
{
public:
  RosAutonomySystem(
    rclcpp::Node & node,
    AutonomyCoreOptions core_options,
    AutonomyRosOptions ros_options);

  ~RosAutonomySystem();

  bool isRunning() const { return running_; }

  command::CommandInterface * commandInterface() { return command_interface_.get(); }

private:
  void startBridges();
  void runControl();

  rclcpp::Node & node_;
  bool running_{false};

  AutonomyOptions options_;

  std::unique_ptr<visualization::Visualizer> visualizer_;
  std::unique_ptr<debug::DiagnosticsPublisher> diagnostics_;
  std::unique_ptr<command::CommandInterface> command_interface_;
  ::autonomy::system::Autonomy::UniquePtr core_;

  std::unique_ptr<bridge::TfBridge> tf_bridge_;
  std::unique_ptr<bridge::MapBridge> map_bridge_;
  std::unique_ptr<bridge::CostmapBridge> costmap_bridge_;
  std::unique_ptr<bridge::SensorBridge> sensor_bridge_;
  std::unique_ptr<bridge::ActuationBridge> actuation_bridge_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__ROS_AUTONOMY_SYSTEM_HPP_
