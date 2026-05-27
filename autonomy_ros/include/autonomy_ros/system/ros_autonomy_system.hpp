// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__SYSTEM__ROS_AUTONOMY_SYSTEM_HPP_
#define AUTONOMY_ROS__SYSTEM__ROS_AUTONOMY_SYSTEM_HPP_

#include <memory>

#include "autonomy/system/autonomy.hpp"
#include "autonomy_ros/system/options.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::bridge
{
class ActuationBridge;
class CostmapBridge;
class MapBridge;
class SensorBridge;
class TfBridge;
}  // namespace autonomy_ros::bridge

namespace autonomy_ros::navigation
{
class NavigationServer;
}  // namespace autonomy_ros::navigation

namespace autonomy_ros::viz
{
class Visualizer;
}  // namespace autonomy_ros::viz

namespace autonomy_ros::debug
{
class DiagnosticsPublisher;
}  // namespace autonomy_ros::debug

namespace autonomy_ros::system
{

/**
 * @brief ROS 2 integration for autonomy::system::Autonomy.
 *
 * Wires sensor/map/costmap I/O and exposes navigation via NavigationServer.
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

  navigation::NavigationServer * navigationServer() { return navigation_server_.get(); }

private:
  void startBridges();
  void runControl();

  rclcpp::Node & node_;
  bool running_{false};

  AutonomyOptions options_;

  std::unique_ptr<viz::Visualizer> visualizer_;
  std::unique_ptr<debug::DiagnosticsPublisher> diagnostics_;
  std::unique_ptr<navigation::NavigationServer> navigation_server_;
  ::autonomy::system::Autonomy::UniquePtr core_;

  std::unique_ptr<bridge::TfBridge> tf_bridge_;
  std::unique_ptr<bridge::MapBridge> map_bridge_;
  std::unique_ptr<bridge::CostmapBridge> costmap_bridge_;
  std::unique_ptr<bridge::SensorBridge> sensor_bridge_;
  std::unique_ptr<bridge::ActuationBridge> actuation_bridge_;
};

}  // namespace autonomy_ros::system

#endif  // AUTONOMY_ROS__SYSTEM__ROS_AUTONOMY_SYSTEM_HPP_
