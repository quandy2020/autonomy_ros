// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/bridge/costmap_bridge.hpp"

#include <algorithm>
#include <chrono>

#include "autonomy_ros/system/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros::bridge
{

CostmapBridge::CostmapBridge(
  rclcpp::Node & node,
  ::autonomy::map::costmap_2d::Costmap2DWrapper * costmap_wrapper,
  const system::AutonomyRosOptions & ros_options)
: wrapper_(costmap_wrapper), node_(node)
{
  if (!wrapper_) {
    RCLCPP_WARN(node_.get_logger(), "[costmap_bridge] no costmap wrapper; publish disabled");
    return;
  }

  if (!ros_options.publish_costmaps) {
    RCLCPP_INFO(node_.get_logger(), "[costmap_bridge] publish_costmaps=false");
    return;
  }

  global_costmap_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
    constants::topics::kGlobalCostmap,
    rclcpp::QoS(constants::defaults::kCostmapPubDepth).transient_local());
  local_costmap_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
    constants::topics::kLocalCostmap,
    rclcpp::QoS(constants::defaults::kCostmapPubDepth).transient_local());
  const auto period_ms = static_cast<int>(
    constants::defaults::kCostmapHzToMs /
    std::max(ros_options.costmap_publish_hz, constants::defaults::kCostmapMinHz));
  publish_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&CostmapBridge::publishCostmaps, this));
  RCLCPP_INFO(
    node_.get_logger(), "[costmap_bridge] publish global/local at %.1f Hz",
    ros_options.costmap_publish_hz);
}

CostmapBridge::~CostmapBridge()
{
  publish_timer_.reset();
  global_costmap_pub_.reset();
  local_costmap_pub_.reset();
}

void CostmapBridge::publishCostmaps()
{
  if (!wrapper_) {
    return;
  }

  wrapper_->updateMap();

  ::autonomy::commsgs::map_msgs::OccupancyGrid grid;
  if (!wrapper_->snapshotOccupancyGrid(grid)) {
    return;
  }
  const auto ros_grid = conversions::toRos(grid);

  if (global_costmap_pub_) {
    global_costmap_pub_->publish(ros_grid);
  }
  if (local_costmap_pub_) {
    local_costmap_pub_->publish(ros_grid);
  }
}

}  // namespace autonomy_ros::bridge
