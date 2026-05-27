// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__BRIDGE__COSTMAP_BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE__COSTMAP_BRIDGE_HPP_

#include <memory>

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy_ros/system/options.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::bridge
{

/**
 * @brief Publishes planner costmap snapshots to ROS (/global_costmap, /local_costmap).
 *
 * Sensor and map updates are handled inside autonomy core (SensorCollator / MapServer).
 */
class CostmapBridge
{
public:
  CostmapBridge(
    rclcpp::Node & node,
    ::autonomy::map::costmap_2d::Costmap2DWrapper * costmap_wrapper,
    const system::AutonomyRosOptions & ros_options);
  ~CostmapBridge();

private:
  void publishCostmaps();

  ::autonomy::map::costmap_2d::Costmap2DWrapper * wrapper_{nullptr};
  rclcpp::Node & node_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace autonomy_ros::bridge

#endif  // AUTONOMY_ROS__BRIDGE__COSTMAP_BRIDGE_HPP_
