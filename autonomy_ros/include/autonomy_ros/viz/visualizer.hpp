// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__VIZ__VISUALIZER_HPP_
#define AUTONOMY_ROS__VIZ__VISUALIZER_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::viz
{

/**
 * @brief Navigation visualization: global plan and robot pose for RViz.
 *
 * Map and costmaps are published by MapBridge / CostmapBridge on operational
 * topics (/map, /global_costmap, /local_costmap).
 */
class Visualizer
{
public:
  explicit Visualizer(rclcpp::Node & node);

  void onGlobalPath(const nav_msgs::msg::Path & path);
  void onNavigationGoal(const geometry_msgs::msg::PoseStamped & goal);
  void onRobotPose(const nav_msgs::msg::Odometry & odom);

private:
  rclcpp::Node & node_;
  std::string frame_id_{"odom"};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_publisher_;
};

}  // namespace autonomy_ros::viz

#endif  // AUTONOMY_ROS__VIZ__VISUALIZER_HPP_
