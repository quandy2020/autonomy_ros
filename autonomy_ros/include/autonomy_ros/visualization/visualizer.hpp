// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__VISUALIZATION__VISUALIZER_HPP_
#define AUTONOMY_ROS__VISUALIZATION__VISUALIZER_HPP_

#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_ros::visualization
{

/**
 * @brief RViz markers driven by Autonomy callbacks (no duplicate /odom or /plan subs).
 */
class Visualizer
{
public:
  explicit Visualizer(rclcpp::Node & node);

  void start();

  void onPlan(const nav_msgs::msg::Path & path);
  void onRobotPose(const nav_msgs::msg::Odometry & odom);

private:
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishMarkers();

  rclcpp::Node & node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string frame_id_{"odom"};
  std::optional<nav_msgs::msg::Path> latest_path_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_robot_pose_;
};

}  // namespace autonomy_ros::visualization

#endif  // AUTONOMY_ROS__VISUALIZATION__VISUALIZER_HPP_
