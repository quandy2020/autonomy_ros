/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_ROS__VISUALIZER_HPP_
#define AUTONOMY_ROS__VISUALIZER_HPP_

#include <string>

#include "autonomy_ros/constants.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/** @brief Publishes plan, navigation goal, and robot pose for RViz. */
class Visualizer
{
public:
  Visualizer(rclcpp::Node & node, std::string frame_id);

  void OnGlobalPath(const nav_msgs::msg::Path & path);
  void OnNavigationGoal(const geometry_msgs::msg::PoseStamped & goal);
  void OnRobotPose(const nav_msgs::msg::Odometry & odom);

private:
  rclcpp::Node & node_;
  std::string frame_id_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_publisher_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__VISUALIZER_HPP_
