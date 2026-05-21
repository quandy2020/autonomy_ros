// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_ROS__VISUALIZATION__VISUALIZER_HPP_
#define AUTONOMY_ROS__VISUALIZATION__VISUALIZER_HPP_

#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_ros::visualization
{

/**
 * @class autonomy_ros::visualization::Visualizer
 * @brief RViz debug overlay for planned path and manual goals
 *
 * Publishes visualization_msgs/MarkerArray on visualization/markers:
 * - LINE_STRIP (green): latest /plan
 * - SPHERE (red): latest /goal_pose from RViz
 *
 * Parameter: visualization.frame_id (default odom)
 */
class Visualizer
{
public:
  /**
   * @brief Constructor for autonomy_ros::visualization::Visualizer
   * @param node Parent node used to create publishers and subscriptions
   */
  explicit Visualizer(rclcpp::Node & node);

  /**
   * @brief Subscribe to plan and goal_pose; advertise marker publisher
   */
  void start();

private:
  /**
   * @brief Cache path and republish markers
   * @param msg Planned path from Planner
   */
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Cache interactive goal from RViz
   * @param msg Pose from 2D Goal Pose tool
   */
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Assemble MarkerArray from cached path and goal
   */
  void publishMarkers();

  rclcpp::Node & node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string frame_id_{"odom"};
  std::optional<nav_msgs::msg::Path> latest_path_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
};

}  // namespace autonomy_ros::visualization

#endif  // AUTONOMY_ROS__VISUALIZATION__VISUALIZER_HPP_
