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

#ifndef AUTONOMY_ROS__PLANNER__PLANNER_HPP_
#define AUTONOMY_ROS__PLANNER__PLANNER_HPP_

#include <optional>
#include <string>

#include "autonomy_ros/map/map_manager.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::planner
{

/**
 * @class autonomy_ros::planner::Planner
 * @brief Publishes a straight-line nav_msgs/Path from current pose to goal
 *
 * Inputs:
 * - odom: current robot pose in planner.global_frame (default odom)
 * - goal_pose: optional RViz goal for manual debugging
 *
 * Outputs:
 * - plan: path consumed by Controller
 *
 * Parameters: planner.waypoint_spacing, planner.global_frame
 */
class Planner
{
public:
  /**
   * @brief Constructor for autonomy_ros::planner::Planner
   * @param node Parent node used to create publishers and subscriptions
   * @param map_manager Reserved for future obstacle-aware planning
   */
  Planner(rclcpp::Node & node, map::MapManager & map_manager);

  /**
   * @brief Create odom subscription, goal_pose subscription, and plan publisher
   */
  void start();

  /**
   * @brief Interpolate from latest odom pose to goal and publish /plan
   * @param goal Target pose; header.frame_id should match global_frame
   */
  void setGoal(const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Copy of the most recently published path (for action results)
   * @return Last path if setGoal has completed at least once
   */
  std::optional<nav_msgs::msg::Path> lastPath() const;

private:
  /**
   * @brief Cache odometry; plan pending goal when odom arrives late
   * @param msg Latest /odom message
   */
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Build evenly spaced poses along a straight segment
   * @param start Start pose (robot position)
   * @param goal Goal pose
   */
  void publishPath(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);

  rclcpp::Node & node_;
  map::MapManager & map_manager_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  double waypoint_spacing_{0.1};
  std::string global_frame_{"odom"};
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  std::optional<nav_msgs::msg::Path> last_path_;
};

}  // namespace autonomy_ros::planner

#endif  // AUTONOMY_ROS__PLANNER__PLANNER_HPP_
