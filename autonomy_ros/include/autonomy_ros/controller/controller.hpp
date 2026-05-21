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

#ifndef AUTONOMY_ROS__CONTROLLER__CONTROLLER_HPP_
#define AUTONOMY_ROS__CONTROLLER__CONTROLLER_HPP_

#include <atomic>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::controller
{

/**
 * @class autonomy_ros::controller::Controller
 * @brief Simple lookahead path tracker publishing geometry_msgs/TwistStamped on cmd_vel
 *
 * When disabled, publishes zero velocity (pause, estop, teleop handover).
 * CommandInterface may lower max linear speed via setMaxLinearVel for tour cruise
 * or NavigatePose max_speed goals.
 *
 * Parameters: controller.max_linear_vel, max_angular_vel, goal_tolerance, lookahead,
 * base_frame
 */
class Controller
{
public:
  /**
   * @brief Constructor for autonomy_ros::controller::Controller
   * @param node Parent node used to create publishers, subscriptions and timers
   */
  explicit Controller(rclcpp::Node & node);

  /**
   * @brief Subscribe to plan and odom; start 20 Hz control timer
   */
  void start();

  /**
   * @brief Enable or disable autonomous velocity output
   * @param enabled When false, cmd_vel is zeroed each control cycle
   */
  void setEnabled(bool enabled);

  /**
   * @brief Whether autonomous commands are published
   * @return True if enabled
   */
  bool enabled() const;

  /**
   * @brief Distance to final path pose considered "goal reached"
   * @return Tolerance in meters (parameter controller.goal_tolerance)
   */
  double goalTolerance() const;

  /**
   * @brief Override linear speed cap for current task
   * @param max_linear Positive value in m/s; 0 restores parameter default
   */
  void setMaxLinearVel(double max_linear);

  /**
   * @brief Linear speed from ROS parameter at startup
   * @return controller.max_linear_vel
   */
  double defaultMaxLinearVel() const;

private:
  /**
   * @brief Reset path index when a new plan arrives
   * @param msg Path from Planner on topic plan
   */
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Update pose used for tracking
   * @param msg Latest /odom
   */
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Compute velocity toward lookahead point; stop at goal tolerance
   */
  void controlStep();

  rclcpp::Node & node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_linear_vel_{0.22};
  double default_max_linear_vel_{0.22};
  double max_angular_vel_{1.5};
  double goal_tolerance_{0.15};
  double lookahead_{0.3};
  std::string base_frame_{"base_footprint"};

  std::atomic<bool> enabled_{true};

  std::optional<nav_msgs::msg::Path> active_path_;
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  size_t path_index_{0};
};

}  // namespace autonomy_ros::controller

#endif  // AUTONOMY_ROS__CONTROLLER__CONTROLLER_HPP_
