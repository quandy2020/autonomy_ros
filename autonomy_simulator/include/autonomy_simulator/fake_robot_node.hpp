// Copyright 2026 autonomy_ros contributors
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
//
// Differential-drive fake robot adapted from ROBOTIS turtlebot3_fake_node:
// https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_fake_node

#ifndef AUTONOMY_SIMULATOR__FAKE_ROBOT_NODE_HPP_
#define AUTONOMY_SIMULATOR__FAKE_ROBOT_NODE_HPP_

#include <array>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace autonomy_simulator
{

/**
 * @brief Lightweight diff-drive simulator: cmd_vel in, odom / joint_states / tf out.
 *
 * No Gazebo required; suitable for RViz and autonomy_ros stack bring-up tests.
 */
class FakeRobotNode : public rclcpp::Node
{
public:
  FakeRobotNode();

private:
  static constexpr std::size_t kLeft = 0;
  static constexpr std::size_t kRight = 1;

  void loadParameters();
  void initState();
  void onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onUpdate();

  bool integrateOdometry(const rclcpp::Duration & dt);
  void publishJointStates(const rclcpp::Time & stamp);
  void publishTf(const rclcpp::Time & stamp);

  std::string odom_topic_{"odom"};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string joint_states_topic_{"joint_states"};
  std::string tf_topic_{"tf"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::string joint_states_frame_{"base_footprint"};

  double wheel_separation_{0.287};
  double wheel_radius_{0.033};
  double cmd_vel_timeout_{1.0};
  double update_rate_hz_{100.0};

  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_update_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_states_;

  std::array<double, 2> wheel_speed_cmd_{0.0, 0.0};
  std::array<double, 2> last_wheel_pos_{0.0, 0.0};
  std::array<double, 2> last_wheel_vel_{0.0, 0.0};
  double goal_linear_{0.0};
  double goal_angular_{0.0};
  std::array<float, 3> pose_{0.0F, 0.0F, 0.0F};
  std::array<float, 3> vel_{0.0F, 0.0F, 0.0F};
};

}  // namespace autonomy_simulator

#endif  // AUTONOMY_SIMULATOR__FAKE_ROBOT_NODE_HPP_
