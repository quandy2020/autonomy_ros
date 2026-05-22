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

#ifndef AUTONOMY_ROS__BRIDGE__PLATFORM_BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE__PLATFORM_BRIDGE_HPP_

#include <functional>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::bridge
{

/**
 * @brief Robot platform I/O: odom in, cmd_vel out (Gazebo bridge or hardware driver)
 */
class PlatformBridge
{
public:
  explicit PlatformBridge(rclcpp::Node & node);

  void start(std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)> odom_handler);
  void stop();

  void publishCmdVel(const ::autonomy::commsgs::geometry_msgs::TwistStamped & cmd);
  void publishZeroCmdVel();
  void setMaxLinearVel(double max_linear);

  const std::string & cmdVelTopic() const { return cmd_vel_topic_; }
  const std::string & odomTopic() const { return odom_topic_; }
  const std::string & baseFrame() const { return base_frame_; }

private:
  rclcpp::Node & node_;
  std::string odom_topic_{"odom"};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string base_frame_{"base_footprint"};
  double max_linear_vel_{0.22};
  double max_linear_override_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)> odom_handler_;
};

}  // namespace autonomy_ros::bridge

#endif  // AUTONOMY_ROS__BRIDGE__PLATFORM_BRIDGE_HPP_
