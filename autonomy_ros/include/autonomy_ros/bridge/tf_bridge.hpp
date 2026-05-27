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

#ifndef AUTONOMY_ROS__BRIDGE__TF_BRIDGE_HPP_
#define AUTONOMY_ROS__BRIDGE__TF_BRIDGE_HPP_

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "autonomy_ros/system/options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace autonomy_ros::bridge
{

/**
 * @brief Bridges ROS /tf and /tf_static into autonomy::transform::Buffer
 */
class TfBridge
{
public:
  explicit TfBridge(rclcpp::Node & node, const system::AutonomyRosOptions & ros_options);
  ~TfBridge();

private:
  void onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);
  void injectTransform(const geometry_msgs::msg::TransformStamped & tf, bool is_static);

  rclcpp::Node & node_;
  std::string tf_topic_;
  std::string tf_static_topic_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
};

}  // namespace autonomy_ros::bridge

#endif  // AUTONOMY_ROS__BRIDGE__TF_BRIDGE_HPP_
