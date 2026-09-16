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

#include "autonomy_ros/node.hpp"

#include "autolink/autolink.hpp"
#include "autonomy_ros/bridge.hpp"
#include "autonomy_ros/rviz_tools.hpp"
#include "autonomy_ros/visualizer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace autonomy_ros
{

RosAutonomySystem::RosAutonomySystem(rclcpp::Node & node, Options options)
: node_(node)
, options_(std::move(options))
{
  al_node_ = autolink::CreateNode("autonomy_ros_bridge");
  if (!al_node_) {
    RCLCPP_ERROR(node_.get_logger(), "failed to create autolink node");
    return;
  }

  visualizer_ = std::make_unique<Visualizer>(
    node_, options_.navigation.visualization_frame_id);

  bridge_ = std::make_unique<AutolinkBridge>(
    node_,
    al_node_,
    options_,
    [this](const nav_msgs::msg::Path & path) {
      if (visualizer_) {
        visualizer_->OnGlobalPath(path);
      }
    },
    [this](const nav_msgs::msg::Odometry & odom) {
      if (visualizer_) {
        visualizer_->OnRobotPose(odom);
      }
    });

  rviz_tools_ = std::make_unique<RvizTools>(
    node_, al_node_, options_, visualizer_.get());

  running_ = true;
  RCLCPP_INFO(
    node_.get_logger(),
    "[autonomy_ros] process-isolated bridge ready (autolink node=%s)",
    al_node_->Name().c_str());
}

RosAutonomySystem::~RosAutonomySystem()
{
  rviz_tools_.reset();
  bridge_.reset();
  visualizer_.reset();
  al_node_.reset();
  running_ = false;
}

}  // namespace autonomy_ros
