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

#include "autonomy_ros/autonomy_node.hpp"

#include "nav_msgs/msg/path.hpp"

namespace autonomy_ros
{

AutonomyNode::AutonomyNode()
: Node("autonomy_node")
{
  declare_parameter<bool>("enable_autonomy", enable_autonomy_);
  declare_parameter<bool>("enable_visualization", enable_visualization_);
  declare_parameter<bool>("enable_command", enable_command_);

  enable_autonomy_ = get_parameter("enable_autonomy").as_bool();
  enable_visualization_ = get_parameter("enable_visualization").as_bool();
  enable_command_ = get_parameter("enable_command").as_bool();

  task_manager_ = std::make_unique<task::TaskManager>(*this);
  task_manager_->start();

  if (enable_autonomy_) {
    autonomy_ = std::make_unique<Autonomy>(*this);
    autonomy_->start();
  }

  if (enable_visualization_) {
    visualizer_ = std::make_unique<visualization::Visualizer>(*this);
    visualizer_->start();
  }

  if (enable_autonomy_ && autonomy_ && autonomy_->isRunning()) {
    if (visualizer_) {
      autonomy_->addPlanListener(
        [this](const nav_msgs::msg::Path & path) { visualizer_->onPlan(path); });
      autonomy_->addOdomListener(
        [this](const nav_msgs::msg::Odometry::SharedPtr & msg) {
          if (msg) {
            visualizer_->onRobotPose(*msg);
          }
        });
    }
  }

  if (enable_command_) {
    if (!autonomy_ || !autonomy_->isRunning()) {
      RCLCPP_WARN(
        get_logger(),
        "enable_command requires enable_autonomy and a running core; command disabled");
    } else {
      command_interface_ = std::make_unique<command::CommandInterface>(
        *this, *task_manager_, *autonomy_);
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "autonomy_ros: core=%s visualization=%s command=%s",
    (autonomy_ && autonomy_->isRunning()) ? "on" : "off",
    enable_visualization_ ? "on" : "off",
    (enable_command_ && command_interface_) ? "on" : "off");
}

void AutonomyNode::startCommandInterface()
{
  if (command_interface_) {
    command_interface_->start();
  }
}

AutonomyNode::~AutonomyNode()
{
  if (command_interface_) {
    command_interface_.reset();
  }
  if (visualizer_) {
    visualizer_.reset();
  }
  if (autonomy_) {
    autonomy_->shutdown();
    autonomy_.reset();
  }
  if (task_manager_) {
    task_manager_.reset();
  }
}

}  // namespace autonomy_ros
