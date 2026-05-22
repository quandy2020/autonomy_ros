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

#ifndef AUTONOMY_ROS__AUTONOMY_NODE_HPP_
#define AUTONOMY_ROS__AUTONOMY_NODE_HPP_

#include <memory>

#include "autonomy_ros/autonomy.hpp"
#include "autonomy_ros/command/command_interface.hpp"
#include "autonomy_ros/task/task_manager.hpp"
#include "autonomy_ros/visualization/visualizer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/**
 * @class autonomy_ros::AutonomyNode
 * @brief autonomy_ros 包根节点
 *
 * 本包不实现导航算法，负责：
 * - 地图加载/更新与向 ROS 透传（MapBridge）
 * - TF 转接到 autonomy::transform::Buffer（TfBridge）
 * - 接收 RViz / 命令行指令（CommandInterface + autonomy_msgs）
 * - 仿真或真机速度输出（PlatformBridge → cmd_vel）
 * - 核心输出可视化透传（Visualizer）
 *
 * 算法与任务逻辑在 autonomy 核心，经 Autonomy 唯一门面调用。
 */
class AutonomyNode : public rclcpp::Node
{
public:
  AutonomyNode();
  ~AutonomyNode() override;

private:
  bool enable_autonomy_{true};
  bool enable_visualization_{true};
  bool enable_command_{true};

  std::unique_ptr<Autonomy> autonomy_;
  std::unique_ptr<task::TaskManager> task_manager_;
  std::unique_ptr<visualization::Visualizer> visualizer_;
  std::unique_ptr<command::CommandInterface> command_interface_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__AUTONOMY_NODE_HPP_
