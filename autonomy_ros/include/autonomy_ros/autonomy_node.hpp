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

#include "autonomy_ros/command/command_interface.hpp"
#include "autonomy_ros/controller/controller.hpp"
#include "autonomy_ros/map/map_manager.hpp"
#include "autonomy_ros/planner/planner.hpp"
#include "autonomy_ros/task/task_manager.hpp"
#include "autonomy_ros/visualization/visualizer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/**
 * @class autonomy_ros::AutonomyNode
 * @brief Root ROS 2 node that composes the exhibition-robot autonomy stack
 *
 * Wires optional submodules (map, planner, controller, visualization, command)
 * according to boolean parameters declared on the node. TaskManager is always
 * constructed so status and events are available when command is disabled.
 *
 * Parameters (see config/autonomy_params.yaml):
 * - enable_map, enable_planner, enable_controller, enable_visualization, enable_command
 * - use_sim_time
 *
 * Command requires both planner and controller; otherwise it is not started.
 */
class AutonomyNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct AutonomyNode and initialize enabled submodules from parameters
   */
  AutonomyNode();

private:
  /** @brief Subscribe to /map when true */
  bool enable_map_{true};
  /** @brief Run straight-line planner when true */
  bool enable_planner_{true};
  /** @brief Run path-tracking controller when true */
  bool enable_controller_{true};
  /** @brief Publish RViz markers for plan/goal when true */
  bool enable_visualization_{true};
  /** @brief Host autonomy_msgs action/service servers when true */
  bool enable_command_{true};

  std::unique_ptr<task::TaskManager> task_manager_;
  std::unique_ptr<map::MapManager> map_manager_;
  std::unique_ptr<planner::Planner> planner_;
  std::unique_ptr<controller::Controller> controller_;
  std::unique_ptr<visualization::Visualizer> visualizer_;
  std::unique_ptr<command::CommandInterface> command_interface_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__AUTONOMY_NODE_HPP_
