/*
 * Copyright 2026 autonomy_ros contributors
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

/**
 * @file
 * @brief RViz visualization for MPPI candidate and optimal trajectories.
 */

#ifndef AUTONOMY_CONTROLLER_MPPI_TRAJECTORY_VIZ_HPP_
#define AUTONOMY_CONTROLLER_MPPI_TRAJECTORY_VIZ_HPP_

#include <memory>
#include <string>

#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_controller
{

/**
 * @class MppiTrajectoryVisualizer
 * @brief Publishes MPPI rollout LINE_STRIP markers and the optimal path.
 */
class MppiTrajectoryVisualizer
{
public:
  /**
   * @brief Wire publishers and visualization limits.
   * @param node Parent node used for publisher creation.
   * @param frame_id TF frame for marker/path headers.
   * @param max_candidates Max candidate LINE_STRIP count in RViz.
   * @param line_width Candidate trajectory line width (m).
   */
  void Configure(
    rclcpp::Node * node,
    const std::string & frame_id,
    int max_candidates,
    double line_width);

  /**
   * @brief Publish visualization when subscribers are connected.
   * @param mppi Active MPPI controller (non-owning).
   * @param stamp Message stamp shared by markers and path.
   */
  void Publish(
    autonomy::control::controller::mppi_controller::MPPIController * mppi,
    const rclcpp::Time & stamp);

private:
  std::string frame_id_;
  int max_candidates_{40};
  double line_width_{0.008};
  int published_candidates_{0};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    candidates_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;
};

}  // namespace autonomy_controller

#endif  // AUTONOMY_CONTROLLER_MPPI_TRAJECTORY_VIZ_HPP_
