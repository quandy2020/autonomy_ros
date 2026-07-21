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
 * @brief Utility helpers for controller simulation (options, costmap, TF).
 */

#ifndef AUTONOMY_CONTROLLER_CONTROLLER_SIM_UTILS_HPP_
#define AUTONOMY_CONTROLLER_CONTROLLER_SIM_UTILS_HPP_

#include <string>

#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "nav_msgs/msg/odometry.hpp"

namespace autonomy_controller
{

/**
 * @brief Load controller options from control/controller.lua (+ common.lua).
 * @param configuration_directory Override config root; empty uses install share.
 */
autonomy::control::proto::ControllerOptions LoadControllerOptionsBundle(
  const std::string & configuration_directory);

/**
 * @brief Build rolling Costmap2DOptions for controller sim.
 */
autonomy::map::proto::Costmap2DOptions MakeSimCostmapOptions(
  const std::string & frame_id,
  const std::string & cloud_topic,
  double resolution,
  double width,
  double height,
  double update_frequency,
  double robot_radius,
  double inflation_radius,
  double inflation_cost_scaling_factor,
  double obstacle_min_height,
  double obstacle_max_height,
  double raytrace_max_range);

/**
 * @brief Push odom->base transform into the autonomy TF buffer.
 */
void PublishOdomTf(
  const nav_msgs::msg::Odometry & odom,
  const std::string & default_parent,
  const std::string & default_child);

}  // namespace autonomy_controller

#endif  // AUTONOMY_CONTROLLER_CONTROLLER_SIM_UTILS_HPP_
