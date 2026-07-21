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
 * @brief Utility helpers for planner simulation (map path, costmap, TF).
 */

#ifndef AUTONOMY_PLANNER_PLANNER_SIM_UTILS_HPP_
#define AUTONOMY_PLANNER_PLANNER_SIM_UTILS_HPP_

#include <string>

#include "autonomy/map/proto/map_2d_option.pb.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace autonomy_planner {

/**
 * @brief Resolve a map YAML path under autonomy config/data or config/map.
 * @param map_file Basename or absolute path from ROS parameter.
 * @return Absolute path when found; empty string otherwise.
 */
std::string ResolveMapYamlPath(const std::string & map_file);

/**
 * @brief Build Costmap2DOptions for planner sim (static + obstacle + inflation).
 */
autonomy::map::proto::Costmap2DOptions MakeSimCostmapOptions(
  const std::string & frame_id,
  const std::string & cloud_topic,
  const std::string & map_topic,
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
 * @brief Human-readable name for autonomy PlannerResultCode values.
 * @param code Numeric planner result code.
 */
const char * PlannerResultName(uint32_t code);

/**
 * @brief Push odom->base transform into the autonomy TF buffer.
 * @param odom Latest odometry message.
 * @param default_parent Frame when odom.header.frame_id is empty.
 * @param default_child Frame when odom.child_frame_id is empty.
 */
void PublishOdomTf(
  const nav_msgs::msg::Odometry & odom,
  const std::string & default_parent,
  const std::string & default_child);

/**
 * @brief Extract planar yaw (rad) from a unit quaternion.
 * @param q Orientation quaternion (z/w used for 2-D heading).
 */
double YawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

}  // namespace autonomy_planner

#endif  // AUTONOMY_PLANNER_PLANNER_SIM_UTILS_HPP_
