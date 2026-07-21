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
 * @brief Implements planner simulation utility helpers.
 */

#include "autonomy_planner/planner_sim_utils.hpp"

#include <cmath>
#include <filesystem>
#include <vector>

#include "autonomy/common/config.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

namespace autonomy_planner {

std::string ResolveMapYamlPath(const std::string & map_file)
{
  namespace fs = std::filesystem;
  if (map_file.empty()) {
    return {};
  }

  const fs::path requested(map_file);
  if (requested.is_absolute()) {
    return fs::exists(requested) ? requested.string() : std::string{};
  }

  const fs::path config_root(autonomy::common::kConfigurationFilesDirectory);
  const std::vector<fs::path> candidates = {
    config_root / "data" / map_file,
    config_root / "map" / map_file,
  };
  for (const auto & candidate : candidates) {
    if (fs::exists(candidate)) {
      return candidate.string();
    }
  }
  return {};
}

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
  double raytrace_max_range)
{
  autonomy::map::proto::Costmap2DOptions options;
  options.set_enabled(true);
  options.set_frame_id(frame_id);
  options.set_name("planner_sim_costmap");
  options.set_resolution(resolution);
  options.set_width(static_cast<int32_t>(std::lround(width)));
  options.set_height(static_cast<int32_t>(std::lround(height)));
  options.set_update_frequency(update_frequency);
  options.set_rolling_window(false);
  options.set_robot_radius(robot_radius);
  options.add_plugins("static_layer");
  options.add_plugins("obstacle_layer");
  options.add_plugins("inflation_layer");

  auto * static_layer = options.mutable_static_layer();
  static_layer->set_enabled(true);
  static_layer->set_subscribe_to_updates(false);
  static_layer->set_footprint_clearing_enabled(false);
  static_layer->set_map_topic(map_topic);
  static_layer->set_transform_tolerance(0.1);

  auto * obstacle = options.mutable_obstacle_layer();
  obstacle->set_enabled(true);
  obstacle->set_footprint_clearing_enabled(true);
  auto & sources = *obstacle->mutable_sensor_sources();
  auto & src = sources["sim_cloud"];
  src.set_topic(cloud_topic);
  src.set_data_type("PointCloud2");
  src.set_marking(true);
  src.set_clearing(false);
  src.set_min_obstacle_height(obstacle_min_height);
  src.set_max_obstacle_height(obstacle_max_height);
  src.set_raytrace_max_range(raytrace_max_range);
  src.set_raytrace_min_range(0.0);

  auto * inflation = options.mutable_inflation_layer();
  inflation->set_enabled(true);
  inflation->set_cost_scaling_factor(inflation_cost_scaling_factor);
  inflation->set_inflation_radius(inflation_radius);
  return options;
}

const char * PlannerResultName(uint32_t code)
{
  using autonomy::planning::proto::PlannerResultCode;
  switch (static_cast<PlannerResultCode>(code)) {
    case PlannerResultCode::PLANNER_SUCCESS: return "SUCCESS";
    case PlannerResultCode::PLANNER_FAILURE: return "FAILURE";
    case PlannerResultCode::PLANNER_CANCELED: return "CANCELED";
    case PlannerResultCode::PLANNER_INVALID_START: return "INVALID_START";
    case PlannerResultCode::PLANNER_INVALID_GOAL: return "INVALID_GOAL";
    case PlannerResultCode::PLANNER_BLOCKED_START: return "BLOCKED_START";
    case PlannerResultCode::PLANNER_BLOCKED_GOAL: return "BLOCKED_GOAL";
    case PlannerResultCode::PLANNER_NO_PATH_FOUND: return "NO_PATH_FOUND";
    case PlannerResultCode::PLANNER_PAT_EXCEEDED: return "PAT_EXCEEDED";
    case PlannerResultCode::PLANNER_EMPTY_PATH: return "EMPTY_PATH";
    case PlannerResultCode::PLANNER_TF_ERROR: return "TF_ERROR";
    case PlannerResultCode::PLANNER_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case PlannerResultCode::PLANNER_INVALID_PLUGIN: return "INVALID_PLUGIN";
    case PlannerResultCode::PLANNER_INTERNAL_ERROR: return "INTERNAL_ERROR";
    default: return "UNKNOWN";
  }
}

void PublishOdomTf(
  const nav_msgs::msg::Odometry & odom,
  const std::string & default_parent,
  const std::string & default_child)
{
  geometry_msgs::TransformStamped tf;
  const auto & stamp = odom.header.stamp;
  tf.header.stamp =
    static_cast<uint64_t>(stamp.sec) * 1'000'000'000ULL +
    static_cast<uint64_t>(stamp.nanosec);
  tf.header.frame_id =
    odom.header.frame_id.empty() ? default_parent : odom.header.frame_id;
  tf.child_frame_id =
    odom.child_frame_id.empty() ? default_child : odom.child_frame_id;
  tf.transform.translation.x = odom.pose.pose.position.x;
  tf.transform.translation.y = odom.pose.pose.position.y;
  tf.transform.translation.z = odom.pose.pose.position.z;
  tf.transform.rotation.x = odom.pose.pose.orientation.x;
  tf.transform.rotation.y = odom.pose.pose.orientation.y;
  tf.transform.rotation.z = odom.pose.pose.orientation.z;
  tf.transform.rotation.w = odom.pose.pose.orientation.w;
  autonomy::transform::Buffer::Instance()->setTransform(
    tf, "planner_sim", false);
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace autonomy_planner
