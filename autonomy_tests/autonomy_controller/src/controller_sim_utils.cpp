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
 * @brief Implements controller simulation utility helpers.
 */

#include "autonomy_controller/controller_sim_utils.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/parameter_options.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

namespace autonomy_controller
{

autonomy::control::proto::ControllerOptions LoadControllerOptionsBundle(
  const std::string & configuration_directory)
{
  const auto dirs =
    ::autonomy::common::ConfigurationSearchDirectories(configuration_directory);
  auto resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
    *resolver, "control/controller.lua");
  auto file_resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  ::autonomy::common::LuaParameterDictionary lua(code, std::move(file_resolver));
  auto root = lua.GetDictionary("AUTONOMY_CONTROLLER");
  auto options = autonomy::control::LoadOptions(root.get());

  if (!options.has_pure_pursuit_controller_options()) {
    auto empty_resolver =
      std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
    ::autonomy::common::LuaParameterDictionary empty(
      "return {}", std::move(empty_resolver));
    *options.mutable_pure_pursuit_controller_options() =
      autonomy::control::controller::pure_pursuit_controller::LoadOptions(&empty);
  }
  return options;
}

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
  double raytrace_max_range)
{
  autonomy::map::proto::Costmap2DOptions options;
  options.set_enabled(true);
  options.set_frame_id(frame_id);
  options.set_name("controller_sim_costmap");
  options.set_resolution(resolution);
  options.set_width(width);
  options.set_height(height);
  options.set_update_frequency(update_frequency);
  options.set_rolling_window(true);
  options.set_robot_radius(robot_radius);
  options.add_plugins("obstacle_layer");
  options.add_plugins("inflation_layer");

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
    tf, "controller_sim", false);
}

}  // namespace autonomy_controller
