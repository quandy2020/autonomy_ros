// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/options.hpp"

#include <utility>

namespace autonomy_ros
{

namespace
{
namespace params = constants::params;

template<typename T>
void declareIfNeeded(
  rclcpp::Node & node, const char * name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
}

}  // namespace

AutonomyCoreOptions loadAutonomyCoreOptions(rclcpp::Node & node)
{
  AutonomyCoreOptions options;

  declareIfNeeded(
    node, params::kAutonomyConfigDirectory, options.config_directory);
  declareIfNeeded(node, params::kAutonomyConfigFile, options.config_file);
  declareIfNeeded(node, params::kAutonomyEnableBtTasks, options.enable_bt_tasks);
  declareIfNeeded(
    node, params::kAutonomyUseBtNavigation, options.use_bt_navigation);
  declareIfNeeded(node, params::kAutonomyPlannerId, options.planner_id);
  declareIfNeeded(node, params::kAutonomyControllerId, options.controller_id);
  declareIfNeeded(node, params::kAutonomyGoalCheckerId, options.goal_checker_id);
  declareIfNeeded(
    node, params::kAutonomyProgressCheckerId, options.progress_checker_id);
  declareIfNeeded(node, params::kAutonomyGlobalFrame, options.global_frame);
  declareIfNeeded(node, params::kAutonomyGoalTolerance, options.goal_tolerance);
  declareIfNeeded(node, params::kAutonomyBaseFrame, options.base_frame);
  declareIfNeeded(node, params::kAutonomyMaxLinearVel, options.max_linear_vel);

  options.config_directory =
    node.get_parameter(params::kAutonomyConfigDirectory).as_string();
  options.config_file =
    node.get_parameter(params::kAutonomyConfigFile).as_string();
  options.enable_bt_tasks =
    node.get_parameter(params::kAutonomyEnableBtTasks).as_bool();
  options.use_bt_navigation =
    node.get_parameter(params::kAutonomyUseBtNavigation).as_bool();
  options.planner_id =
    node.get_parameter(params::kAutonomyPlannerId).as_string();
  options.controller_id =
    node.get_parameter(params::kAutonomyControllerId).as_string();
  options.goal_checker_id =
    node.get_parameter(params::kAutonomyGoalCheckerId).as_string();
  options.progress_checker_id =
    node.get_parameter(params::kAutonomyProgressCheckerId).as_string();
  options.global_frame =
    node.get_parameter(params::kAutonomyGlobalFrame).as_string();
  options.goal_tolerance =
    node.get_parameter(params::kAutonomyGoalTolerance).as_double();
  options.base_frame =
    node.get_parameter(params::kAutonomyBaseFrame).as_string();
  options.max_linear_vel =
    node.get_parameter(params::kAutonomyMaxLinearVel).as_double();

  return options;
}

AutonomyRosOptions loadAutonomyRosOptions(rclcpp::Node & node)
{
  AutonomyRosOptions options;

  declareIfNeeded(node, "autonomy.map_topic", options.map_topic);
  declareIfNeeded(node, "autonomy.publish_map", options.publish_map);
  declareIfNeeded(node, "autonomy.tf_topic", options.tf_topic);
  declareIfNeeded(node, "autonomy.tf_static_topic", options.tf_static_topic);

  declareIfNeeded(node, params::kAutonomyOdomTopic, options.odom_topic);
  declareIfNeeded(node, params::kAutonomyCmdVelTopic, options.cmd_vel_topic);
  declareIfNeeded(
    node, params::kAutonomyCmdVelTeleopTopic, options.cmd_vel_teleop_topic);

  declareIfNeeded(node, params::kAutonomyEnableScanBridge, options.scan_enabled);
  declareIfNeeded(node, params::kAutonomyScanTopic, options.scan_topic);
  declareIfNeeded(node, params::kAutonomyPublishCostmaps, options.publish_costmaps);
  declareIfNeeded(
    node, params::kAutonomyCostmapPublishHz, options.costmap_publish_hz);
  declareIfNeeded(
    node, params::kAutonomyPublishDiagnostics, options.publish_diagnostics);

  auto & sb = options.sensor_bridge;
  declareIfNeeded(
    node, params::kSensorBridgePointCloudEnabled, sb.point_cloud_enabled);
  declareIfNeeded(node, params::kSensorBridgePointCloudTopic, sb.point_cloud_topic);
  declareIfNeeded(node, params::kSensorBridgeRangeEnabled, sb.range_enabled);
  declareIfNeeded(node, params::kSensorBridgeRangeTopic, sb.range_topic);
  declareIfNeeded(node, params::kSensorBridgeImageEnabled, sb.image_enabled);
  declareIfNeeded(node, params::kSensorBridgeImageTopic, sb.image_topic);
  declareIfNeeded(node, params::kSensorBridgeImuEnabled, sb.imu_enabled);
  declareIfNeeded(node, params::kSensorBridgeImuTopic, sb.imu_topic);

  options.map_topic = node.get_parameter("autonomy.map_topic").as_string();
  options.publish_map = node.get_parameter("autonomy.publish_map").as_bool();
  options.tf_topic = node.get_parameter("autonomy.tf_topic").as_string();
  options.tf_static_topic =
    node.get_parameter("autonomy.tf_static_topic").as_string();

  options.odom_topic = node.get_parameter(params::kAutonomyOdomTopic).as_string();
  options.cmd_vel_topic =
    node.get_parameter(params::kAutonomyCmdVelTopic).as_string();
  options.cmd_vel_teleop_topic =
    node.get_parameter(params::kAutonomyCmdVelTeleopTopic).as_string();

  options.scan_enabled =
    node.get_parameter(params::kAutonomyEnableScanBridge).as_bool();
  options.scan_topic = node.get_parameter(params::kAutonomyScanTopic).as_string();
  options.publish_costmaps =
    node.get_parameter(params::kAutonomyPublishCostmaps).as_bool();
  options.costmap_publish_hz =
    node.get_parameter(params::kAutonomyCostmapPublishHz).as_double();
  options.publish_diagnostics =
    node.get_parameter(params::kAutonomyPublishDiagnostics).as_bool();

  sb.point_cloud_enabled =
    node.get_parameter(params::kSensorBridgePointCloudEnabled).as_bool();
  sb.point_cloud_topic =
    node.get_parameter(params::kSensorBridgePointCloudTopic).as_string();
  sb.range_enabled = node.get_parameter(params::kSensorBridgeRangeEnabled).as_bool();
  sb.range_topic = node.get_parameter(params::kSensorBridgeRangeTopic).as_string();
  sb.image_enabled = node.get_parameter(params::kSensorBridgeImageEnabled).as_bool();
  sb.image_topic = node.get_parameter(params::kSensorBridgeImageTopic).as_string();
  sb.imu_enabled = node.get_parameter(params::kSensorBridgeImuEnabled).as_bool();
  sb.imu_topic = node.get_parameter(params::kSensorBridgeImuTopic).as_string();

  return options;
}

std::tuple<AutonomyCoreOptions, AutonomyRosOptions> createOptions(rclcpp::Node & node)
{
  AutonomyCoreOptions core = loadAutonomyCoreOptions(node);
  AutonomyRosOptions ros = loadAutonomyRosOptions(node);
  return {std::move(core), std::move(ros)};
}

::autonomy::tasks::RuntimeOptions toRuntimeOptions(const AutonomyCoreOptions & core)
{
  ::autonomy::tasks::RuntimeOptions opts;
  opts.enable_bt_tasks = core.enable_bt_tasks;
  opts.use_bt_navigation = core.use_bt_navigation;
  opts.config_directory = core.config_directory;
  opts.planner_id = core.planner_id;
  opts.controller_id = core.controller_id;
  opts.goal_checker_id = core.goal_checker_id;
  opts.progress_checker_id = core.progress_checker_id;
  opts.global_frame = core.global_frame;
  opts.goal_tolerance = core.goal_tolerance;
  return opts;
}

}  // namespace autonomy_ros
