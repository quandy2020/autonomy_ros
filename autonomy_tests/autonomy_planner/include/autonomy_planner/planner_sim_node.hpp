/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace autonomy_planner
{

class PlannerSimNode : public rclcpp::Node
{
public:
  PlannerSimNode();
  ~PlannerSimNode() override;

private:
  void SetupCostmap();
  void SetupPlanners();
  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OnTick();
  void UpdateAndPublishCostmap();
  bool PlanToGoal(const autonomy::commsgs::geometry_msgs::PoseStamped & goal);
  autonomy::planning::common::GlobalPlanner::SharedPtr GetActivePlanner() const;

  std::string frame_id_;
  std::string base_frame_;
  std::string planner_id_;
  std::string configuration_directory_;
  double costmap_publish_hz_{5.0};

  autonomy::planning::proto::PlannerOptions planner_options_;
  std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
  std::shared_ptr<autonomy::map::costmap_2d::Costmap2DWrapper> costmap_;
  std::unordered_map<std::string, autonomy::planning::common::GlobalPlanner::SharedPtr>
  planners_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};

  std::mutex cloud_mutex_;
  autonomy::commsgs::sensor_msgs::PointCloud2 latest_cloud_;
  bool have_cloud_{false};

  std::mutex costmap_mutex_;
  std::mutex plan_mutex_;
  autonomy::commsgs::planning_msgs::Path latest_plan_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_debug_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace autonomy_planner
