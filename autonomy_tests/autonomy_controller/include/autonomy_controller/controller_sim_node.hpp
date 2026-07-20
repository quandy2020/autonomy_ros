/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy_controller/path_generator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_controller
{

class ControllerSimNode : public rclcpp::Node
{
public:
  ControllerSimNode();
  ~ControllerSimNode() override;

private:
  void SetupCostmap();
  void SetupController();
  void SetupPath();
  bool ClosedLoopTrackingMode() const;
  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OnTick();
  void ApplyReferencePlan(bool reset_executed_path);
  void PublishSetPose(const autonomy::commsgs::geometry_msgs::PoseStamped & pose);
  void PublishReferencePath();
  void PublishZeroCmd();
  /** Feed latest cloud (if any), update rolling costmap, publish OccupancyGrid. */
  void UpdateAndPublishCostmap();
  /** Publish MPPI candidate (cyan) + optimal (orange) trajectories for RViz. */
  void PublishMppiTrajectories(double robot_x, double robot_y);

  std::string frame_id_;
  std::string base_frame_;
  std::string controller_id_;
  std::string configuration_directory_;
  double controller_frequency_{20.0};
  bool repeat_path_{true};
  bool snap_robot_to_path_start_{true};
  bool following_{false};
  /** For closed paths: require leaving the goal neighborhood before lap complete. */
  bool left_goal_region_{false};
  /** Wait until robot is near path start (after set_pose) before arming lap logic. */
  bool armed_from_start_{false};
  /** True when navigating to a /goal_pose (open path, not closed loop). */
  bool goal_pose_mode_{false};
  /** Straight-line distance to goal when the current goal_pose was issued. */
  double goal_initial_dist_{0.0};

  autonomy_ros::PathShape path_shape_{autonomy_ros::PathShape::Circle};
  autonomy_ros::PathGeneratorParams path_params_;

  autonomy::control::proto::ControllerOptions controller_options_;
  std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
  std::shared_ptr<autonomy::map::costmap_2d::Costmap2DWrapper> costmap_;
  std::unique_ptr<autonomy::control::common::ControllerInterface> controller_;
  /** Non-owning; set when controller_id is mppi. */
  autonomy::control::controller::mppi_controller::MPPIController * mppi_{nullptr};
  autonomy::control::checker::SimpleGoalChecker goal_checker_;
  autonomy::commsgs::planning_msgs::Path reference_path_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};

  std::mutex cloud_mutex_;
  autonomy::commsgs::sensor_msgs::PointCloud2 latest_cloud_;
  bool have_cloud_{false};

  /** Serialize feed/updateMap vs control-cycle costmap reads. */
  std::mutex costmap_mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr set_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executed_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    mppi_candidates_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mppi_optimal_path_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  nav_msgs::msg::Path executed_path_;

  /** Max LINE_STRIP count for /controller_sim/mppi_candidates. */
  int mppi_viz_max_candidates_{40};
  double mppi_viz_line_width_{0.008};
  int mppi_viz_published_candidates_{0};
};

}  // namespace autonomy_controller
