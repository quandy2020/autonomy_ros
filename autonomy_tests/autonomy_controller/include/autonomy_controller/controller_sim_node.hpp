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
 * @brief ROS 2 node for closed-loop controller simulation.
 */

#ifndef AUTONOMY_CONTROLLER_CONTROLLER_SIM_NODE_HPP_
#define AUTONOMY_CONTROLLER_CONTROLLER_SIM_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "autonomy_ros/conversions/sensor_msgs.hpp"
#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy_controller/mppi_trajectory_viz.hpp"
#include "autonomy_controller/path_generator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace autonomy_controller
{

/**
 * @class ControllerSimNode
 * @brief Tracks a reference path with MPPI / RPP / Graceful on a rolling costmap.
 *
 * Default mode follows a generated closed path (circle, rectangle, figure-eight).
 * /goal_pose switches to open-loop point-to-point navigation.
 */
class ControllerSimNode : public rclcpp::Node
{
public:
  /** @brief Declares parameters, loads controller/costmap, starts control timer. */
  ControllerSimNode();

  /** @brief Stops controller, publishes zero cmd_vel, tears down costmap. */
  ~ControllerSimNode() override;

private:
  struct LapProgress
  {
    bool armed_from_start{false};
    bool left_goal_region{false};
    double goal_initial_dist{0.0};
  };

  void DeclareParameters();
  void LoadParameters();
  void InitTransform();
  void SetupRosInterfaces();
  void SetupCostmap();
  void SetupController();
  void SetupPath();

  bool ClosedLoopTrackingMode() const;
  bool ShouldCheckGoalReached(
    const automsgs::msgs::geometry_msgs::PoseStamped & pose,
    double xy_tolerance);

  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OnTick();

  void ApplyReferencePlan(bool reset_executed_path);
  void PublishSetPose(const automsgs::msgs::geometry_msgs::PoseStamped & pose);
  void PublishReferencePath();
  void PublishZeroCmd();
  void UpdateAndPublishCostmap();
  void AppendExecutedPose(const nav_msgs::msg::Odometry & odom);

  std::string frame_id_;
  std::string base_frame_;
  std::string controller_id_;
  std::string configuration_directory_;
  double controller_frequency_{20.0};
  bool repeat_path_{true};
  bool snap_robot_to_path_start_{true};
  bool following_{false};
  bool goal_pose_mode_{false};

  LapProgress lap_;

  autonomy_ros::PathShape path_shape_{autonomy_ros::PathShape::Circle};
  autonomy_ros::PathGeneratorParams path_params_;

  autonomy::control::proto::ControllerOptions controller_options_;
  std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
  std::shared_ptr<autonomy::map::costmap_2d::Costmap2DWrapper> costmap_;
  std::unique_ptr<autonomy::control::common::ControllerInterface> controller_;
  autonomy::control::controller::mppi_controller::MPPIController * mppi_{nullptr};
  autonomy::control::checker::SimpleGoalChecker goal_checker_;
  automsgs::msgs::nav_msgs::Path reference_path_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};

  std::mutex cloud_mutex_;
  automsgs::msgs::sensor_msgs::PointCloud2 latest_cloud_;
  bool have_cloud_{false};

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
  rclcpp::TimerBase::SharedPtr tick_timer_;

  MppiTrajectoryVisualizer mppi_viz_;
  nav_msgs::msg::Path executed_path_;
};

}  // namespace autonomy_controller

#endif  // AUTONOMY_CONTROLLER_CONTROLLER_SIM_NODE_HPP_
