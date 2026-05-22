// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__AUTONOMY_HPP_
#define AUTONOMY_ROS__AUTONOMY_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/system/system.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/scheduler/task_scheduler.hpp"
#include "autonomy_ros/bridge/map_bridge.hpp"
#include "autonomy_ros/bridge/platform_bridge.hpp"
#include "autonomy_ros/bridge/tf_bridge.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/**
 * @class autonomy_ros::Autonomy
 * @brief Sole ROS 2 facade to the autonomy core (map, TF, platform, navigation).
 */
class Autonomy
{
public:
  explicit Autonomy(rclcpp::Node & node);

  void start();
  void shutdown();

  bool isRunning() const { return running_; }

  ::autonomy::system::AutonomyNode & core();
  const ::autonomy::system::AutonomyNode & core() const;

  ::autonomy::tasks::common::TaskContext * taskContext();
  const ::autonomy::tasks::common::TaskContext * taskContext() const;

  ::autonomy::tasks::scheduler::TaskScheduler * taskScheduler();
  const ::autonomy::tasks::scheduler::TaskScheduler * taskScheduler() const;
  bool useBehaviorTreeNavigation() const;
  bool hasTaskScheduler() const;

  /**
   * @brief Navigate to pose using BT when enabled, else planner + controller tick loop.
   */
  bool navigateToPose(
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker = nullptr,
    double timeout_sec = 0.0);

  void setNavigationGoal(const geometry_msgs::msg::PoseStamped & goal);
  std::optional<nav_msgs::msg::Path> lastPath() const;
  void setControllerEnabled(bool enabled);
  bool controllerEnabled() const;
  double goalTolerance() const;
  void applyControllerSpeedLimit(
    const ::autonomy::commsgs::planning_msgs::SpeedLimit & limit);
  void clearControllerSpeedLimit();
  void requestCancelNavigation();

  const std::string & globalFrame() const { return global_frame_; }

  void addOdomListener(
    std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)> listener);
  void addPlanListener(std::function<void(const nav_msgs::msg::Path &)> listener);

  bool isFollowingPath() const { return following_path_.load(); }
  bool hasOdom() const;
  bool lastFollowPathSucceeded() const;
  bool lastFollowPathFailed() const;

  diagnostic_msgs::msg::DiagnosticArray buildDiagnostics() const;

private:
  void loadParameters();
  std::string resolveConfigDirectory() const;
  void startCore();
  void startTaskScheduler();
  void startRosBridges();
  void startOutboundIo();
  void stopOutboundIo();
  void controlStep();
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onSpeedLimitTopic(const std_msgs::msg::Float32::SharedPtr msg);
  void publishCostmaps();
  void publishDiagnostics();
  bool planToGoal(const geometry_msgs::msg::PoseStamped & goal);
  bool waitForDirectNavigation(
    const geometry_msgs::msg::PoseStamped & goal, double tolerance,
    std::function<bool()> cancel_checker, double timeout_sec);
  std::function<bool()> navigationCancelChecker();
  ::autonomy::map::costmap_2d::Costmap2DWrapper * plannerCostmap();
  void applyMapToCostmap(const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map);
  void feedScanToCostmap(const ::autonomy::commsgs::sensor_msgs::LaserScan & scan);
  bool snapshotCostmap(::autonomy::commsgs::map_msgs::OccupancyGrid & grid);
  void dispatchOdom(const nav_msgs::msg::Odometry::SharedPtr & msg);
  void notifyPlan(const nav_msgs::msg::Path & path);

  rclcpp::Node & node_;
  bool running_{false};

  std::string config_directory_;
  std::string config_file_{"autonomy.lua"};
  bool enable_bt_tasks_{true};
  bool use_bt_navigation_{true};
  std::string planner_id_;
  std::string controller_id_{"FollowPath"};
  std::string goal_checker_id_{"goal_checker"};
  std::string progress_checker_id_{"progress_checker"};
  std::string global_frame_{"map"};
  double goal_tolerance_{0.15};

  ::autonomy::system::AutonomyNode::UniquePtr core_;
  std::shared_ptr<::autonomy::control::utils::OdomSmoother> odom_smoother_;
  std::unique_ptr<::autonomy::tasks::scheduler::TaskScheduler> task_scheduler_;

  std::unique_ptr<bridge::TfBridge> tf_bridge_;
  std::unique_ptr<bridge::MapBridge> map_bridge_;
  std::unique_ptr<bridge::PlatformBridge> platform_bridge_;

  bool scan_enabled_{true};
  std::string scan_topic_{"/scan"};
  bool costmaps_enabled_{true};
  double costmap_hz_{1.0};
  bool diagnostics_enabled_{true};
  bool speed_limit_enabled_{true};
  bool speed_limit_percentage_{false};
  std::string speed_limit_topic_{"autonomy/speed_limit"};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_limit_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr costmap_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::atomic<bool> controller_enabled_{false};
  std::atomic<bool> bt_navigation_active_{false};
  std::atomic<bool> following_path_{false};
  std::atomic<bool> cancel_navigation_{false};
  std::atomic<int> last_follow_result_{0};

  mutable std::mutex odom_mutex_;
  std::vector<std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)>> odom_listeners_;
  std::vector<std::function<void(const nav_msgs::msg::Path &)>> plan_listeners_;

  mutable std::mutex path_mutex_;
  std::optional<nav_msgs::msg::Path> last_path_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__AUTONOMY_HPP_
