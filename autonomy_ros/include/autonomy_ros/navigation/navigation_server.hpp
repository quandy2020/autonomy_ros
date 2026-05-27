// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__NAVIGATION__NAVIGATION_SERVER_HPP_
#define AUTONOMY_ROS__NAVIGATION__NAVIGATION_SERVER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "autonomy_ros/system/constants.hpp"
#include "autonomy_ros/system/options.hpp"
#include "autonomy_ros/navigation/task_manager.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "autonomy_msgs/action/navigate_pose.hpp"
#include "autonomy_msgs/action/navigate_through.hpp"
#include "autonomy_msgs/srv/cancel_task.hpp"
#include "autonomy_msgs/srv/get_task_status.hpp"
#include "autonomy_msgs/srv/pause_task.hpp"
#include "autonomy_msgs/srv/resume_task.hpp"
#include "autonomy_msgs/srv/set_initial_pose.hpp"
#include "autonomy_msgs/srv/trigger_emergency_stop.hpp"

namespace autonomy::system
{
class Autonomy;
}

namespace autonomy_ros::viz
{
class Visualizer;
}

namespace autonomy_ros::navigation
{

using system::AutonomyCoreOptions;

/**
 * @brief ROS navigation API: single-goal and multi-waypoint actions plus RViz topics.
 */
class NavigationServer
{
public:
  NavigationServer(
    rclcpp::Node & node,
    ::autonomy::system::Autonomy & core,
    const AutonomyCoreOptions & core_options,
    TaskManager::StopMotionFn stop_motion = {},
    viz::Visualizer * visualizer = nullptr);

  void updateOdom(const nav_msgs::msg::Odometry & odom);

  bool hasOdometry() const;

  bool hasActiveNavigationTask() const;

  bool isControllerEnabled() const;

private:
  using NavigatePose = autonomy_msgs::action::NavigatePose;
  using NavigateThrough = autonomy_msgs::action::NavigateThrough;

  bool wasPreempted(const std::string & task_id) const;

  rclcpp_action::GoalResponse handleGoal(const std::string & task_id, uint8_t task_type) const;

  rclcpp_action::CancelResponse handleCancel();

  template<typename SrvT, typename HandlerFn>
  void registerService(
    std::shared_ptr<rclcpp::Service<SrvT>> & server,
    const char * service_name,
    HandlerFn && handler);

  template<typename ActionT, typename GoalFn>
  void registerActionServer(
    std::shared_ptr<rclcpp_action::Server<ActionT>> & server,
    const char * action_name,
    GoalFn && goal_fn,
    void (NavigationServer::*execute)(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>));

  void loadParameters();

  bool navigateToGoal(
    const std::string & task_id,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double timeout_sec,
    const std::function<bool()> & extra_cancel = {});

  void executeNavigatePose(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigatePose>> handle);

  void executeNavigateThrough(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThrough>> handle);

  void onInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void runTopicGoalPose(geometry_msgs::msg::PoseStamped goal, const std::string & task_id);

  rclcpp::Node & node_;
  std::unique_ptr<TaskManager> task_manager_;
  viz::Visualizer * visualizer_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  std::string init_pose_topic_{constants::topics::kInitPose};
  std::string goal_pose_topic_{constants::topics::kGoalPose};
  double waypoint_timeout_sec_{constants::defaults::kNavigationWaypointTimeoutSec};

  rclcpp_action::Server<NavigatePose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigateThrough>::SharedPtr navigate_through_server_;

  rclcpp::Service<autonomy_msgs::srv::CancelTask>::SharedPtr cancel_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::GetTaskStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<autonomy_msgs::srv::PauseTask>::SharedPtr pause_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::ResumeTask>::SharedPtr resume_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::TriggerEmergencyStop>::SharedPtr estop_srv_;
  rclcpp::Service<autonomy_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_srv_;
};

}  // namespace autonomy_ros::navigation

template<typename SrvT, typename HandlerFn>
void autonomy_ros::navigation::NavigationServer::registerService(
  std::shared_ptr<rclcpp::Service<SrvT>> & server,
  const char * service_name,
  HandlerFn && handler)
{
  server = node_.create_service<SrvT>(
    service_name,
    [fn = std::forward<HandlerFn>(handler)](
      const std::shared_ptr<typename SrvT::Request> req,
      std::shared_ptr<typename SrvT::Response> res) {
      fn(req, res);
    });
}

template<typename ActionT, typename GoalFn>
void autonomy_ros::navigation::NavigationServer::registerActionServer(
  std::shared_ptr<rclcpp_action::Server<ActionT>> & server,
  const char * action_name,
  GoalFn && goal_fn,
  void (NavigationServer::*execute)(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle))
{
  using Goal = typename ActionT::Goal;
  const auto node = node_.shared_from_this();
  server = rclcpp_action::create_server<ActionT>(
    node, action_name,
    [fn = std::forward<GoalFn>(goal_fn)](
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Goal> goal) -> rclcpp_action::GoalResponse {
      return fn(goal);
    },
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> &) {
      return handleCancel();
    },
    [this, execute](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & handle) {
      std::thread{execute, this, handle}.detach();
    });
}

#endif  // AUTONOMY_ROS__NAVIGATION__NAVIGATION_SERVER_HPP_
