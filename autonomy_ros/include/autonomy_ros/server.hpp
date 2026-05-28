/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 * email: quandy2020@126.com
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

#ifndef AUTONOMY_ROS__SERVER_HPP_
#define AUTONOMY_ROS__SERVER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/options.hpp"
#include "autonomy_ros/manager.hpp"
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

namespace autonomy_ros
{
class Visualizer;

/** @brief ROS navigation API: actions, services, and RViz goal topics. */
class NavigationService
{
public:
  NavigationService(
    rclcpp::Node & node,
    ::autonomy::system::Autonomy & core,
    const CoreOptions & core_options,
    const NavigationOptions & navigation_options,
    TaskManager::StopMotionFn stop_motion = {},
    Visualizer * visualizer = nullptr);

  void UpdateOdom(const nav_msgs::msg::Odometry & odom);

  bool HasOdometry() const;

  bool HasActiveNavigationTask() const;

  bool IsControllerEnabled() const;

private:
  using NavigatePose = autonomy_msgs::action::NavigatePose;
  using NavigateThrough = autonomy_msgs::action::NavigateThrough;

  bool WasPreempted(const std::string & task_id) const;

  rclcpp_action::GoalResponse HandleGoal(const std::string & task_id, uint8_t task_type) const;

  rclcpp_action::CancelResponse HandleCancel();

  template<typename SrvT, typename HandlerFn>
  void RegisterService(
    std::shared_ptr<rclcpp::Service<SrvT>> & server,
    const char * service_name,
    HandlerFn && handler);

  template<typename ActionT, typename GoalFn>
  void RegisterActionServer(
    std::shared_ptr<rclcpp_action::Server<ActionT>> & server,
    const char * action_name,
    GoalFn && goal_fn,
    void (NavigationService::*execute)(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>));

  bool NavigateToGoal(
    const std::string & task_id,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double timeout_sec,
    const std::function<bool()> & extra_cancel = {});

  void ExecuteNavigatePose(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigatePose>> handle);

  void ExecuteNavigateThrough(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThrough>> handle);

  void OnInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void RunTopicGoalPose(geometry_msgs::msg::PoseStamped goal, const std::string & task_id);

  rclcpp::Node & node_;
  std::unique_ptr<TaskManager> task_manager_;
  Visualizer * visualizer_{nullptr};

  std::string init_pose_topic_;
  std::string goal_pose_topic_;
  double waypoint_timeout_sec_{120.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  rclcpp_action::Server<NavigatePose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigateThrough>::SharedPtr navigate_through_server_;

  rclcpp::Service<autonomy_msgs::srv::CancelTask>::SharedPtr cancel_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::GetTaskStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<autonomy_msgs::srv::PauseTask>::SharedPtr pause_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::ResumeTask>::SharedPtr resume_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::TriggerEmergencyStop>::SharedPtr estop_srv_;
  rclcpp::Service<autonomy_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_srv_;
};

}  // namespace autonomy_ros

template<typename SrvT, typename HandlerFn>
void autonomy_ros::NavigationService::RegisterService(
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
void autonomy_ros::NavigationService::RegisterActionServer(
  std::shared_ptr<rclcpp_action::Server<ActionT>> & server,
  const char * action_name,
  GoalFn && goal_fn,
  void (NavigationService::*execute)(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>))
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
      return HandleCancel();
    },
    [this, execute](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & handle) {
      std::thread{execute, this, handle}.detach();
    });
}

#endif  // AUTONOMY_ROS__SERVER_HPP_
