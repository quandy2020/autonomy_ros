// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_ROS__COMMAND__COMMAND_INTERFACE_HPP_
#define AUTONOMY_ROS__COMMAND__COMMAND_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "autonomy_ros/controller/controller.hpp"
#include "autonomy_ros/planner/planner.hpp"
#include "autonomy_ros/task/task_manager.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "autonomy_msgs/action/dock.hpp"
#include "autonomy_msgs/action/follow.hpp"
#include "autonomy_msgs/action/guided_tour.hpp"
#include "autonomy_msgs/action/navigate_pose.hpp"
#include "autonomy_msgs/action/navigate_through.hpp"
#include "autonomy_msgs/action/teleop.hpp"
#include "autonomy_msgs/msg/dock_station.hpp"
#include "autonomy_msgs/srv/cancel_task.hpp"
#include "autonomy_msgs/srv/continue_tour.hpp"
#include "autonomy_msgs/srv/get_task_status.hpp"
#include "autonomy_msgs/srv/list_docks.hpp"
#include "autonomy_msgs/srv/pause_task.hpp"
#include "autonomy_msgs/srv/resume_task.hpp"
#include "autonomy_msgs/srv/set_initial_pose.hpp"
#include "autonomy_msgs/srv/set_teleop_mode.hpp"
#include "autonomy_msgs/srv/skip_to_exhibit.hpp"
#include "autonomy_msgs/srv/trigger_emergency_stop.hpp"

namespace autonomy_ros::command
{

/**
 * @class autonomy_ros::command::CommandInterface
 * @brief External API surface: autonomy_msgs actions, services, and motion execution
 *
 * Registers servers under autonomy/* on the parent node. Each action runs in a
 * detached thread; TaskManager + TaskMuxer ensure one logical owner. Planner
 * and Controller implement navigation; teleop disables Controller so external
 * /cmd_vel is not overwritten.
 *
 * Parameters: command.default_dock_id, command.waypoint_timeout_sec,
 * command.dock_x/y/w (see loadDocks).
 */
class CommandInterface
{
public:
  /**
   * @brief Constructor for autonomy_ros::command::CommandInterface
   * @param node Parent node for servers, services and publishers
   * @param task_manager Shared task state used across all commands
   * @param planner Planner invoked for navigation goals
   * @param controller Controller enabled/disabled per task and safety state
   */
  CommandInterface(
    rclcpp::Node & node, task::TaskManager & task_manager,
    planner::Planner & planner, controller::Controller & controller);

  /**
   * @brief Register all autonomy/* action servers and services
   */
  void start();

private:
  using NavigatePose = autonomy_msgs::action::NavigatePose;
  using NavigateThrough = autonomy_msgs::action::NavigateThrough;
  using Follow = autonomy_msgs::action::Follow;
  using GuidedTour = autonomy_msgs::action::GuidedTour;
  using Dock = autonomy_msgs::action::Dock;
  using Teleop = autonomy_msgs::action::Teleop;

  /**
   * @brief Planar distance from current odom pose to goal
   * @param goal Target position in same frame as odom
   * @return Euclidean distance in meters
   */
  double distanceToPose(const geometry_msgs::msg::Pose & goal) const;

  /**
   * @brief True if robot is within tolerance of goal position
   * @param goal Target pose
   * @param tolerance Radius in meters
   * @return True if distance < tolerance
   */
  bool reachedPose(const geometry_msgs::msg::Pose & goal, double tolerance) const;

  /**
   * @brief Blocking loop for action execution (10 Hz)
   * @param task_id Muxer owner checked each iteration
   * @param cancel_check Return true to exit loop
   * @param on_tick Feedback / progress callback
   */
  void spinUntilCancel(
    const std::string & task_id,
    const std::function<bool()> & cancel_check,
    const std::function<void()> & on_tick);

  /**
   * @brief True if muxer no longer grants this task the active slot
   * @param task_id Task identifier
   * @return True if preempted by a higher-priority task
   */
  bool wasPreempted(const std::string & task_id) const;

  /**
   * @brief Action goal validation (estop + canBeginTask)
   * @param task_id Goal task_id field
   * @param task_type autonomy_msgs/TaskType for priority
   * @param force_preempt Teleop preempt_other_tasks
   * @return ACCEPT_AND_EXECUTE or REJECT
   */
  rclcpp_action::GoalResponse handleGoal(
    const std::string & task_id, uint8_t task_type, bool force_preempt = false) const;

  /**
   * @brief Action cancel: stop publishing autonomous cmd_vel
   * @return ACCEPT
   */
  rclcpp_action::CancelResponse handleCancel();

  /**
   * @brief Load command.* parameters from the node
   */
  void loadParameters();

  /**
   * @brief Build docks_ from command.dock_* parameters
   */
  void loadDocks();

  /**
   * @brief Plan, enable controller, and wait until reached or timeout/preempt
   * @param task_id Muxer owner
   * @param goal_pose Navigation target
   * @param tolerance Arrival radius
   * @param timeout_sec Max wait before failure
   * @param extra_cancel Additional cancel predicate (e.g. action cancel)
   * @return True if goal reached while still owning task
   */
  bool navigateToGoal(
    const std::string & task_id,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double tolerance,
    double timeout_sec,
    const std::function<bool()> & extra_cancel = {});

  /**
   * @brief Staging (optional) then align to dock pose
   * @param task_id Muxer owner
   * @param goal Dock action goal fields
   * @param result Filled on success/failure
   * @param cancel_check Exit when true
   * @return True if dock pose reached
   */
  bool runDock(
    const std::string & task_id,
    const autonomy_msgs::action::Dock::Goal & goal,
    autonomy_msgs::action::Dock::Result & result,
    const std::function<bool()> & cancel_check);

  /**
   * @brief Apply NavigatePose/Tour cruise speed cap on controller
   * @param max_speed From goal; 0 means no change
   */
  void applySpeedLimit(float max_speed);

  /**
   * @brief Restore controller max_linear_vel to parameter default
   */
  void restoreSpeedLimit();

  /**
   * @brief NavigatePose action worker
   * @param handle Server goal handle
   */
  void executeNavigatePose(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigatePose>> handle);

  /**
   * @brief NavigateThrough action worker
   * @param handle Server goal handle
   */
  void executeNavigateThrough(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThrough>> handle);

  /**
   * @brief Follow action worker (target_pose tracking)
   * @param handle Server goal handle
   */
  void executeFollow(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> handle);

  /**
   * @brief GuidedTour exhibition loop worker
   * @param handle Server goal handle
   */
  void executeGuidedTour(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GuidedTour>> handle);

  /**
   * @brief Dock action worker
   * @param handle Server goal handle
   */
  void executeDock(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Dock>> handle);

  /**
   * @brief Teleop session worker (controller disabled)
   * @param handle Server goal handle
   */
  void executeTeleop(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Teleop>> handle);

  rclcpp::Node & node_;
  task::TaskManager & task_manager_;
  planner::Planner & planner_;
  controller::Controller & controller_;

  /** @brief Feeds TaskManager.current_pose from /odom */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  /** @brief SetInitialPose service publishes here (AMCL-compatible topic name) */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  /** @brief Charging stations from parameters (ListDocks) */
  std::vector<autonomy_msgs::msg::DockStation> docks_;
  /** @brief Default dock_id for tour-complete and low-battery dock */
  std::string default_dock_id_{"dock_main"};
  /** @brief Per-waypoint navigation timeout (command.waypoint_timeout_sec) */
  double waypoint_timeout_sec_{120.0};

  rclcpp_action::Server<NavigatePose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigateThrough>::SharedPtr navigate_through_server_;
  rclcpp_action::Server<Follow>::SharedPtr follow_server_;
  rclcpp_action::Server<GuidedTour>::SharedPtr guided_tour_server_;
  rclcpp_action::Server<Dock>::SharedPtr dock_server_;
  rclcpp_action::Server<Teleop>::SharedPtr teleop_server_;

  rclcpp::Service<autonomy_msgs::srv::CancelTask>::SharedPtr cancel_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::GetTaskStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<autonomy_msgs::srv::PauseTask>::SharedPtr pause_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::ResumeTask>::SharedPtr resume_task_srv_;
  rclcpp::Service<autonomy_msgs::srv::ContinueTour>::SharedPtr continue_tour_srv_;
  rclcpp::Service<autonomy_msgs::srv::SkipToExhibit>::SharedPtr skip_exhibit_srv_;
  rclcpp::Service<autonomy_msgs::srv::TriggerEmergencyStop>::SharedPtr estop_srv_;
  rclcpp::Service<autonomy_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_srv_;
  rclcpp::Service<autonomy_msgs::srv::SetTeleopMode>::SharedPtr set_teleop_mode_srv_;
  rclcpp::Service<autonomy_msgs::srv::ListDocks>::SharedPtr list_docks_srv_;
};

}  // namespace autonomy_ros::command

#endif  // AUTONOMY_ROS__COMMAND__COMMAND_INTERFACE_HPP_
