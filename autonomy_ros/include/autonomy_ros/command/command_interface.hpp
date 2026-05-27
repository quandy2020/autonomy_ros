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
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <optional>

#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/options.hpp"
#include "autonomy_ros/task/task_manager.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

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

namespace autonomy::system
{
class Autonomy;
}

namespace autonomy_ros::visualization
{
class Visualizer;
}

namespace autonomy_ros::command
{

/**
 * @class autonomy_ros::command::CommandInterface
 * @brief External API surface: autonomy_msgs actions, services, and motion execution
 *
 * Registers action servers and services under the autonomy namespace on the parent node. Each action runs in a
 * detached thread; TaskManager + TaskMuxer ensure one logical owner. Planner
 * and Controller implement navigation; teleop disables Controller so external
 * /cmd_vel is not overwritten.
 *
 * Parameters: command.default_dock_id, command.waypoint_timeout_sec,
 * command.dock_x/y/w (see loadDocks), command.init_pose_topic,
 * command.goal_pose_topic.
 *
 * Subscriptions:
 * - init_pose (PoseWithCovarianceStamped): republished to /initialpose for localization
 * - goal_pose (PoseStamped): starts a NavigatePose-style task (RViz 2D Goal Pose)
 */
class CommandInterface
{
public:
  /**
   * @brief Construct and register all autonomy action servers and services.
   * @param node Parent node for servers, services and publishers
   * @param core Autonomy core executed via TaskManager
   * @param core_options Planner / controller options from ROS parameters
   * @param stop_motion Hook when motion must stop (e.g. publish zero cmd_vel)
   * @param cmd_vel_teleop_topic Teleop ingress topic; empty disables subscription
   */
  CommandInterface(
    rclcpp::Node & node,
    ::autonomy::system::Autonomy & core,
    const AutonomyCoreOptions & core_options,
    task::TaskManager::StopMotionFn stop_motion = {},
    const std::string & cmd_vel_teleop_topic = {},
    visualization::Visualizer * visualizer = nullptr);

  void updateOdom(const nav_msgs::msg::Odometry & odom);

  std::optional<::autonomy::commsgs::geometry_msgs::TwistStamped> teleopCommand() const;

  bool hasOdometry() const;

  bool hasActiveNavigationTask() const;

  bool isControllerEnabled() const;

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
   * @brief Register an autonomy service with a request/response handler.
   */
  template<typename SrvT, typename HandlerFn>
  void registerService(
    std::shared_ptr<rclcpp::Service<SrvT>> & server,
    const char * service_name,
    HandlerFn && handler);

  /**
   * @brief Register an autonomy action server (goal check, cancel, detached execute).
   */
  template<typename ActionT, typename GoalFn>
  void registerActionServer(
    std::shared_ptr<rclcpp_action::Server<ActionT>> & server,
    const char * action_name,
    GoalFn && goal_fn,
    void (CommandInterface::*execute)(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>));

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

  /**
   * @brief init_pose topic: forward to /initialpose (same as SetInitialPose service)
   */
  void onInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief goal_pose topic: plan and run navigation task (RViz 2D Goal Pose)
   */
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Background worker for goal_pose topic navigation
   */
  void runTopicGoalPose(geometry_msgs::msg::PoseStamped goal, const std::string & task_id);

  void startFollowTargetTracking();
  void startTeleopIngress(const std::string & cmd_vel_teleop_topic);
  void onFollowDetections(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  std::optional<geometry_msgs::msg::PoseStamped> lookupFollowTargetPose(
    const std::string & target_id) const;
  bool followTargetTrackingAvailable() const;
  static geometry_msgs::msg::PoseStamped poseFromDetection(
    const vision_msgs::msg::Detection3D & detection);

  rclcpp::Node & node_;
  std::unique_ptr<task::TaskManager> task_manager_;
  visualization::Visualizer * visualizer_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_teleop_sub_;

  // RViz / external init_pose (republished to `/initialpose` topic)
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

  // RViz 2D Goal Pose (from `/goal_pose` topic)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  // SetInitialPose service and init_pose callback publish to `/initialpose` topic
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  std::string init_pose_topic_{constants::topics::kInitPose};
  std::string goal_pose_topic_{constants::topics::kGoalPose};

  // Charging stations from parameters (ListDocks)
  std::vector<autonomy_msgs::msg::DockStation> docks_;

  // Default dock_id for tour-complete and low-battery dock
  std::string default_dock_id_{constants::defaults::kCommandDockId};

  // Per-waypoint navigation timeout (command.waypoint_timeout_sec)
  double waypoint_timeout_sec_{constants::defaults::kCommandWaypointTimeoutSec};

  bool follow_detections_enabled_{constants::defaults::kCommandFollowDetectionsEnabled};
  std::string follow_detections_topic_{constants::topics::kFollowDetections};
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr follow_detections_sub_;
  mutable std::mutex follow_mutex_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> follow_targets_;
  rclcpp::Time follow_last_update_{0, 0, RCL_ROS_TIME};

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

template<typename SrvT, typename HandlerFn>
void autonomy_ros::command::CommandInterface::registerService(
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
void autonomy_ros::command::CommandInterface::registerActionServer(
  std::shared_ptr<rclcpp_action::Server<ActionT>> & server,
  const char * action_name,
  GoalFn && goal_fn,
  void (CommandInterface::*execute)(
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

#endif  // AUTONOMY_ROS__COMMAND__COMMAND_INTERFACE_HPP_
