#include "autonomy_ros/command/command_interface.hpp"

#include "autonomy_ros/constants.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "autonomy_msgs/msg/error.hpp"
#include "autonomy_msgs/msg/event.hpp"
#include "autonomy_msgs/msg/task_state.hpp"
#include "autonomy_msgs/msg/task_type.hpp"
#include "autonomy_msgs/msg/waypoint_status.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy_ros/conversions/conversions.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace autonomy_ros::command
{

CommandInterface::CommandInterface(
  rclcpp::Node & node, task::TaskManager & task_manager, Autonomy & autonomy)
: node_(node), task_manager_(task_manager), autonomy_(autonomy)
{
  loadParameters();
  loadDocks();
}

void CommandInterface::loadParameters()
{
  node_.declare_parameter<std::string>(
    constants::params::kCommandDefaultDockId, default_dock_id_);
  node_.declare_parameter<double>(
    constants::params::kCommandWaypointTimeoutSec, waypoint_timeout_sec_);
  node_.declare_parameter<std::string>(
    constants::params::kCommandInitPoseTopic, init_pose_topic_);
  node_.declare_parameter<std::string>(
    constants::params::kCommandGoalPoseTopic, goal_pose_topic_);
  node_.declare_parameter<bool>(
    constants::params::kCommandEnableFollowDetections, follow_detections_enabled_);
  node_.declare_parameter<std::string>(
    constants::params::kCommandFollowDetectionsTopic, follow_detections_topic_);
  default_dock_id_ =
    node_.get_parameter(constants::params::kCommandDefaultDockId).as_string();
  waypoint_timeout_sec_ =
    node_.get_parameter(constants::params::kCommandWaypointTimeoutSec).as_double();
  init_pose_topic_ =
    node_.get_parameter(constants::params::kCommandInitPoseTopic).as_string();
  goal_pose_topic_ =
    node_.get_parameter(constants::params::kCommandGoalPoseTopic).as_string();
  follow_detections_enabled_ =
    node_.get_parameter(constants::params::kCommandEnableFollowDetections).as_bool();
  follow_detections_topic_ =
    node_.get_parameter(constants::params::kCommandFollowDetectionsTopic).as_string();
}

void CommandInterface::loadDocks()
{
  docks_.clear();
  node_.declare_parameter<double>(
    constants::params::kCommandDockX, constants::defaults::kCommandDockX);
  node_.declare_parameter<double>(
    constants::params::kCommandDockY, constants::defaults::kCommandDockY);
  node_.declare_parameter<double>(
    constants::params::kCommandDockW, constants::defaults::kCommandDockW);
  autonomy_msgs::msg::DockStation dock;
  dock.dock_id = default_dock_id_;
  dock.dock_type = constants::defaults::kDockType;
  dock.dock_pose.header.frame_id = constants::defaults::kDockFrame;
  dock.dock_pose.pose.position.x =
    node_.get_parameter(constants::params::kCommandDockX).as_double();
  dock.dock_pose.pose.position.y =
    node_.get_parameter(constants::params::kCommandDockY).as_double();
  dock.dock_pose.pose.orientation.w =
    node_.get_parameter(constants::params::kCommandDockW).as_double();
  dock.staging_pose = dock.dock_pose;
  docks_.push_back(dock);
}

void CommandInterface::start()
{
  autonomy_.addOdomListener([this](const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (msg) {
      task_manager_.updateOdom(*msg);
    }
  });

  initial_pose_pub_ =
    node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      constants::topics::kInitialPose, constants::defaults::kInitialPosePubDepth);

  init_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    init_pose_topic_, constants::defaults::kQueueDepth,
    std::bind(&CommandInterface::onInitPose, this, std::placeholders::_1));
  goal_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_, constants::defaults::kQueueDepth,
    std::bind(&CommandInterface::onGoalPose, this, std::placeholders::_1));

  startFollowTargetTracking();

  registerActionServer(navigate_pose_server_, constants::action_names::kNavigatePose,
    [this](const std::shared_ptr<const NavigatePose::Goal> & goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::NAVIGATION);
    },
    &CommandInterface::executeNavigatePose);

  registerActionServer(navigate_through_server_, constants::action_names::kNavigateThrough,
    [this](const std::shared_ptr<const NavigateThrough::Goal> & goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::WAYPOINTS);
    },
    &CommandInterface::executeNavigateThrough);

  registerActionServer(follow_server_, constants::action_names::kFollow,
    [this](const std::shared_ptr<const Follow::Goal> & goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::FOLLOW);
    },
    &CommandInterface::executeFollow);

  registerActionServer(guided_tour_server_, constants::action_names::kGuidedTour,
    [this](const std::shared_ptr<const GuidedTour::Goal> & goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::GUIDED_TOUR);
    },
    &CommandInterface::executeGuidedTour);

  registerActionServer(dock_server_, constants::action_names::kDock,
    [this](const std::shared_ptr<const Dock::Goal> & goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::DOCK);
    },
    &CommandInterface::executeDock);

  registerActionServer(teleop_server_, constants::action_names::kTeleop,
    [this](const std::shared_ptr<const Teleop::Goal> & goal) {
      return handleGoal(
        goal->task_id, autonomy_msgs::msg::TaskType::TELEOP, goal->preempt_other_tasks);
    },
    &CommandInterface::executeTeleop);

  registerService(cancel_task_srv_, constants::service_names::kCancelTask,
    [this](const std::shared_ptr<autonomy_msgs::srv::CancelTask::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::CancelTask::Response> res) {
      autonomy_.setControllerEnabled(false);
      res->success = task_manager_.cancelTask(
        req->task_id, req->cancel_all, req->task_type.value);
      res->status = task_manager_.getStatus();
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_.makeError(
          autonomy_msgs::msg::Error::INVALID_REQUEST, constants::msg::kCancelFailed);
    });

  registerService(get_status_srv_, constants::service_names::kGetTaskStatus,
    [this](const std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Response> res) {
      res->success = task_manager_.getStatusFor(req->task_id, res->status);
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_.makeError(
          autonomy_msgs::msg::Error::INVALID_REQUEST, constants::msg::kUnknownTaskId);
    });

  registerService(pause_task_srv_, constants::service_names::kPauseTask,
    [this](const std::shared_ptr<autonomy_msgs::srv::PauseTask::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::PauseTask::Response> res) {
      autonomy_.setControllerEnabled(false);
      res->success = task_manager_.pauseTask(req->reason);
      res->status = task_manager_.getStatus();
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_.makeError(
          autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kNoActiveTask);
    });

  registerService(resume_task_srv_, constants::service_names::kResumeTask,
    [this](const std::shared_ptr<autonomy_msgs::srv::ResumeTask::Request> &,
      std::shared_ptr<autonomy_msgs::srv::ResumeTask::Response> res) {
      res->success = task_manager_.resumeTask();
      if (res->success && !task_manager_.isEstop()) {
        const auto st = task_manager_.getStatus();
        if (st.task_type.value != autonomy_msgs::msg::TaskType::TELEOP) {
          autonomy_.setControllerEnabled(true);
        }
      }
      res->status = task_manager_.getStatus();
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_.makeError(
          autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kCannotResume);
    });

  registerService(continue_tour_srv_, constants::service_names::kContinueTour,
    [this](const std::shared_ptr<autonomy_msgs::srv::ContinueTour::Request> &,
      std::shared_ptr<autonomy_msgs::srv::ContinueTour::Response> res) {
      task_manager_.requestContinueTour();
      res->success = true;
      res->status = task_manager_.getStatus();
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
    });

  registerService(skip_exhibit_srv_, constants::service_names::kSkipToExhibit,
    [this](const std::shared_ptr<autonomy_msgs::srv::SkipToExhibit::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::SkipToExhibit::Response> res) {
      task_manager_.requestSkipExhibit(req->exhibit_index, req->exhibit_id);
      res->success = true;
      res->status = task_manager_.getStatus();
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
    });

  registerService(estop_srv_, constants::service_names::kTriggerEstop,
    [this](const std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Response> res) {
      if (req->engage) {
        autonomy_.setControllerEnabled(false);
        task_manager_.triggerEstop(req->reason);
      } else {
        task_manager_.releaseEstop();
        if (!task_manager_.isPaused()) {
          const auto st = task_manager_.getStatus();
          if (st.task_type.value != autonomy_msgs::msg::TaskType::TELEOP &&
            st.task_state.value == autonomy_msgs::msg::TaskState::RUNNING)
          {
            autonomy_.setControllerEnabled(true);
          }
        }
      }
      res->success = true;
      res->status = task_manager_.getStatus();
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
    });

  registerService(set_initial_pose_srv_, constants::service_names::kSetInitialPose,
    [this](const std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Response> res) {
      initial_pose_pub_->publish(req->pose);
      res->success = true;
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
    });

  registerService(set_teleop_mode_srv_, constants::service_names::kSetTeleopMode,
    [this](const std::shared_ptr<autonomy_msgs::srv::SetTeleopMode::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::SetTeleopMode::Response> res) {
      if (req->enable) {
        if (!task_manager_.beginTask(
            constants::msg::kTaskTeleopSrv, autonomy_msgs::msg::TaskType::TELEOP,
            constants::msg::kEmpty, req->preempt_other_tasks))
        {
          res->success = false;
          res->error = task_manager_.makeError(
            autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kBusy);
        } else {
          autonomy_.setControllerEnabled(false);
          res->success = true;
          res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
        }
      } else {
        task_manager_.cancelTask(constants::msg::kTaskTeleopSrv, false);
        if (!task_manager_.isEstop() && !task_manager_.isPaused() && task_manager_.hasActiveTask()) {
          autonomy_.setControllerEnabled(true);
        } else if (!task_manager_.hasActiveTask()) {
          autonomy_.setControllerEnabled(false);
        }
        res->success = true;
        res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
      }
      res->status = task_manager_.getStatus();
    });

  registerService(list_docks_srv_, constants::service_names::kListDocks,
    [this](const std::shared_ptr<autonomy_msgs::srv::ListDocks::Request> &,
      std::shared_ptr<autonomy_msgs::srv::ListDocks::Response> res) {
      res->success = true;
      res->docks = docks_;
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
    });

  RCLCPP_INFO(
    node_.get_logger(),
    "[command] external action/srv ready; topics init_pose=%s goal_pose=%s",
    init_pose_topic_.c_str(), goal_pose_topic_.c_str());
}

void CommandInterface::onInitPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  initial_pose_pub_->publish(*msg);
  RCLCPP_INFO(
    node_.get_logger(), "[command] init_pose -> /initialpose (%.2f, %.2f)",
    msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void CommandInterface::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (task_manager_.isEstop()) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal_pose: estop active");
    return;
  }
  if (!task_manager_.canBeginTask(
      constants::msg::kTaskGoalPose, autonomy_msgs::msg::TaskType::NAVIGATION, true))
  {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal_pose: another task is running");
    return;
  }
  geometry_msgs::msg::PoseStamped goal = *msg;
  std::thread{
    &CommandInterface::runTopicGoalPose, this, std::move(goal),
    std::string(constants::msg::kTaskGoalPose)}
    .detach();
  RCLCPP_INFO(
    node_.get_logger(), "[command] goal_pose accepted (%.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y);
}

void CommandInterface::runTopicGoalPose(
  geometry_msgs::msg::PoseStamped goal, const std::string & task_id)
{
  if (!task_manager_.beginTask(
      task_id, autonomy_msgs::msg::TaskType::NAVIGATION, constants::msg::kEmpty, true))
  {
    RCLCPP_WARN(node_.get_logger(), "[command] goal_pose task start failed");
    return;
  }
  autonomy_.setControllerEnabled(true);
  const bool ok = autonomy_.navigateToPose(
    goal, [&]() { return !task_manager_.ownsTask(task_id); });
  if (wasPreempted(task_id)) {
    task_manager_.endTask(
      autonomy_msgs::msg::TaskState::CANCELED,
      task_manager_.makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted),
      task_id);
  } else if (ok) {
    task_manager_.endTask(
      autonomy_msgs::msg::TaskState::SUCCEEDED,
      task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk),
      task_id);
    RCLCPP_INFO(node_.get_logger(), "[command] goal_pose navigation succeeded");
  } else {
    task_manager_.endTask(
      autonomy_msgs::msg::TaskState::FAILED,
      task_manager_.makeError(
        autonomy_msgs::msg::Error::TIMEOUT, constants::msg::kGoalPoseTimeout),
      task_id);
    RCLCPP_WARN(node_.get_logger(), "[command] goal_pose navigation failed or timed out");
  }
  if (!task_manager_.hasActiveTask() || task_manager_.isPaused() || task_manager_.isEstop()) {
    autonomy_.setControllerEnabled(false);
  }
}

rclcpp_action::GoalResponse CommandInterface::handleGoal(
  const std::string & task_id, uint8_t task_type, bool force_preempt) const
{
  if (task_manager_.isEstop()) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal %s: estop", task_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!task_manager_.canBeginTask(task_id, task_type, force_preempt)) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal %s: busy", task_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CommandInterface::handleCancel()
{
  autonomy_.setControllerEnabled(false);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CommandInterface::applySpeedLimit(float max_speed)
{
  if (max_speed <= 0.0f) {
    return;
  }
  ::autonomy::commsgs::planning_msgs::SpeedLimit limit;
  limit.header.stamp = ::autonomy::commsgs::builtin_interfaces::Time::Now();
  limit.percentage = false;
  limit.speed_limit = static_cast<double>(max_speed);
  autonomy_.applyControllerSpeedLimit(limit);
}

void CommandInterface::restoreSpeedLimit()
{
  autonomy_.clearControllerSpeedLimit();
}

double CommandInterface::distanceToPose(const geometry_msgs::msg::Pose & goal) const
{
  const auto status = task_manager_.getStatus();
  const double dx = goal.position.x - status.current_pose.pose.position.x;
  const double dy = goal.position.y - status.current_pose.pose.position.y;
  return std::hypot(dx, dy);
}

bool CommandInterface::reachedPose(const geometry_msgs::msg::Pose & goal, double tolerance) const
{
  return distanceToPose(goal) < tolerance;
}

bool CommandInterface::navigateToGoal(
  const std::string & task_id,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  double tolerance,
  double timeout_sec,
  const std::function<bool()> & extra_cancel)
{
  (void)tolerance;
  auto cancel = [&]() {
    if (wasPreempted(task_id)) {
      return true;
    }
    return extra_cancel && extra_cancel();
  };
  autonomy_.setControllerEnabled(true);
  return autonomy_.navigateToPose(goal_pose, cancel, timeout_sec);
}

bool CommandInterface::runDock(
  const std::string & task_id,
  const autonomy_msgs::action::Dock::Goal & goal,
  autonomy_msgs::action::Dock::Result & result,
  const std::function<bool()> & cancel_check)
{
  geometry_msgs::msg::PoseStamped dock_pose;
  geometry_msgs::msg::PoseStamped staging_pose;
  if (goal.use_dock_id) {
    bool found = false;
    for (const auto & d : docks_) {
      if (d.dock_id == goal.dock_id) {
        dock_pose = d.dock_pose;
        staging_pose = d.staging_pose;
        found = true;
        break;
      }
    }
    if (!found) {
      result.error = task_manager_.makeError(
        autonomy_msgs::msg::Error::DOCK_NOT_FOUND, goal.dock_id);
      return false;
    }
  } else {
    dock_pose = goal.dock.dock_pose;
    staging_pose = goal.dock.staging_pose;
  }

  const double tol = autonomy_.goalTolerance();
  if (goal.navigate_to_staging_pose) {
    if (!navigateToGoal(task_id, staging_pose, tol, goal.max_staging_time, cancel_check)) {
      result.error = task_manager_.makeError(
        autonomy_msgs::msg::Error::DOCK_STAGING_FAILED, constants::msg::kStagingFailed);
      return false;
    }
  }

  autonomy_.setNavigationGoal(dock_pose);
  bool docked = false;
  spinUntilCancel(
    task_id,
    [&]() { return cancel_check() || docked; },
    [&]() {
      if (reachedPose(dock_pose.pose, tol)) {
        docked = true;
      }
    });

  if (wasPreempted(task_id)) {
    result.error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
    return false;
  }
  if (!docked) {
    result.error = task_manager_.makeError(
      autonomy_msgs::msg::Error::DOCK_ALIGN_FAILED, constants::msg::kAlignFailed);
    return false;
  }
  result.success = true;
  result.charging = true;
  result.error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
  return true;
}

void CommandInterface::spinUntilCancel(
  const std::string & task_id,
  const std::function<bool()> & cancel_check,
  const std::function<void()> & on_tick)
{
  rclcpp::Rate rate(constants::defaults::kSpinRateHz);
  while (rclcpp::ok() && !cancel_check()) {
    if (!task_manager_.ownsTask(task_id)) {
      break;
    }
    if (task_manager_.isEstop()) {
      rate.sleep();
      continue;
    }
    while (task_manager_.isPaused() && rclcpp::ok() && !cancel_check() &&
      task_manager_.ownsTask(task_id))
    {
      rate.sleep();
    }
    if (!task_manager_.ownsTask(task_id)) {
      break;
    }
    on_tick();
    rate.sleep();
  }
}

bool CommandInterface::wasPreempted(const std::string & task_id) const
{
  return !task_manager_.ownsTask(task_id);
}

void CommandInterface::executeNavigatePose(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigatePose>> handle)
{
  const auto goal = handle->get_goal();
  const auto & task_id = goal->task_id;
  auto result = std::make_shared<NavigatePose::Result>();
  if (!goal->behavior_tree.empty()) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::NAV_GOAL_INVALID,
      constants::msg::kPerActionBtUnsupported);
    handle->abort(result);
    return;
  }
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::NAVIGATION)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kEstopOrBusy);
    handle->abort(result);
    return;
  }
  applySpeedLimit(goal->max_speed);
  autonomy_.setControllerEnabled(true);
  const auto nav_start = std::chrono::steady_clock::now();
  const bool ok = navigateToGoal(
    task_id, goal->goal, autonomy_.goalTolerance(), waypoint_timeout_sec_,
    [&]() { return handle->is_canceling(); });
  restoreSpeedLimit();
  if (auto path = autonomy_.lastPath()) {
    result->path = *path;
  }
  const auto nav_dur = std::chrono::steady_clock::now() - nav_start;
  result->navigation_time.sec = static_cast<int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(nav_dur).count());
  if (wasPreempted(task_id)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else if (ok) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } else if (handle->is_canceling()) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::TIMEOUT, constants::msg::kNavigatePoseTimeout);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
  }
}

void CommandInterface::executeNavigateThrough(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThrough>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<NavigateThrough::Result>();
  if (!goal->behavior_tree.empty()) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::NAV_GOAL_INVALID,
      constants::msg::kPerActionBtUnsupported);
    handle->abort(result);
    return;
  }
  if (goal->waypoints.empty()) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::WP_NO_VALID_WAYPOINTS, constants::msg::kEmptyWaypoints);
    handle->abort(result);
    return;
  }
  const auto & task_id = goal->task_id;
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::WAYPOINTS)) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kBusy);
    handle->abort(result);
    return;
  }
  autonomy_.setControllerEnabled(true);
  const double tol = autonomy_.goalTolerance();
  uint32_t completed = 0;
  try {
    for (uint32_t loop = 0; loop < goal->number_of_loops; ++loop) {
      for (uint32_t i = goal->start_index; i < goal->waypoints.size(); ++i) {
        if (handle->is_canceling()) {
          throw std::runtime_error(constants::msg::kSigCancel);
        }
        if (wasPreempted(task_id)) {
          throw std::runtime_error(constants::msg::kSigPreempt);
        }
        const auto & wp = goal->waypoints[i];
        const bool ok = navigateToGoal(
          task_id, wp.pose, tol, waypoint_timeout_sec_,
          [&]() { return handle->is_canceling(); });
        autonomy_msgs::msg::WaypointStatus ws;
        ws.waypoint_index = i;
        ws.waypoint_pose = wp.pose;
        if (ok) {
          ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::COMPLETED;
          result->waypoint_statuses.push_back(ws);
          ++completed;
          if (wp.wait_duration > 0) {
            const auto t0 = std::chrono::steady_clock::now();
            while (rclcpp::ok() && task_manager_.ownsTask(task_id) && !handle->is_canceling()) {
              if (std::chrono::steady_clock::now() - t0 >
                std::chrono::duration<double>(wp.wait_duration))
              {
                break;
              }
              rclcpp::sleep_for(
                std::chrono::milliseconds(constants::defaults::kWaypointWaitPollMs));
            }
          }
        } else {
          ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::FAILED;
          ws.error_code = autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT;
          ws.error_msg = constants::msg::kWaypointTimeoutOrPreempted;
          result->waypoint_statuses.push_back(ws);
          if (goal->stop_on_failure) {
            throw std::runtime_error(constants::msg::kSigWpFail);
          }
        }
        task_manager_.setProgress(
          static_cast<float>(i + 1) / static_cast<float>(goal->waypoints.size()));
      }
    }
    result->completed_count = completed;
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    if (what == constants::msg::kSigPreempt) {
      result->error = task_manager_.makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == constants::msg::kSigCancel) {
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == constants::msg::kSigWpFail) {
      result->error = task_manager_.makeError(
        autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT, constants::msg::kWaypointFailed);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
      handle->abort(result);
    }
  }
}

void CommandInterface::executeFollow(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> handle)
{
  const auto goal = handle->get_goal();
  const auto & task_id = goal->task_id;
  auto result = std::make_shared<Follow::Result>();
  if (!goal->target.use_target_pose && !goal->target.use_target_id) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::FOLLOW_TARGET_INVALID, constants::msg::kNoTarget);
    handle->abort(result);
    return;
  }
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::FOLLOW)) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kBusy);
    handle->abort(result);
    return;
  }
  applySpeedLimit(goal->target.max_linear_speed);
  autonomy_.setControllerEnabled(true);
  const auto start = std::chrono::steady_clock::now();
  const double follow_dist = goal->target.follow_distance;
  try {
    spinUntilCancel(
      task_id,
      [&]() { return handle->is_canceling(); },
      [&]() {
        auto fb = std::make_shared<Follow::Feedback>();
        fb->active_target = goal->target;
        fb->current_pose = task_manager_.getStatus().current_pose;
        fb->state.value = autonomy_msgs::msg::TaskState::RUNNING;
        if (goal->target.use_target_pose) {
          autonomy_.setNavigationGoal(goal->target.target_pose);
          const double d = distanceToPose(goal->target.target_pose.pose);
          fb->distance_to_target = static_cast<float>(d);
          if (d <= follow_dist) {
            throw std::runtime_error(constants::msg::kSigReached);
          }
        } else if (goal->target.use_target_id) {
          const auto target_pose = lookupFollowTargetPose(goal->target.target_id);
          if (!target_pose) {
            if (!followTargetTrackingAvailable()) {
              throw std::runtime_error(constants::msg::kSigTargetLost);
            }
            fb->distance_to_target = -1.0f;
          } else {
            autonomy_.setNavigationGoal(*target_pose);
            const double d = distanceToPose(target_pose->pose);
            fb->distance_to_target = static_cast<float>(d);
            if (d <= follow_dist) {
              throw std::runtime_error(constants::msg::kSigReached);
            }
          }
        }
        handle->publish_feedback(fb);
        if (goal->time_allowance.sec > 0 || goal->time_allowance.nanosec > 0) {
          const auto elapsed = std::chrono::steady_clock::now() - start;
          const auto limit = std::chrono::seconds(goal->time_allowance.sec) +
            std::chrono::nanoseconds(goal->time_allowance.nanosec);
          if (elapsed > limit) {
            throw std::runtime_error(constants::msg::kSigTimeout);
          }
        }
      });
  } catch (const std::runtime_error & e) {
    restoreSpeedLimit();
    const std::string what = e.what();
    if (what == constants::msg::kSigTargetLost) {
      result->error = task_manager_.makeError(
        autonomy_msgs::msg::Error::FOLLOW_TARGET_LOST,
        constants::msg::kTargetLostDetail);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
      handle->abort(result);
      return;
    }
    if (what == constants::msg::kSigTimeout || what == constants::msg::kSigReached) {
      const auto elapsed = std::chrono::steady_clock::now() - start;
      result->follow_duration.sec = static_cast<int32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
      handle->succeed(result);
      return;
    }
  }
  restoreSpeedLimit();
  if (wasPreempted(task_id)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  if (handle->is_canceling()) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  result->follow_duration.sec = 0;
  result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
  task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
  handle->succeed(result);
}

void CommandInterface::executeGuidedTour(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GuidedTour>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<GuidedTour::Result>();
  if (goal->exhibits.empty()) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TOUR_INVALID, constants::msg::kNoExhibits);
    handle->abort(result);
    return;
  }
  const auto & task_id = goal->task_id;
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::GUIDED_TOUR, goal->tour_id)) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kBusy);
    handle->abort(result);
    return;
  }
  task_manager_.publishEvent(autonomy_msgs::msg::Event::TOUR_STARTED, goal->tour_name);
  applySpeedLimit(goal->cruise_speed);
  autonomy_.setControllerEnabled(true);
  const double tol = autonomy_.goalTolerance();
  const auto tour_start = std::chrono::steady_clock::now();

  std::vector<std::string> exhibit_ids;
  exhibit_ids.reserve(goal->exhibits.size());
  for (const auto & ex : goal->exhibits) {
    exhibit_ids.push_back(ex.exhibit_id);
  }

  uint32_t completed = 0;
  try {
    for (uint32_t loop = 0; loop < goal->number_of_loops; ++loop) {
      uint32_t i = (loop == 0) ? goal->start_exhibit_index :
        (goal->start_from_beginning ? 0u : goal->start_exhibit_index);
      while (i < goal->exhibits.size()) {
        if (auto skip = task_manager_.consumeSkipExhibitForTour(exhibit_ids)) {
          i = *skip;
        }
        if (goal->auto_dock_on_low_battery &&
          task_manager_.isLowBattery(goal->low_battery_threshold))
        {
          task_manager_.publishEvent(
            autonomy_msgs::msg::Event::LOW_BATTERY, constants::msg::kEventAutoDock);
          autonomy_msgs::action::Dock::Goal dock_goal;
          dock_goal.task_id = task_id + constants::msg::kTaskDockSuffix;
          dock_goal.use_dock_id = true;
          dock_goal.dock_id = default_dock_id_;
          autonomy_msgs::action::Dock::Result dock_result;
          runDock(
            task_id, dock_goal, dock_result,
            [&]() { return handle->is_canceling(); });
          throw std::runtime_error(constants::msg::kSigLowBatteryDock);
        }
        if (handle->is_canceling()) {
          throw std::runtime_error(constants::msg::kSigCancel);
        }
        if (wasPreempted(task_id)) {
          throw std::runtime_error(constants::msg::kSigPreempt);
        }
        const auto & ex = goal->exhibits[i];
        geometry_msgs::msg::PoseStamped ps = ex.pose;
        task_manager_.setNarration(ex.exhibit_id, ex.narration_id);
        const bool arrived = navigateToGoal(
          task_id, ps, std::max(tol, static_cast<double>(ex.xy_tolerance)),
          waypoint_timeout_sec_, [&]() { return handle->is_canceling(); });

        if (wasPreempted(task_id)) {
          throw std::runtime_error(constants::msg::kSigPreempt);
        }
        if (handle->is_canceling()) {
          throw std::runtime_error(constants::msg::kSigCancel);
        }

        autonomy_msgs::msg::WaypointStatus ws;
        ws.waypoint_index = i;
        ws.waypoint_pose = ex.pose;
        if (!arrived) {
          ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::FAILED;
          ws.error_code = autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT;
          result->exhibit_statuses.push_back(ws);
          ++i;
          continue;
        }

        task_manager_.publishEvent(
          autonomy_msgs::msg::Event::ARRIVED_AT_EXHIBIT, ex.exhibit_name);
        task_manager_.setWaitingForContinue(true);
        bool waiting = ex.wait_for_continue;
        if (waiting) {
          while (rclcpp::ok() && !handle->is_canceling() && task_manager_.ownsTask(task_id) &&
            !task_manager_.consumeContinueTour())
          {
            auto fb = std::make_shared<GuidedTour::Feedback>();
            fb->tour_id = goal->tour_id;
            fb->current_exhibit_index = i;
            fb->current_exhibit_id = ex.exhibit_id;
            fb->current_narration_id = ex.narration_id;
            fb->current_pose = task_manager_.getStatus().current_pose;
            fb->progress = static_cast<float>(i + 1) / static_cast<float>(goal->exhibits.size());
            fb->waiting_for_continue = true;
            fb->state.value = autonomy_msgs::msg::TaskState::RUNNING;
            fb->distance_to_next = static_cast<float>(distanceToPose(ex.pose.pose));
            handle->publish_feedback(fb);
            rclcpp::sleep_for(
              std::chrono::milliseconds(constants::defaults::kTourFeedbackPollMs));
          }
        } else if (ex.dwell_duration > 0) {
          const auto t0 = std::chrono::steady_clock::now();
          while (rclcpp::ok() && !handle->is_canceling() && task_manager_.ownsTask(task_id)) {
            auto fb = std::make_shared<GuidedTour::Feedback>();
            fb->tour_id = goal->tour_id;
            fb->waiting_for_continue = false;
            fb->state.value = autonomy_msgs::msg::TaskState::RUNNING;
            fb->current_exhibit_index = i;
            handle->publish_feedback(fb);
            if (std::chrono::steady_clock::now() - t0 >
              std::chrono::duration<double>(ex.dwell_duration))
            {
              break;
            }
            rclcpp::sleep_for(
              std::chrono::milliseconds(constants::defaults::kTourFeedbackPollMs));
          }
        }
        if (wasPreempted(task_id)) {
          throw std::runtime_error(constants::msg::kSigPreempt);
        }
        task_manager_.setWaitingForContinue(false);
        task_manager_.publishEvent(autonomy_msgs::msg::Event::LEFT_EXHIBIT, ex.exhibit_name);
        ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::COMPLETED;
        result->exhibit_statuses.push_back(ws);
        ++completed;
        ++i;
      }
    }
    restoreSpeedLimit();
    result->completed_exhibits = completed;
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
    const auto dur = std::chrono::steady_clock::now() - tour_start;
    result->tour_duration.sec = static_cast<int32_t>(
      std::chrono::duration_cast<std::chrono::seconds>(dur).count());
    task_manager_.publishEvent(autonomy_msgs::msg::Event::TOUR_COMPLETED);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);

    if (goal->return_to_dock_on_complete && rclcpp::ok()) {
      autonomy_msgs::action::Dock::Goal dock_goal;
      dock_goal.task_id = task_id + constants::msg::kTaskDockSuffix;
      dock_goal.use_dock_id = true;
      dock_goal.dock_id = default_dock_id_;
      autonomy_msgs::action::Dock::Result dock_result;
      if (task_manager_.beginTask(dock_goal.task_id, autonomy_msgs::msg::TaskType::DOCK)) {
        autonomy_.setControllerEnabled(true);
        task_manager_.publishEvent(autonomy_msgs::msg::Event::DOCK_STARTED);
        if (runDock(dock_goal.task_id, dock_goal, dock_result, []() { return false; })) {
          task_manager_.publishEvent(autonomy_msgs::msg::Event::DOCK_COMPLETED);
          task_manager_.endTask(
            autonomy_msgs::msg::TaskState::SUCCEEDED, dock_result.error, dock_goal.task_id);
        } else {
          task_manager_.endTask(
            autonomy_msgs::msg::TaskState::FAILED, dock_result.error, dock_goal.task_id);
        }
      }
    }
  } catch (const std::runtime_error & e) {
    restoreSpeedLimit();
    const std::string what = e.what();
    if (what == constants::msg::kSigPreempt) {
      result->error = task_manager_.makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == constants::msg::kSigCancel) {
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == constants::msg::kSigLowBatteryDock) {
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kLowBatteryDock);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
      handle->succeed(result);
    }
  }
}

void CommandInterface::executeDock(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Dock>> handle)
{
  const auto goal = handle->get_goal();
  const auto & task_id = goal->task_id;
  auto result = std::make_shared<Dock::Result>();
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::DOCK)) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kBusy);
    handle->abort(result);
    return;
  }
  task_manager_.publishEvent(autonomy_msgs::msg::Event::DOCK_STARTED);
  autonomy_.setControllerEnabled(true);
  if (runDock(task_id, *goal, *result, [&]() { return handle->is_canceling(); })) {
    task_manager_.publishEvent(autonomy_msgs::msg::Event::DOCK_COMPLETED);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
    return;
  }
  if (wasPreempted(task_id)) {
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else if (handle->is_canceling()) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else {
    task_manager_.endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
  }
}

void CommandInterface::executeTeleop(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Teleop>> handle)
{
  const auto goal = handle->get_goal();
  const auto & task_id = goal->task_id;
  auto result = std::make_shared<Teleop::Result>();
  if (!task_manager_.beginTask(
      task_id, autonomy_msgs::msg::TaskType::TELEOP,
      constants::msg::kEmpty, goal->preempt_other_tasks))
  {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kBusy);
    handle->abort(result);
    return;
  }
  autonomy_.setControllerEnabled(false);
  const auto start = std::chrono::steady_clock::now();
  try {
    spinUntilCancel(
      task_id,
      [&]() { return handle->is_canceling(); },
      [&]() {
        auto fb = std::make_shared<Teleop::Feedback>();
        const auto elapsed = std::chrono::steady_clock::now() - start;
        fb->current_teleop_duration.sec = static_cast<int32_t>(
          std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
        fb->state.value = autonomy_msgs::msg::TaskState::RUNNING;
        handle->publish_feedback(fb);
        if (goal->time_allowance.sec > 0 || goal->time_allowance.nanosec > 0) {
          const auto limit = std::chrono::seconds(goal->time_allowance.sec) +
            std::chrono::nanoseconds(goal->time_allowance.nanosec);
          if (elapsed > limit) {
            throw std::runtime_error(constants::msg::kSigTimeout);
          }
        }
      });
  } catch (const std::runtime_error & e) {
    if (std::string(e.what()) == constants::msg::kSigTimeout) {
      const auto elapsed = std::chrono::steady_clock::now() - start;
      result->total_elapsed_time.sec = static_cast<int32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
      task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
      handle->succeed(result);
      return;
    }
  }
  if (wasPreempted(task_id)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TELEOP_PREEMPTED, constants::msg::kPreempted);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  if (handle->is_canceling()) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  result->total_elapsed_time.sec = static_cast<int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
  result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
  task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
  handle->succeed(result);
}

void CommandInterface::startFollowTargetTracking()
{
  if (!follow_detections_enabled_) {
    RCLCPP_INFO(node_.get_logger(), "[command] follow detections disabled");
    return;
  }
  follow_detections_sub_ = node_.create_subscription<vision_msgs::msg::Detection3DArray>(
    follow_detections_topic_, constants::defaults::kQueueDepth,
    std::bind(&CommandInterface::onFollowDetections, this, std::placeholders::_1));
  RCLCPP_INFO(
    node_.get_logger(), "[command] follow detections %s", follow_detections_topic_.c_str());
}

geometry_msgs::msg::PoseStamped CommandInterface::poseFromDetection(
  const vision_msgs::msg::Detection3D & detection)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = detection.header;
  if (!detection.results.empty()) {
    pose.pose = detection.results.front().pose.pose;
    return pose;
  }
  pose.pose = detection.bbox.center;
  return pose;
}

void CommandInterface::onFollowDetections(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(follow_mutex_);
  follow_last_update_ = node_.now();
  for (const auto & det : msg->detections) {
    auto pose = poseFromDetection(det);
    if (pose.header.frame_id.empty()) {
      continue;
    }
    if (!det.id.empty()) {
      follow_targets_[det.id] = pose;
    }
    for (const auto & hyp : det.results) {
      if (!hyp.hypothesis.class_id.empty()) {
        follow_targets_[hyp.hypothesis.class_id] = pose;
      }
    }
  }
}

bool CommandInterface::followTargetTrackingAvailable() const
{
  std::lock_guard<std::mutex> lock(follow_mutex_);
  if (follow_last_update_.nanoseconds() == 0) {
    return false;
  }
  return (node_.now() - follow_last_update_).seconds() <
    constants::defaults::kFollowDetectionStaleSec;
}

std::optional<geometry_msgs::msg::PoseStamped> CommandInterface::lookupFollowTargetPose(
  const std::string & target_id) const
{
  if (target_id.empty()) {
    return std::nullopt;
  }
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(follow_mutex_);
    const auto it = follow_targets_.find(target_id);
    if (it == follow_targets_.end()) {
      return std::nullopt;
    }
    pose = it->second;
  }

  const auto & global_frame = autonomy_.globalFrame();
  if (pose.header.frame_id.empty() || pose.header.frame_id == global_frame) {
    pose.header.frame_id = global_frame;
    return pose;
  }

  auto * planner = autonomy_.core().planner_server();
  if (!planner) {
    return pose;
  }
  auto wrapper = planner->GetCostmapWrapper();
  if (!wrapper) {
    return pose;
  }
  auto input = conversions::fromRos(pose);
  ::autonomy::commsgs::geometry_msgs::PoseStamped transformed;
  if (wrapper->transformPoseToGlobalFrame(input, transformed)) {
    return conversions::toRos(transformed);
  }
  return pose;
}

}  // namespace autonomy_ros::command
