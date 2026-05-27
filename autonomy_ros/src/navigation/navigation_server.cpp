#include "autonomy_ros/navigation/navigation_server.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autonomy_msgs/msg/error.hpp"
#include "autonomy_msgs/msg/task_state.hpp"
#include "autonomy_msgs/msg/task_type.hpp"
#include "autonomy_msgs/msg/waypoint_status.hpp"
#include "autonomy_ros/viz/visualizer.hpp"

namespace autonomy_ros::navigation
{

using system::AutonomyCoreOptions;

NavigationServer::NavigationServer(
  rclcpp::Node & node,
  ::autonomy::system::Autonomy & core,
  const AutonomyCoreOptions & core_options,
  TaskManager::StopMotionFn stop_motion,
  viz::Visualizer * visualizer)
: node_(node)
, visualizer_(visualizer)
{
  task_manager_ = std::make_unique<TaskManager>(
    node_, &core, core_options, std::move(stop_motion));
  loadParameters();

  initial_pose_pub_ =
    node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      constants::topics::kInitialPose, constants::defaults::kInitialPosePubDepth);

  init_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    init_pose_topic_, constants::defaults::kQueueDepth,
    std::bind(&NavigationServer::onInitPose, this, std::placeholders::_1));
  goal_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_, constants::defaults::kQueueDepth,
    std::bind(&NavigationServer::onGoalPose, this, std::placeholders::_1));

  registerActionServer(navigate_pose_server_, constants::action_names::kNavigatePose,
    [this](const std::shared_ptr<const NavigatePose::Goal> & goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::NAVIGATION);
    },
    &NavigationServer::executeNavigatePose);

  registerActionServer(navigate_through_server_, constants::action_names::kNavigateThrough,
    [this](const std::shared_ptr<const NavigateThrough::Goal> & goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::WAYPOINTS);
    },
    &NavigationServer::executeNavigateThrough);

  registerService(cancel_task_srv_, constants::service_names::kCancelTask,
    [this](const std::shared_ptr<autonomy_msgs::srv::CancelTask::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::CancelTask::Response> res) {
      task_manager_->setControllerEnabled(false);
      res->success = task_manager_->cancelTask(
        req->task_id, req->cancel_all, req->task_type.value);
      res->status = task_manager_->getStatus();
      res->error = res->success ?
        task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_->makeError(
          autonomy_msgs::msg::Error::INVALID_REQUEST, constants::msg::kCancelFailed);
    });

  registerService(get_status_srv_, constants::service_names::kGetTaskStatus,
    [this](const std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Response> res) {
      res->success = task_manager_->getStatusFor(req->task_id, res->status);
      res->error = res->success ?
        task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_->makeError(
          autonomy_msgs::msg::Error::INVALID_REQUEST, constants::msg::kUnknownTaskId);
    });

  registerService(pause_task_srv_, constants::service_names::kPauseTask,
    [this](const std::shared_ptr<autonomy_msgs::srv::PauseTask::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::PauseTask::Response> res) {
      task_manager_->setControllerEnabled(false);
      res->success = task_manager_->pauseTask(req->reason);
      res->status = task_manager_->getStatus();
      res->error = res->success ?
        task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_->makeError(
          autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kNoActiveTask);
    });

  registerService(resume_task_srv_, constants::service_names::kResumeTask,
    [this](const std::shared_ptr<autonomy_msgs::srv::ResumeTask::Request> &,
      std::shared_ptr<autonomy_msgs::srv::ResumeTask::Response> res) {
      res->success = task_manager_->resumeTask();
      if (res->success && !task_manager_->isEstop()) {
        task_manager_->setControllerEnabled(true);
      }
      res->status = task_manager_->getStatus();
      res->error = res->success ?
        task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty) :
        task_manager_->makeError(
          autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kCannotResume);
    });

  registerService(estop_srv_, constants::service_names::kTriggerEstop,
    [this](const std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Response> res) {
      if (req->engage) {
        task_manager_->setControllerEnabled(false);
        task_manager_->triggerEstop(req->reason);
      } else {
        task_manager_->releaseEstop();
        if (!task_manager_->isPaused() && task_manager_->hasActiveTask()) {
          task_manager_->setControllerEnabled(true);
        }
      }
      res->success = true;
      res->status = task_manager_->getStatus();
      res->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
    });

  registerService(set_initial_pose_srv_, constants::service_names::kSetInitialPose,
    [this](const std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Response> res) {
      initial_pose_pub_->publish(req->pose);
      res->success = true;
      res->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kEmpty);
    });

  RCLCPP_INFO(
    node_.get_logger(),
    "[command] navigate_pose + navigate_through; topics init_pose=%s goal_pose=%s",
    init_pose_topic_.c_str(), goal_pose_topic_.c_str());
}

void NavigationServer::updateOdom(const nav_msgs::msg::Odometry & odom)
{
  if (task_manager_) {
    task_manager_->updateOdom(odom);
  }
}

bool NavigationServer::hasOdometry() const
{
  return task_manager_ && task_manager_->hasOdometry();
}

bool NavigationServer::hasActiveNavigationTask() const
{
  return task_manager_ && task_manager_->hasActiveTask();
}

bool NavigationServer::isControllerEnabled() const
{
  return task_manager_ && task_manager_->isControllerEnabled();
}

void NavigationServer::loadParameters()
{
  node_.declare_parameter<double>(
    constants::params::kNavigationWaypointTimeoutSec, waypoint_timeout_sec_);
  node_.declare_parameter<std::string>(
    constants::params::kNavigationInitPoseTopic, init_pose_topic_);
  node_.declare_parameter<std::string>(
    constants::params::kNavigationGoalPoseTopic, goal_pose_topic_);
  waypoint_timeout_sec_ =
    node_.get_parameter(constants::params::kNavigationWaypointTimeoutSec).as_double();
  init_pose_topic_ =
    node_.get_parameter(constants::params::kNavigationInitPoseTopic).as_string();
  goal_pose_topic_ =
    node_.get_parameter(constants::params::kNavigationGoalPoseTopic).as_string();
}

void NavigationServer::onInitPose(
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

void NavigationServer::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (task_manager_->isEstop()) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal_pose: estop active");
    return;
  }
  if (!task_manager_->canBeginTask(
      constants::msg::kTaskGoalPose, autonomy_msgs::msg::TaskType::NAVIGATION, true))
  {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal_pose: another task is running");
    return;
  }
  geometry_msgs::msg::PoseStamped goal = *msg;
  if (visualizer_) {
    visualizer_->onNavigationGoal(goal);
  }
  std::thread{
    &NavigationServer::runTopicGoalPose, this, std::move(goal),
    std::string(constants::msg::kTaskGoalPose)}
    .detach();
  RCLCPP_INFO(
    node_.get_logger(), "[command] goal_pose accepted (%.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y);
}

void NavigationServer::runTopicGoalPose(
  geometry_msgs::msg::PoseStamped goal, const std::string & task_id)
{
  if (!task_manager_->beginTask(
      task_id, autonomy_msgs::msg::TaskType::NAVIGATION, true))
  {
    RCLCPP_WARN(node_.get_logger(), "[command] goal_pose task start failed");
    return;
  }
  task_manager_->setControllerEnabled(true);
  const bool ok = navigateToGoal(
    task_id, goal, waypoint_timeout_sec_,
    [&]() { return !task_manager_->ownsTask(task_id); });
  if (wasPreempted(task_id)) {
    task_manager_->endTask(
      autonomy_msgs::msg::TaskState::CANCELED,
      task_manager_->makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted),
      task_id);
  } else if (ok) {
    task_manager_->endTask(
      autonomy_msgs::msg::TaskState::SUCCEEDED,
      task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk),
      task_id);
    RCLCPP_INFO(node_.get_logger(), "[command] goal_pose navigation succeeded");
  } else {
    task_manager_->endTask(
      autonomy_msgs::msg::TaskState::FAILED,
      task_manager_->makeError(
        autonomy_msgs::msg::Error::TIMEOUT, constants::msg::kGoalPoseTimeout),
      task_id);
    RCLCPP_WARN(node_.get_logger(), "[command] goal_pose navigation failed or timed out");
  }
  if (!task_manager_->hasActiveTask() || task_manager_->isPaused() || task_manager_->isEstop()) {
    task_manager_->setControllerEnabled(false);
  }
}

rclcpp_action::GoalResponse NavigationServer::handleGoal(
  const std::string & task_id, const uint8_t task_type) const
{
  if (task_manager_->isEstop()) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal %s: estop", task_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!task_manager_->canBeginTask(task_id, task_type, false)) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal %s: busy", task_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationServer::handleCancel()
{
  task_manager_->setControllerEnabled(false);
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool NavigationServer::wasPreempted(const std::string & task_id) const
{
  return !task_manager_->ownsTask(task_id);
}

bool NavigationServer::navigateToGoal(
  const std::string & task_id,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const double timeout_sec,
  const std::function<bool()> & extra_cancel)
{
  auto cancel = [&]() {
    if (wasPreempted(task_id)) {
      return true;
    }
    return extra_cancel && extra_cancel();
  };
  task_manager_->setControllerEnabled(true);
  return task_manager_->navigateToPose(goal_pose, cancel, timeout_sec);
}

void NavigationServer::executeNavigatePose(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigatePose>> handle)
{
  const auto goal = handle->get_goal();
  const auto & task_id = goal->task_id;
  auto result = std::make_shared<NavigatePose::Result>();
  if (!goal->behavior_tree.empty()) {
    result->error = task_manager_->makeError(
      autonomy_msgs::msg::Error::NAV_GOAL_INVALID,
      constants::msg::kPerActionBtUnsupported);
    handle->abort(result);
    return;
  }
  if (!task_manager_->beginTask(task_id, autonomy_msgs::msg::TaskType::NAVIGATION)) {
    result->error = task_manager_->makeError(
      autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kEstopOrBusy);
    handle->abort(result);
    return;
  }
  const auto nav_start = std::chrono::steady_clock::now();
  const bool ok = navigateToGoal(
    task_id, goal->goal, waypoint_timeout_sec_,
    [&]() { return handle->is_canceling(); });
  if (auto path = task_manager_->lastPath()) {
    result->path = *path;
  }
  const auto nav_dur = std::chrono::steady_clock::now() - nav_start;
  result->navigation_time.sec = static_cast<int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(nav_dur).count());

  if (wasPreempted(task_id)) {
    result->error = task_manager_->makeError(
      autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
    task_manager_->endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else if (ok) {
    result->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
    task_manager_->endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } else if (handle->is_canceling()) {
    result->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
    task_manager_->endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else {
    result->error = task_manager_->makeError(
      autonomy_msgs::msg::Error::TIMEOUT, constants::msg::kNavigatePoseTimeout);
    task_manager_->endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
  }
}

void NavigationServer::executeNavigateThrough(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThrough>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<NavigateThrough::Result>();
  if (!goal->behavior_tree.empty()) {
    result->error = task_manager_->makeError(
      autonomy_msgs::msg::Error::NAV_GOAL_INVALID,
      constants::msg::kPerActionBtUnsupported);
    handle->abort(result);
    return;
  }
  if (goal->waypoints.empty()) {
    result->error = task_manager_->makeError(
      autonomy_msgs::msg::Error::WP_NO_VALID_WAYPOINTS, constants::msg::kEmptyWaypoints);
    handle->abort(result);
    return;
  }
  const auto & task_id = goal->task_id;
  if (!task_manager_->beginTask(task_id, autonomy_msgs::msg::TaskType::WAYPOINTS)) {
    result->error = task_manager_->makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, constants::msg::kBusy);
    handle->abort(result);
    return;
  }
  task_manager_->setControllerEnabled(true);
  uint32_t completed = 0;

  const bool use_behavior_tree_through =
    task_manager_->coreOptions().use_bt_navigation &&
    goal->number_of_loops <= 1;

  if (use_behavior_tree_through) {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(goal->waypoints.size() - goal->start_index);
    for (uint32_t i = goal->start_index; i < goal->waypoints.size(); ++i) {
      poses.push_back(goal->waypoints[i].pose);
    }
    const double timeout = waypoint_timeout_sec_ *
      static_cast<double>(std::max<std::size_t>(poses.size(), 1));
    const bool ok = task_manager_->navigateThroughPoses(
      poses,
      [&]() { return handle->is_canceling() || wasPreempted(task_id); },
      timeout);
    for (uint32_t i = 0; i < goal->waypoints.size(); ++i) {
      autonomy_msgs::msg::WaypointStatus ws;
      ws.waypoint_index = i;
      ws.waypoint_pose = goal->waypoints[i].pose;
      if (i < goal->start_index) {
        ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::SKIPPED;
      } else if (ok) {
        ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::COMPLETED;
        ++completed;
      } else {
        ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::FAILED;
        ws.error_code = autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT;
        ws.error_msg = constants::msg::kWaypointTimeoutOrPreempted;
      }
      result->waypoint_statuses.push_back(ws);
    }
    result->completed_count = completed;
    if (wasPreempted(task_id)) {
      result->error = task_manager_->makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
      task_manager_->endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
      return;
    }
    if (ok) {
      result->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
      task_manager_->endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
      handle->succeed(result);
      return;
    }
    if (handle->is_canceling()) {
      result->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
      task_manager_->endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
      return;
    }
    result->error = task_manager_->makeError(
      autonomy_msgs::msg::Error::TIMEOUT, constants::msg::kNavigatePoseTimeout);
    task_manager_->endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
    return;
  }

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
          task_id, wp.pose, waypoint_timeout_sec_,
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
            while (rclcpp::ok() && task_manager_->ownsTask(task_id) && !handle->is_canceling()) {
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
        task_manager_->setProgress(
          static_cast<float>(i + 1) / static_cast<float>(goal->waypoints.size()));
      }
    }
    result->completed_count = completed;
    result->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kOk);
    task_manager_->endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    if (what == constants::msg::kSigPreempt) {
      result->error = task_manager_->makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, constants::msg::kPreempted);
      task_manager_->endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == constants::msg::kSigCancel) {
      result->error = task_manager_->makeError(autonomy_msgs::msg::Error::NONE, constants::msg::kCanceled);
      task_manager_->endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == constants::msg::kSigWpFail) {
      result->error = task_manager_->makeError(
        autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT, constants::msg::kWaypointFailed);
      task_manager_->endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
      handle->abort(result);
    }
  }
}

}  // namespace autonomy_ros::navigation
