#include "autonomy_ros/command/command_interface.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "autonomy_msgs/msg/error.hpp"
#include "autonomy_msgs/msg/event.hpp"
#include "autonomy_msgs/msg/task_state.hpp"
#include "autonomy_msgs/msg/task_type.hpp"
#include "autonomy_msgs/msg/waypoint_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace autonomy_ros::command
{

CommandInterface::CommandInterface(
  rclcpp::Node & node, task::TaskManager & task_manager,
  planner::Planner & planner, controller::Controller & controller)
: node_(node), task_manager_(task_manager), planner_(planner), controller_(controller)
{
  loadParameters();
  loadDocks();
}

void CommandInterface::loadParameters()
{
  node_.declare_parameter<std::string>("command.default_dock_id", default_dock_id_);
  node_.declare_parameter<double>("command.waypoint_timeout_sec", waypoint_timeout_sec_);
  default_dock_id_ = node_.get_parameter("command.default_dock_id").as_string();
  waypoint_timeout_sec_ = node_.get_parameter("command.waypoint_timeout_sec").as_double();
}

void CommandInterface::loadDocks()
{
  docks_.clear();
  node_.declare_parameter<double>("command.dock_x", 0.5);
  node_.declare_parameter<double>("command.dock_y", 0.0);
  node_.declare_parameter<double>("command.dock_w", 1.0);
  autonomy_msgs::msg::DockStation dock;
  dock.dock_id = default_dock_id_;
  dock.dock_type = "charger";
  dock.dock_pose.header.frame_id = "odom";
  dock.dock_pose.pose.position.x = node_.get_parameter("command.dock_x").as_double();
  dock.dock_pose.pose.position.y = node_.get_parameter("command.dock_y").as_double();
  dock.dock_pose.pose.orientation.w = node_.get_parameter("command.dock_w").as_double();
  dock.staging_pose = dock.dock_pose;
  docks_.push_back(dock);
}

void CommandInterface::start()
{
  const auto node = node_.shared_from_this();

  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      task_manager_.updateOdom(*msg);
    });
  initial_pose_pub_ =
    node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);

  auto cancel_cb = [this](auto &&) { return handleCancel(); };

  navigate_pose_server_ = rclcpp_action::create_server<NavigatePose>(
    node, "autonomy/navigate_pose",
    [this](auto, auto goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::NAVIGATION);
    },
    cancel_cb,
    [this](const auto & handle) {
      std::thread{&CommandInterface::executeNavigatePose, this, handle}.detach();
    });

  navigate_through_server_ = rclcpp_action::create_server<NavigateThrough>(
    node, "autonomy/navigate_through",
    [this](auto, auto goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::WAYPOINTS);
    },
    cancel_cb,
    [this](const auto & handle) {
      std::thread{&CommandInterface::executeNavigateThrough, this, handle}.detach();
    });

  follow_server_ = rclcpp_action::create_server<Follow>(
    node, "autonomy/follow",
    [this](auto, auto goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::FOLLOW);
    },
    cancel_cb,
    [this](const auto & handle) {
      std::thread{&CommandInterface::executeFollow, this, handle}.detach();
    });

  guided_tour_server_ = rclcpp_action::create_server<GuidedTour>(
    node, "autonomy/guided_tour",
    [this](auto, auto goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::GUIDED_TOUR);
    },
    cancel_cb,
    [this](const auto & handle) {
      std::thread{&CommandInterface::executeGuidedTour, this, handle}.detach();
    });

  dock_server_ = rclcpp_action::create_server<Dock>(
    node, "autonomy/dock",
    [this](auto, auto goal) {
      return handleGoal(goal->task_id, autonomy_msgs::msg::TaskType::DOCK);
    },
    cancel_cb,
    [this](const auto & handle) {
      std::thread{&CommandInterface::executeDock, this, handle}.detach();
    });

  teleop_server_ = rclcpp_action::create_server<Teleop>(
    node, "autonomy/teleop",
    [this](auto, auto goal) {
      return handleGoal(
        goal->task_id, autonomy_msgs::msg::TaskType::TELEOP, goal->preempt_other_tasks);
    },
    cancel_cb,
    [this](const auto & handle) {
      std::thread{&CommandInterface::executeTeleop, this, handle}.detach();
    });

  cancel_task_srv_ = node_.create_service<autonomy_msgs::srv::CancelTask>(
    "autonomy/cancel_task",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::CancelTask::Request> req,
      std::shared_ptr<autonomy_msgs::srv::CancelTask::Response> res) {
      controller_.setEnabled(false);
      res->success = task_manager_.cancelTask(
        req->task_id, req->cancel_all, req->task_type.value);
      res->status = task_manager_.getStatus();
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_.makeError(autonomy_msgs::msg::Error::INVALID_REQUEST, "cancel failed");
    });

  get_status_srv_ = node_.create_service<autonomy_msgs::srv::GetTaskStatus>(
    "autonomy/get_task_status",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Request> req,
      std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Response> res) {
      res->success = task_manager_.getStatusFor(req->task_id, res->status);
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_.makeError(
          autonomy_msgs::msg::Error::INVALID_REQUEST, "unknown task_id");
    });

  pause_task_srv_ = node_.create_service<autonomy_msgs::srv::PauseTask>(
    "autonomy/pause_task",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::PauseTask::Request> req,
      std::shared_ptr<autonomy_msgs::srv::PauseTask::Response> res) {
      controller_.setEnabled(false);
      res->success = task_manager_.pauseTask(req->reason);
      res->status = task_manager_.getStatus();
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "no active task");
    });

  resume_task_srv_ = node_.create_service<autonomy_msgs::srv::ResumeTask>(
    "autonomy/resume_task",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::ResumeTask::Request> req,
      std::shared_ptr<autonomy_msgs::srv::ResumeTask::Response> res) {
      (void)req;
      res->success = task_manager_.resumeTask();
      if (res->success && !task_manager_.isEstop()) {
        const auto st = task_manager_.getStatus();
        if (st.task_type.value != autonomy_msgs::msg::TaskType::TELEOP) {
          controller_.setEnabled(true);
        }
      }
      res->status = task_manager_.getStatus();
      res->error = res->success ?
        task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "cannot resume");
    });

  continue_tour_srv_ = node_.create_service<autonomy_msgs::srv::ContinueTour>(
    "autonomy/continue_tour",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::ContinueTour::Request> req,
      std::shared_ptr<autonomy_msgs::srv::ContinueTour::Response> res) {
      (void)req;
      task_manager_.requestContinueTour();
      res->success = true;
      res->status = task_manager_.getStatus();
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "");
    });

  skip_exhibit_srv_ = node_.create_service<autonomy_msgs::srv::SkipToExhibit>(
    "autonomy/skip_to_exhibit",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::SkipToExhibit::Request> req,
      std::shared_ptr<autonomy_msgs::srv::SkipToExhibit::Response> res) {
      task_manager_.requestSkipExhibit(req->exhibit_index, req->exhibit_id);
      res->success = true;
      res->status = task_manager_.getStatus();
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "");
    });

  estop_srv_ = node_.create_service<autonomy_msgs::srv::TriggerEmergencyStop>(
    "autonomy/trigger_estop",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Request> req,
      std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Response> res) {
      if (req->engage) {
        controller_.setEnabled(false);
        task_manager_.triggerEstop(req->reason);
      } else {
        task_manager_.releaseEstop();
        if (!task_manager_.isPaused()) {
          const auto st = task_manager_.getStatus();
          if (st.task_type.value != autonomy_msgs::msg::TaskType::TELEOP &&
            st.task_state.value == autonomy_msgs::msg::TaskState::RUNNING)
          {
            controller_.setEnabled(true);
          }
        }
      }
      res->success = true;
      res->status = task_manager_.getStatus();
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "");
    });

  set_initial_pose_srv_ = node_.create_service<autonomy_msgs::srv::SetInitialPose>(
    "autonomy/set_initial_pose",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Request> req,
      std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Response> res) {
      initial_pose_pub_->publish(req->pose);
      res->success = true;
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "");
    });

  set_teleop_mode_srv_ = node_.create_service<autonomy_msgs::srv::SetTeleopMode>(
    "autonomy/set_teleop_mode",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::SetTeleopMode::Request> req,
      std::shared_ptr<autonomy_msgs::srv::SetTeleopMode::Response> res) {
      if (req->enable) {
        if (!task_manager_.beginTask(
            "teleop_srv", autonomy_msgs::msg::TaskType::TELEOP, "", req->preempt_other_tasks))
        {
          res->success = false;
          res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "busy");
        } else {
          controller_.setEnabled(false);
          res->success = true;
          res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "");
        }
      } else {
        task_manager_.cancelTask("teleop_srv", false);
        if (!task_manager_.isEstop() && !task_manager_.isPaused() && task_manager_.hasActiveTask()) {
          controller_.setEnabled(true);
        } else if (!task_manager_.hasActiveTask()) {
          controller_.setEnabled(false);
        }
        res->success = true;
        res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "");
      }
      res->status = task_manager_.getStatus();
    });

  list_docks_srv_ = node_.create_service<autonomy_msgs::srv::ListDocks>(
    "autonomy/list_docks",
    [this](
      const std::shared_ptr<autonomy_msgs::srv::ListDocks::Request> req,
      std::shared_ptr<autonomy_msgs::srv::ListDocks::Response> res) {
      (void)req;
      res->success = true;
      res->docks = docks_;
      res->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "");
    });

  RCLCPP_INFO(node_.get_logger(), "[command] external action/srv interface ready under autonomy/*");
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
  controller_.setEnabled(false);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CommandInterface::applySpeedLimit(float max_speed)
{
  if (max_speed > 0.0f) {
    controller_.setMaxLinearVel(static_cast<double>(max_speed));
  }
}

void CommandInterface::restoreSpeedLimit()
{
  controller_.setMaxLinearVel(0.0);
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
  planner_.setGoal(goal_pose);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_sec);
  bool reached = false;
  bool timed_out = false;
  spinUntilCancel(
    task_id,
    [&]() {
      if (extra_cancel && extra_cancel()) {
        return true;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        timed_out = true;
        return true;
      }
      return reached;
    },
    [&]() {
      if (reachedPose(goal_pose.pose, tolerance)) {
        reached = true;
      }
    });
  if (wasPreempted(task_id)) {
    return false;
  }
  if (timed_out && !reached) {
    return false;
  }
  return reached;
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

  const double tol = controller_.goalTolerance();
  if (goal.navigate_to_staging_pose) {
    if (!navigateToGoal(task_id, staging_pose, tol, goal.max_staging_time, cancel_check)) {
      result.error = task_manager_.makeError(
        autonomy_msgs::msg::Error::DOCK_STAGING_FAILED, "staging");
      return false;
    }
  }

  planner_.setGoal(dock_pose);
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
    result.error = task_manager_.makeError(autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
    return false;
  }
  if (!docked) {
    result.error = task_manager_.makeError(autonomy_msgs::msg::Error::DOCK_ALIGN_FAILED, "align");
    return false;
  }
  result.success = true;
  result.charging = true;
  result.error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
  return true;
}

void CommandInterface::spinUntilCancel(
  const std::string & task_id,
  const std::function<bool()> & cancel_check,
  const std::function<void()> & on_tick)
{
  rclcpp::Rate rate(10);
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
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::NAVIGATION)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::NOT_AVAILABLE, "estop or busy");
    handle->abort(result);
    return;
  }
  applySpeedLimit(goal->max_speed);
  controller_.setEnabled(true);
  const double tol = controller_.goalTolerance();
  const auto nav_start = std::chrono::steady_clock::now();
  const bool ok = navigateToGoal(
    task_id, goal->goal, tol, waypoint_timeout_sec_,
    [&]() { return handle->is_canceling(); });
  restoreSpeedLimit();
  if (auto path = planner_.lastPath()) {
    result->path = *path;
  }
  const auto nav_dur = std::chrono::steady_clock::now() - nav_start;
  result->navigation_time.sec = static_cast<int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(nav_dur).count());
  if (wasPreempted(task_id)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else if (ok) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } else if (handle->is_canceling()) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "canceled");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::TIMEOUT, "navigate_pose");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
  }
}

void CommandInterface::executeNavigateThrough(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThrough>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<NavigateThrough::Result>();
  if (goal->waypoints.empty()) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::WP_NO_VALID_WAYPOINTS, "empty waypoints");
    handle->abort(result);
    return;
  }
  const auto & task_id = goal->task_id;
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::WAYPOINTS)) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "busy");
    handle->abort(result);
    return;
  }
  controller_.setEnabled(true);
  const double tol = controller_.goalTolerance();
  uint32_t completed = 0;
  try {
    for (uint32_t loop = 0; loop < goal->number_of_loops; ++loop) {
      for (uint32_t i = goal->start_index; i < goal->waypoints.size(); ++i) {
        if (handle->is_canceling()) {
          throw std::runtime_error("cancel");
        }
        if (wasPreempted(task_id)) {
          throw std::runtime_error("preempt");
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
              rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
          }
        } else {
          ws.waypoint_status = autonomy_msgs::msg::WaypointStatus::FAILED;
          ws.error_code = autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT;
          ws.error_msg = "timeout or preempted";
          result->waypoint_statuses.push_back(ws);
          if (goal->stop_on_failure) {
            throw std::runtime_error("wp_fail");
          }
        }
        task_manager_.setProgress(
          static_cast<float>(i + 1) / static_cast<float>(goal->waypoints.size()));
      }
    }
    result->completed_count = completed;
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    if (what == "preempt") {
      result->error = task_manager_.makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == "cancel") {
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "canceled");
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == "wp_fail") {
      result->error = task_manager_.makeError(
        autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT, "waypoint failed");
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
      autonomy_msgs::msg::Error::FOLLOW_TARGET_INVALID, "no target");
    handle->abort(result);
    return;
  }
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::FOLLOW)) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "busy");
    handle->abort(result);
    return;
  }
  applySpeedLimit(goal->target.max_linear_speed);
  controller_.setEnabled(true);
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
          planner_.setGoal(goal->target.target_pose);
          const double d = distanceToPose(goal->target.target_pose.pose);
          fb->distance_to_target = static_cast<float>(d);
          if (d <= follow_dist) {
            throw std::runtime_error("reached");
          }
        } else {
          fb->distance_to_target = 0.0f;
        }
        handle->publish_feedback(fb);
        if (goal->time_allowance.sec > 0 || goal->time_allowance.nanosec > 0) {
          const auto elapsed = std::chrono::steady_clock::now() - start;
          const auto limit = std::chrono::seconds(goal->time_allowance.sec) +
            std::chrono::nanoseconds(goal->time_allowance.nanosec);
          if (elapsed > limit) {
            throw std::runtime_error("timeout");
          }
        }
      });
  } catch (const std::runtime_error & e) {
    restoreSpeedLimit();
    const std::string what = e.what();
    if (what == "timeout" || what == "reached") {
      const auto elapsed = std::chrono::steady_clock::now() - start;
      result->follow_duration.sec = static_cast<int32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
      task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
      handle->succeed(result);
      return;
    }
  }
  restoreSpeedLimit();
  if (wasPreempted(task_id)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  if (handle->is_canceling()) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "canceled");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  if (goal->target.use_target_id) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::FOLLOW_TARGET_LOST, "target_id tracking not implemented");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
    return;
  }
  result->follow_duration.sec = 0;
  result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
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
      autonomy_msgs::msg::Error::TOUR_INVALID, "no exhibits");
    handle->abort(result);
    return;
  }
  const auto & task_id = goal->task_id;
  if (!task_manager_.beginTask(task_id, autonomy_msgs::msg::TaskType::GUIDED_TOUR, goal->tour_id)) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "busy");
    handle->abort(result);
    return;
  }
  task_manager_.publishEvent(autonomy_msgs::msg::Event::TOUR_STARTED, goal->tour_name);
  applySpeedLimit(goal->cruise_speed);
  controller_.setEnabled(true);
  const double tol = controller_.goalTolerance();
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
          task_manager_.publishEvent(autonomy_msgs::msg::Event::LOW_BATTERY, "auto dock");
          autonomy_msgs::action::Dock::Goal dock_goal;
          dock_goal.task_id = task_id + "_dock";
          dock_goal.use_dock_id = true;
          dock_goal.dock_id = default_dock_id_;
          autonomy_msgs::action::Dock::Result dock_result;
          runDock(
            task_id, dock_goal, dock_result,
            [&]() { return handle->is_canceling(); });
          throw std::runtime_error("low_battery_dock");
        }
        if (handle->is_canceling()) {
          throw std::runtime_error("cancel");
        }
        if (wasPreempted(task_id)) {
          throw std::runtime_error("preempt");
        }
        const auto & ex = goal->exhibits[i];
        geometry_msgs::msg::PoseStamped ps = ex.pose;
        task_manager_.setNarration(ex.exhibit_id, ex.narration_id);
        const bool arrived = navigateToGoal(
          task_id, ps, std::max(tol, static_cast<double>(ex.xy_tolerance)),
          waypoint_timeout_sec_, [&]() { return handle->is_canceling(); });

        if (wasPreempted(task_id)) {
          throw std::runtime_error("preempt");
        }
        if (handle->is_canceling()) {
          throw std::runtime_error("cancel");
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
            rclcpp::sleep_for(std::chrono::milliseconds(200));
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
            rclcpp::sleep_for(std::chrono::milliseconds(200));
          }
        }
        if (wasPreempted(task_id)) {
          throw std::runtime_error("preempt");
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
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
    const auto dur = std::chrono::steady_clock::now() - tour_start;
    result->tour_duration.sec = static_cast<int32_t>(
      std::chrono::duration_cast<std::chrono::seconds>(dur).count());
    task_manager_.publishEvent(autonomy_msgs::msg::Event::TOUR_COMPLETED);
    task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);

    if (goal->return_to_dock_on_complete && rclcpp::ok()) {
      autonomy_msgs::action::Dock::Goal dock_goal;
      dock_goal.task_id = task_id + "_dock";
      dock_goal.use_dock_id = true;
      dock_goal.dock_id = default_dock_id_;
      autonomy_msgs::action::Dock::Result dock_result;
      if (task_manager_.beginTask(dock_goal.task_id, autonomy_msgs::msg::TaskType::DOCK)) {
        controller_.setEnabled(true);
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
    if (what == "preempt") {
      result->error = task_manager_.makeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == "cancel") {
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "canceled");
      task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == "low_battery_dock") {
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "low battery dock");
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
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "busy");
    handle->abort(result);
    return;
  }
  task_manager_.publishEvent(autonomy_msgs::msg::Event::DOCK_STARTED);
  controller_.setEnabled(true);
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
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "canceled");
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
      task_id, autonomy_msgs::msg::TaskType::TELEOP, "", goal->preempt_other_tasks))
  {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "busy");
    handle->abort(result);
    return;
  }
  controller_.setEnabled(false);
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
            throw std::runtime_error("timeout");
          }
        }
      });
  } catch (const std::runtime_error & e) {
    if (std::string(e.what()) == "timeout") {
      const auto elapsed = std::chrono::steady_clock::now() - start;
      result->total_elapsed_time.sec = static_cast<int32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
      result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
      task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
      handle->succeed(result);
      return;
    }
  }
  if (wasPreempted(task_id)) {
    result->error = task_manager_.makeError(
      autonomy_msgs::msg::Error::TELEOP_PREEMPTED, "preempted");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  if (handle->is_canceling()) {
    result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "canceled");
    task_manager_.endTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
    return;
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  result->total_elapsed_time.sec = static_cast<int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
  result->error = task_manager_.makeError(autonomy_msgs::msg::Error::NONE, "ok");
  task_manager_.endTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
  handle->succeed(result);
}

}  // namespace autonomy_ros::command
