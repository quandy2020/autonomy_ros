/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy_ros/server.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autonomy_msgs/msg/error.hpp"
#include "autonomy_msgs/msg/task_state.hpp"
#include "autonomy_msgs/msg/task_type.hpp"
#include "autonomy_msgs/msg/waypoint_status.hpp"
#include "autonomy_ros/visualizer.hpp"

namespace autonomy_ros
{


NavigationService::NavigationService(
  rclcpp::Node & node,
  ::autonomy::system::Autonomy & core,
  const CoreOptions & core_options,
  const NavigationOptions & navigation_options,
  TaskManager::StopMotionFn stop_motion,
  Visualizer * visualizer)
: node_(node)
, visualizer_(visualizer)
, init_pose_topic_(navigation_options.init_pose_topic)
, goal_pose_topic_(navigation_options.goal_pose_topic)
, waypoint_timeout_sec_(navigation_options.waypoint_timeout_sec)
{
  task_manager_ = std::make_unique<TaskManager>(
    node_, &core, core_options, std::move(stop_motion));


  initial_pose_pub_ =
    node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      kInitialPoseTopic, 1);

  init_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    init_pose_topic_, 10,
    std::bind(&NavigationService::OnInitPose, this, std::placeholders::_1));
  goal_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_, 10,
    std::bind(&NavigationService::OnGoalPose, this, std::placeholders::_1));

  RegisterActionServer(navigate_pose_server_, kNavigatePoseAction,
    [this](const std::shared_ptr<const NavigatePose::Goal> & goal) {
      return HandleGoal(goal->task_id, autonomy_msgs::msg::TaskType::NAVIGATION);
    },
    &NavigationService::ExecuteNavigatePose);

  RegisterActionServer(navigate_through_server_, kNavigateThroughAction,
    [this](const std::shared_ptr<const NavigateThrough::Goal> & goal) {
      return HandleGoal(goal->task_id, autonomy_msgs::msg::TaskType::WAYPOINTS);
    },
    &NavigationService::ExecuteNavigateThrough);

  RegisterService(cancel_task_srv_, kCancelTaskService,
    [this](const std::shared_ptr<autonomy_msgs::srv::CancelTask::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::CancelTask::Response> res) {
      task_manager_->SetControllerEnabled(false);
      res->success = task_manager_->CancelTask(
        req->task_id, req->cancel_all, req->task_type.value);
      res->status = task_manager_->GetStatus();
      res->error = res->success ?
        task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_->MakeError(
          autonomy_msgs::msg::Error::INVALID_REQUEST, "cancel failed");
    });

  RegisterService(get_status_srv_, kGetTaskStatusService,
    [this](const std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::GetTaskStatus::Response> res) {
      res->success = task_manager_->GetStatusFor(req->task_id, res->status);
      res->error = res->success ?
        task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_->MakeError(
          autonomy_msgs::msg::Error::INVALID_REQUEST, "unknown task_id");
    });

  RegisterService(pause_task_srv_, kPauseTaskService,
    [this](const std::shared_ptr<autonomy_msgs::srv::PauseTask::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::PauseTask::Response> res) {
      task_manager_->SetControllerEnabled(false);
      res->success = task_manager_->PauseTask(req->reason);
      res->status = task_manager_->GetStatus();
      res->error = res->success ?
        task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_->MakeError(
          autonomy_msgs::msg::Error::NOT_AVAILABLE, "no active task");
    });

  RegisterService(resume_task_srv_, kResumeTaskService,
    [this](const std::shared_ptr<autonomy_msgs::srv::ResumeTask::Request> &,
      std::shared_ptr<autonomy_msgs::srv::ResumeTask::Response> res) {
      res->success = task_manager_->ResumeTask();
      if (res->success && !task_manager_->IsEstop()) {
        task_manager_->SetControllerEnabled(true);
      }
      res->status = task_manager_->GetStatus();
      res->error = res->success ?
        task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "") :
        task_manager_->MakeError(
          autonomy_msgs::msg::Error::NOT_AVAILABLE, "cannot resume");
    });

  RegisterService(estop_srv_, kTriggerEstopService,
    [this](const std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::TriggerEmergencyStop::Response> res) {
      if (req->engage) {
        task_manager_->SetControllerEnabled(false);
        task_manager_->TriggerEstop(req->reason);
      } else {
        task_manager_->ReleaseEstop();
        if (!task_manager_->IsPaused() && task_manager_->HasActiveTask()) {
          task_manager_->SetControllerEnabled(true);
        }
      }
      res->success = true;
      res->status = task_manager_->GetStatus();
      res->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "");
    });

  RegisterService(set_initial_pose_srv_, kSetInitialPoseService,
    [this](const std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Request> & req,
      std::shared_ptr<autonomy_msgs::srv::SetInitialPose::Response> res) {
      initial_pose_pub_->publish(req->pose);
      res->success = true;
      res->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "");
    });

  RCLCPP_INFO(
    node_.get_logger(),
    "[command] navigate_pose + navigate_through; topics init_pose=%s goal_pose=%s",
    init_pose_topic_.c_str(), goal_pose_topic_.c_str());
}

void NavigationService::UpdateOdom(const nav_msgs::msg::Odometry & odom)
{
  if (task_manager_) {
    task_manager_->UpdateOdom(odom);
  }
}

bool NavigationService::HasOdometry() const
{
  return task_manager_ && task_manager_->HasOdometry();
}

bool NavigationService::HasActiveNavigationTask() const
{
  return task_manager_ && task_manager_->HasActiveTask();
}

bool NavigationService::IsControllerEnabled() const
{
  return task_manager_ && task_manager_->IsControllerEnabled();
}


void NavigationService::OnInitPose(
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

void NavigationService::OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (task_manager_->IsEstop()) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal_pose: estop active");
    return;
  }
  if (!task_manager_->CanBeginTask(
      goal_pose_topic_, autonomy_msgs::msg::TaskType::NAVIGATION, true))
  {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal_pose: another task is running");
    return;
  }
  geometry_msgs::msg::PoseStamped goal = *msg;
  if (visualizer_) {
    visualizer_->OnNavigationGoal(goal);
  }
  std::thread{
    &NavigationService::RunTopicGoalPose, this, std::move(goal),
    goal_pose_topic_}
    .detach();
  RCLCPP_INFO(
    node_.get_logger(), "[command] goal_pose accepted (%.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y);
}

void NavigationService::RunTopicGoalPose(
  geometry_msgs::msg::PoseStamped goal, const std::string & task_id)
{
  if (!task_manager_->BeginTask(
      task_id, autonomy_msgs::msg::TaskType::NAVIGATION, true))
  {
    RCLCPP_WARN(node_.get_logger(), "[command] goal_pose task start failed");
    return;
  }
  task_manager_->SetControllerEnabled(true);
  const bool ok = NavigateToGoal(
    task_id, goal, waypoint_timeout_sec_,
    [&]() { return !task_manager_->OwnsTask(task_id); });
  if (WasPreempted(task_id)) {
    task_manager_->EndTask(
      autonomy_msgs::msg::TaskState::CANCELED,
      task_manager_->MakeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted"),
      task_id);
  } else if (ok) {
    task_manager_->EndTask(
      autonomy_msgs::msg::TaskState::SUCCEEDED,
      task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "ok"),
      task_id);
    RCLCPP_INFO(node_.get_logger(), "[command] goal_pose navigation succeeded");
  } else {
    task_manager_->EndTask(
      autonomy_msgs::msg::TaskState::FAILED,
      task_manager_->MakeError(
        autonomy_msgs::msg::Error::TIMEOUT, goal_pose_topic_.c_str()),
      task_id);
    RCLCPP_WARN(node_.get_logger(), "[command] goal_pose navigation failed or timed out");
  }
  if (!task_manager_->HasActiveTask() || task_manager_->IsPaused() || task_manager_->IsEstop()) {
    task_manager_->SetControllerEnabled(false);
  }
}

rclcpp_action::GoalResponse NavigationService::HandleGoal(
  const std::string & task_id, const uint8_t task_type) const
{
  if (task_manager_->IsEstop()) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal %s: estop", task_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!task_manager_->CanBeginTask(task_id, task_type, false)) {
    RCLCPP_WARN(node_.get_logger(), "[command] reject goal %s: busy", task_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationService::HandleCancel()
{
  task_manager_->SetControllerEnabled(false);
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool NavigationService::WasPreempted(const std::string & task_id) const
{
  return !task_manager_->OwnsTask(task_id);
}

bool NavigationService::NavigateToGoal(
  const std::string & task_id,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const double timeout_sec,
  const std::function<bool()> & extra_cancel)
{
  auto cancel = [&]() {
    if (WasPreempted(task_id)) {
      return true;
    }
    return extra_cancel && extra_cancel();
  };
  task_manager_->SetControllerEnabled(true);
  return task_manager_->NavigateToPose(goal_pose, cancel, timeout_sec);
}

void NavigationService::ExecuteNavigatePose(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigatePose>> handle)
{
  const auto goal = handle->get_goal();
  const auto & task_id = goal->task_id;
  auto result = std::make_shared<NavigatePose::Result>();
  if (!goal->behavior_tree.empty()) {
    result->error = task_manager_->MakeError(
      autonomy_msgs::msg::Error::NAV_GOAL_INVALID,
      "per-action behavior_tree is not supported; set default BT in tasks lua");
    handle->abort(result);
    return;
  }
  if (!task_manager_->BeginTask(task_id, autonomy_msgs::msg::TaskType::NAVIGATION)) {
    result->error = task_manager_->MakeError(
      autonomy_msgs::msg::Error::NOT_AVAILABLE, "estop or busy");
    handle->abort(result);
    return;
  }
  const auto nav_start = std::chrono::steady_clock::now();
  const bool ok = NavigateToGoal(
    task_id, goal->goal, waypoint_timeout_sec_,
    [&]() { return handle->is_canceling(); });
  if (auto path = task_manager_->LastPath()) {
    result->path = *path;
  }
  const auto nav_dur = std::chrono::steady_clock::now() - nav_start;
  result->navigation_time.sec = static_cast<int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(nav_dur).count());

  if (WasPreempted(task_id)) {
    result->error = task_manager_->MakeError(
      autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
    task_manager_->EndTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else if (ok) {
    result->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "ok");
    task_manager_->EndTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } else if (handle->is_canceling()) {
    result->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "canceled");
    task_manager_->EndTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
    handle->canceled(result);
  } else {
    result->error = task_manager_->MakeError(
      autonomy_msgs::msg::Error::TIMEOUT, kNavigatePoseAction);
    task_manager_->EndTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
  }
}

void NavigationService::ExecuteNavigateThrough(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThrough>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<NavigateThrough::Result>();
  if (!goal->behavior_tree.empty()) {
    result->error = task_manager_->MakeError(
      autonomy_msgs::msg::Error::NAV_GOAL_INVALID,
      "per-action behavior_tree is not supported; set default BT in tasks lua");
    handle->abort(result);
    return;
  }
  if (goal->waypoints.empty()) {
    result->error = task_manager_->MakeError(
      autonomy_msgs::msg::Error::WP_NO_VALID_WAYPOINTS, "empty waypoints");
    handle->abort(result);
    return;
  }
  const auto & task_id = goal->task_id;
  if (!task_manager_->BeginTask(task_id, autonomy_msgs::msg::TaskType::WAYPOINTS)) {
    result->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NOT_AVAILABLE, "busy");
    handle->abort(result);
    return;
  }
  task_manager_->SetControllerEnabled(true);
  uint32_t completed = 0;

  const bool use_behavior_tree_through =
    task_manager_->GetCoreOptions().use_bt_navigation &&
    goal->number_of_loops <= 1;

  if (use_behavior_tree_through) {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(goal->waypoints.size() - goal->start_index);
    for (uint32_t i = goal->start_index; i < goal->waypoints.size(); ++i) {
      poses.push_back(goal->waypoints[i].pose);
    }
    const double timeout = waypoint_timeout_sec_ *
      static_cast<double>(std::max<std::size_t>(poses.size(), 1));
    const bool ok = task_manager_->NavigateThroughPoses(
      poses,
      [&]() { return handle->is_canceling() || WasPreempted(task_id); },
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
        ws.error_msg = "timeout or preempted";
      }
      result->waypoint_statuses.push_back(ws);
    }
    result->completed_count = completed;
    if (WasPreempted(task_id)) {
      result->error = task_manager_->MakeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
      task_manager_->EndTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
      return;
    }
    if (ok) {
      result->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "ok");
      task_manager_->EndTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
      handle->succeed(result);
      return;
    }
    if (handle->is_canceling()) {
      result->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "canceled");
      task_manager_->EndTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
      return;
    }
    result->error = task_manager_->MakeError(
      autonomy_msgs::msg::Error::TIMEOUT, kNavigatePoseAction);
    task_manager_->EndTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
    handle->abort(result);
    return;
  }

  try {
    for (uint32_t loop = 0; loop < goal->number_of_loops; ++loop) {
      for (uint32_t i = goal->start_index; i < goal->waypoints.size(); ++i) {
        if (handle->is_canceling()) {
          throw std::runtime_error("cancel");
        }
        if (WasPreempted(task_id)) {
          throw std::runtime_error("preempt");
        }
        const auto & wp = goal->waypoints[i];
        const bool ok = NavigateToGoal(
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
            while (rclcpp::ok() && task_manager_->OwnsTask(task_id) && !handle->is_canceling()) {
              if (std::chrono::steady_clock::now() - t0 >
                std::chrono::duration<double>(wp.wait_duration))
              {
                break;
              }
              rclcpp::sleep_for(
                std::chrono::milliseconds(100));
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
        task_manager_->SetProgress(
          static_cast<float>(i + 1) / static_cast<float>(goal->waypoints.size()));
      }
    }
    result->completed_count = completed;
    result->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "ok");
    task_manager_->EndTask(autonomy_msgs::msg::TaskState::SUCCEEDED, result->error, task_id);
    handle->succeed(result);
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    if (what == "preempt") {
      result->error = task_manager_->MakeError(
        autonomy_msgs::msg::Error::TASK_CONFLICT, "preempted");
      task_manager_->EndTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == "cancel") {
      result->error = task_manager_->MakeError(autonomy_msgs::msg::Error::NONE, "canceled");
      task_manager_->EndTask(autonomy_msgs::msg::TaskState::CANCELED, result->error, task_id);
      handle->canceled(result);
    } else if (what == "wp_fail") {
      result->error = task_manager_->MakeError(
        autonomy_msgs::msg::Error::WP_MISSED_WAYPOINT, "waypoint failed");
      task_manager_->EndTask(autonomy_msgs::msg::TaskState::FAILED, result->error, task_id);
      handle->abort(result);
    }
  }
}

}  // namespace autonomy_ros
