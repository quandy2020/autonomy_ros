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

#include "autonomy_ros/manager.hpp"

#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"

#include "autonomy/system/autonomy.hpp"
#include "autonomy_msgs/msg/task_state.hpp"
#include "autonomy_msgs/msg/task_type.hpp"

namespace autonomy_ros
{

namespace
{

using autonomy_msgs::msg::TaskType;

}  // namespace

uint8_t TaskManager::PriorityFor(const uint8_t task_type)
{
  switch (task_type) {
    case TaskType::WAYPOINTS:
      return 55;
    case TaskType::NAVIGATION:
      return 50;
    default:
      return 0;
  }
}

bool TaskManager::CanAcquire(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt) const
{
  std::lock_guard<std::mutex> lock(muxer_mutex_);
  const uint8_t priority = PriorityFor(task_type);
  if (!active_task_) {
    return true;
  }
  if (active_task_->task_id == task_id) {
    return true;
  }
  return force_preempt || priority > active_task_->priority;
}

TaskManager::AcquireResult TaskManager::Acquire(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt)
{
  std::lock_guard<std::mutex> lock(muxer_mutex_);
  last_preempted_id_.reset();
  const uint8_t priority = PriorityFor(task_type);

  if (!active_task_) {
    active_task_ = ActiveTask{task_id, task_type, priority};
    return AcquireResult::kAcquired;
  }

  if (active_task_->task_id == task_id) {
    active_task_->task_type = task_type;
    active_task_->priority = priority;
    return AcquireResult::kAcquired;
  }

  if (force_preempt || priority > active_task_->priority) {
    last_preempted_id_ = active_task_->task_id;
    active_task_ = ActiveTask{task_id, task_type, priority};
    return AcquireResult::kPreemptedPrevious;
  }

  return AcquireResult::kRejectedBusy;
}

void TaskManager::Release(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(muxer_mutex_);
  if (active_task_ && active_task_->task_id == task_id) {
    active_task_.reset();
  }
}

void TaskManager::CancelAllTasks()
{
  std::lock_guard<std::mutex> lock(muxer_mutex_);
  active_task_.reset();
}

std::optional<std::string> TaskManager::ConsumeLastPreempted()
{
  std::lock_guard<std::mutex> lock(muxer_mutex_);
  auto id = last_preempted_id_;
  last_preempted_id_.reset();
  return id;
}

TaskManager::TaskManager(
  rclcpp::Node & node,
  ::autonomy::system::Autonomy * core,
  const CoreOptions & core_options,
  StopMotionFn stop_motion)
: node_(node)
, core_(core)
, core_options_(core_options)
, stop_motion_(std::move(stop_motion))
{
  status_.task_state.value = autonomy_msgs::msg::TaskState::UNKNOWN;
  status_.task_type.value = TaskType::IDLE;
  status_.error = MakeError(autonomy_msgs::msg::Error::NONE, "");

  status_pub_ = node_.create_publisher<autonomy_msgs::msg::TaskStatus>(
    kTaskStatusTopic, 10);
  event_pub_ = node_.create_publisher<autonomy_msgs::msg::Event>(
    kEventsTopic, 10);
  status_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&TaskManager::PublishStatus, this));
  RCLCPP_INFO(
    node_.get_logger(), "[task] status -> %s, events -> %s",
    kTaskStatusTopic, kEventsTopic);
}

bool TaskManager::CanBeginTask(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt) const
{
  if (estop_.load()) {
    return false;
  }
  return CanAcquire(task_id, task_type, force_preempt);
}

bool TaskManager::BeginTask(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt)
{
  if (estop_.load()) {
    return false;
  }

  const auto acquire = Acquire(task_id, task_type, force_preempt);
  if (acquire == AcquireResult::kRejectedBusy) {
    return false;
  }

  if (acquire == AcquireResult::kPreemptedPrevious) {
    if (core_) {
      // Ensure previous navigation loop exits before new task starts.
      core_->RequestCancelNavigation();
    }
    SetControllerEnabled(false);
    if (auto preempted = ConsumeLastPreempted()) {
      RCLCPP_WARN(
        node_.get_logger(), "[task] preempted task '%s' for '%s'",
        preempted->c_str(), task_id.c_str());
      PublishEvent(
        autonomy_msgs::msg::Event::TASK_PREEMPTED,
        "preempted_by=" + task_id + ",preempted=" + *preempted);
    }
  }

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.task_id = task_id;
    status_.task_type.value = task_type;
    status_.task_state.value = autonomy_msgs::msg::TaskState::RUNNING;
    status_.paused = false;
    status_.progress = 0.0f;
    status_.error = MakeError(autonomy_msgs::msg::Error::NONE, "");
    status_.description = "running";
  }
  paused_.store(false);
  PublishStatus();
  return true;
}

void TaskManager::EndTask(const uint8_t state, const std::string & task_id)
{
  EndTask(state, MakeError(autonomy_msgs::msg::Error::NONE, ""), task_id);
}

void TaskManager::EndTask(
  const uint8_t state, const autonomy_msgs::msg::Error & error, const std::string & task_id)
{
  Release(task_id);

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_.task_id != task_id) {
      return;
    }
    status_.task_state.value = state;
    status_.error = error;
    if (state == autonomy_msgs::msg::TaskState::SUCCEEDED ||
      state == autonomy_msgs::msg::TaskState::FAILED ||
      state == autonomy_msgs::msg::TaskState::CANCELED)
    {
      status_.task_type.value = TaskType::IDLE;
      status_.progress =
        (state == autonomy_msgs::msg::TaskState::SUCCEEDED) ? 1.0f : status_.progress;
    }
  }
  PublishStatus();
}

bool TaskManager::OwnsTask(const std::string & task_id) const
{
  std::lock_guard<std::mutex> lock(muxer_mutex_);
  return active_task_ && active_task_->task_id == task_id;
}

bool TaskManager::HasActiveTask() const
{
  std::lock_guard<std::mutex> lock(muxer_mutex_);
  return active_task_.has_value();
}

autonomy_msgs::msg::Error TaskManager::MakeError(const uint16_t code, const std::string & msg) const
{
  autonomy_msgs::msg::Error error;
  error.error_code = code;
  error.error_msg = msg;
  return error;
}

autonomy_msgs::msg::TaskStatus TaskManager::GetStatus() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  auto snapshot = status_;
  snapshot.header.stamp = node_.now();
  if (latest_odom_) {
    snapshot.current_pose.header = latest_odom_->header;
    snapshot.current_pose.pose = latest_odom_->pose.pose;
  }
  return snapshot;
}

bool TaskManager::GetStatusFor(
  const std::string & task_id, autonomy_msgs::msg::TaskStatus & out_status) const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (task_id.empty() || task_id == status_.task_id) {
    out_status = status_;
    out_status.header.stamp = node_.now();
    if (latest_odom_) {
      out_status.current_pose.header = latest_odom_->header;
      out_status.current_pose.pose = latest_odom_->pose.pose;
    }
    return true;
  }
  return false;
}

bool TaskManager::IsPaused() const { return paused_.load(); }

bool TaskManager::IsEstop() const { return estop_.load(); }

bool TaskManager::PauseTask(const std::string & reason)
{
  if (!HasActiveTask()) {
    return false;
  }
  paused_.store(true);
  SetControllerEnabled(false);
  const std::string desc = reason.empty() ? "paused" : reason;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.paused = true;
    status_.description = desc;
  }
  PublishEvent(autonomy_msgs::msg::Event::TASK_PAUSED, desc);
  return true;
}

bool TaskManager::ResumeTask()
{
  if (!HasActiveTask()) {
    return false;
  }
  if (estop_.load()) {
    return false;
  }
  paused_.store(false);
  SetControllerEnabled(true);
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.paused = false;
    status_.description = "running";
  }
  PublishEvent(autonomy_msgs::msg::Event::TASK_RESUMED);
  return true;
}

bool TaskManager::CancelTask(
  const std::string & task_id, const bool cancel_all, const uint8_t filter_task_type)
{
  const bool has_active = HasActiveTask();
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!cancel_all && !task_id.empty() && task_id != status_.task_id) {
      return false;
    }
    if (filter_task_type != 255 && filter_task_type != 0 &&
      status_.task_type.value != filter_task_type)
    {
      return false;
    }
    if (!has_active && !cancel_all && task_id.empty()) {
      return false;
    }
  }
  if (core_) {
    core_->RequestCancelNavigation();
  }
  CancelAllTasks();
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.task_state.value = autonomy_msgs::msg::TaskState::CANCELED;
    status_.task_type.value = TaskType::IDLE;
    status_.description = "canceled";
  }
  paused_.store(false);
  PublishStatus();
  return true;
}

void TaskManager::TriggerEstop(const std::string & reason)
{
  estop_.store(true);
  paused_.store(true);
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.paused = true;
    status_.description = reason;
    status_.error = MakeError(autonomy_msgs::msg::Error::SAFETY_ESTOP, reason);
  }
  PublishEvent(autonomy_msgs::msg::Event::EMERGENCY_STOP, reason);
}

void TaskManager::ReleaseEstop() { estop_.store(false); }

void TaskManager::UpdateOdom(const nav_msgs::msg::Odometry & odom)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  latest_odom_ = std::make_shared<nav_msgs::msg::Odometry>(odom);
}

bool TaskManager::NavigateToPose(
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker,
  const double timeout_sec)
{
  if (!core_) {
    return false;
  }
  return core_->NavigateToPose(
    fromRos(goal), std::move(cancel_checker),
    []() { return rclcpp::ok(); }, timeout_sec);
}

bool TaskManager::NavigateThroughPoses(
  const std::vector<geometry_msgs::msg::PoseStamped> & goals,
  std::function<bool()> cancel_checker,
  const double timeout_sec)
{
  if (!core_ || goals.empty()) {
    return false;
  }
  std::vector<::autonomy::commsgs::geometry_msgs::PoseStamped> core_goals;
  core_goals.reserve(goals.size());
  for (const auto & goal : goals) {
    core_goals.push_back(fromRos(goal));
  }
  return core_->NavigateThroughPoses(
    core_goals, std::move(cancel_checker), []() { return rclcpp::ok(); },
    timeout_sec);
}

std::optional<nav_msgs::msg::Path> TaskManager::LastPath() const
{
  if (!core_) {
    return std::nullopt;
  }
  if (auto path = core_->GetLastPath()) {
    return toRos(*path);
  }
  return std::nullopt;
}

void TaskManager::SetControllerEnabled(const bool enabled)
{
  controller_enabled_.store(enabled);
  if (core_) {
    core_->SetControllerEnabled(enabled);
  }
  if (!enabled && stop_motion_) {
    stop_motion_();
  }
}

bool TaskManager::HasOdometry() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return latest_odom_ != nullptr;
}

void TaskManager::SetProgress(const float progress)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.progress = progress;
}

void TaskManager::PublishEvent(const uint8_t event_type, const std::string & message)
{
  autonomy_msgs::msg::Event event;
  event.header.stamp = node_.now();
  event.event_type = event_type;
  event.message = message;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    event.task_id = status_.task_id;
    event.status = status_;
    event.status.header.stamp = node_.now();
  }
  event_pub_->publish(event);
}

void TaskManager::PublishStatus()
{
  status_pub_->publish(GetStatus());
}

}  // namespace autonomy_ros
