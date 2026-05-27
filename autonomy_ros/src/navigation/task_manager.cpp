#include "autonomy_ros/navigation/task_manager.hpp"

#include "autonomy_ros/system/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"

#include "autonomy/system/autonomy.hpp"
#include "autonomy_msgs/msg/task_state.hpp"
#include "autonomy_msgs/msg/task_type.hpp"

namespace autonomy_ros::navigation
{

using system::AutonomyCoreOptions;

TaskManager::TaskManager(
  rclcpp::Node & node,
  ::autonomy::system::Autonomy * core,
  const AutonomyCoreOptions & core_options,
  StopMotionFn stop_motion)
: node_(node)
, core_(core)
, core_options_(core_options)
, stop_motion_(std::move(stop_motion))
{
  status_.task_state.value = autonomy_msgs::msg::TaskState::UNKNOWN;
  status_.task_type.value = autonomy_msgs::msg::TaskType::IDLE;
  status_.error = makeError(autonomy_msgs::msg::Error::NONE, "");

  status_pub_ = node_.create_publisher<autonomy_msgs::msg::TaskStatus>(
    constants::topics::kTaskStatus, 10);
  event_pub_ = node_.create_publisher<autonomy_msgs::msg::Event>(
    constants::topics::kEvents, 10);
  status_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&TaskManager::publishStatus, this));
  RCLCPP_INFO(
    node_.get_logger(),
    "[task] status -> %s, events -> %s",
    constants::topics::kTaskStatus, constants::topics::kEvents);
}

bool TaskManager::canBeginTask(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt) const
{
  if (estop_.load()) {
    return false;
  }
  return muxer_.canAcquire(task_id, task_type, force_preempt);
}

bool TaskManager::beginTask(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt)
{
  if (estop_.load()) {
    return false;
  }

  const auto acquire = muxer_.acquire(task_id, task_type, force_preempt);
  if (acquire == TaskMuxer::AcquireResult::RejectedBusy) {
    return false;
  }

  if (acquire == TaskMuxer::AcquireResult::PreemptedPrevious) {
    if (auto preempted = muxer_.consumeLastPreempted()) {
      RCLCPP_WARN(
        node_.get_logger(), "[task] preempted task '%s' for '%s'",
        preempted->c_str(), task_id.c_str());
      publishEvent(
        autonomy_msgs::msg::Event::TASK_PREEMPTED,
        "preempted_by=" + task_id + ",preempted=" + *preempted);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  status_.task_id = task_id;
  status_.task_type.value = task_type;
  status_.task_state.value = autonomy_msgs::msg::TaskState::RUNNING;
  status_.paused = false;
  status_.progress = 0.0f;
  status_.error = makeError(autonomy_msgs::msg::Error::NONE, "");
  status_.description = "running";
  paused_.store(false);
  publishStatus();
  return true;
}

void TaskManager::endTask(const uint8_t state, const std::string & task_id)
{
  endTask(state, makeError(autonomy_msgs::msg::Error::NONE, ""), task_id);
}

void TaskManager::endTask(
  const uint8_t state, const autonomy_msgs::msg::Error & error, const std::string & task_id)
{
  muxer_.release(task_id);

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_.task_id != task_id) {
    return;
  }
  status_.task_state.value = state;
  status_.error = error;
  if (state == autonomy_msgs::msg::TaskState::SUCCEEDED ||
    state == autonomy_msgs::msg::TaskState::FAILED ||
    state == autonomy_msgs::msg::TaskState::CANCELED)
  {
    status_.task_type.value = autonomy_msgs::msg::TaskType::IDLE;
    status_.progress =
      (state == autonomy_msgs::msg::TaskState::SUCCEEDED) ? 1.0f : status_.progress;
  }
  publishStatus();
}

bool TaskManager::ownsTask(const std::string & task_id) const
{
  return muxer_.ownsTask(task_id);
}

bool TaskManager::hasActiveTask() const
{
  return muxer_.hasActiveTask();
}

autonomy_msgs::msg::Error TaskManager::makeError(const uint16_t code, const std::string & msg) const
{
  autonomy_msgs::msg::Error error;
  error.error_code = code;
  error.error_msg = msg;
  return error;
}

autonomy_msgs::msg::TaskStatus TaskManager::getStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto snapshot = status_;
  snapshot.header.stamp = node_.now();
  if (latest_odom_) {
    snapshot.current_pose.header = latest_odom_->header;
    snapshot.current_pose.pose = latest_odom_->pose.pose;
  }
  return snapshot;
}

bool TaskManager::getStatusFor(
  const std::string & task_id, autonomy_msgs::msg::TaskStatus & out_status) const
{
  std::lock_guard<std::mutex> lock(mutex_);
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

bool TaskManager::isPaused() const { return paused_.load(); }
bool TaskManager::isEstop() const { return estop_.load(); }

bool TaskManager::pauseTask(const std::string & reason)
{
  if (!hasActiveTask()) {
    return false;
  }
  paused_.store(true);
  if (core_ && core_->GetTask()) {
    core_->GetTask()->PauseNavigation();
  }
  setControllerEnabled(false);
  const std::string desc = reason.empty() ? "paused" : reason;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.paused = true;
    status_.description = desc;
  }
  publishEvent(autonomy_msgs::msg::Event::TASK_PAUSED, desc);
  return true;
}

bool TaskManager::resumeTask()
{
  if (!hasActiveTask()) {
    return false;
  }
  if (estop_.load()) {
    return false;
  }
  paused_.store(false);
  if (core_ && core_->GetTask()) {
    core_->GetTask()->ResumeNavigation();
  }
  setControllerEnabled(true);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.paused = false;
    status_.description = "running";
  }
  publishEvent(autonomy_msgs::msg::Event::TASK_RESUMED);
  return true;
}

bool TaskManager::cancelTask(
  const std::string & task_id, const bool cancel_all, const uint8_t filter_task_type)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cancel_all && !task_id.empty() && task_id != status_.task_id) {
    return false;
  }
  if (filter_task_type != 255 && filter_task_type != 0 &&
    status_.task_type.value != filter_task_type)
  {
    return false;
  }
  if (!hasActiveTask() && !cancel_all && task_id.empty()) {
    return false;
  }
  if (core_) {
    core_->RequestCancelNavigation();
  }
  muxer_.cancelAll();
  status_.task_state.value = autonomy_msgs::msg::TaskState::CANCELED;
  status_.task_type.value = autonomy_msgs::msg::TaskType::IDLE;
  status_.description = "canceled";
  paused_.store(false);
  publishStatus();
  return true;
}

void TaskManager::triggerEstop(const std::string & reason)
{
  estop_.store(true);
  paused_.store(true);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.paused = true;
    status_.description = reason;
    status_.error = makeError(autonomy_msgs::msg::Error::SAFETY_ESTOP, reason);
  }
  publishEvent(autonomy_msgs::msg::Event::EMERGENCY_STOP, reason);
}

void TaskManager::releaseEstop() { estop_.store(false); }

void TaskManager::updateOdom(const nav_msgs::msg::Odometry & odom)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odom_ = std::make_shared<nav_msgs::msg::Odometry>(odom);
}

bool TaskManager::navigateToPose(
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker,
  const double timeout_sec)
{
  if (!core_) {
    return false;
  }
  return core_->NavigateToPose(
    conversions::fromRos(goal), std::move(cancel_checker),
    []() { return rclcpp::ok(); }, timeout_sec);
}

bool TaskManager::navigateThroughPoses(
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
    core_goals.push_back(conversions::fromRos(goal));
  }
  return core_->NavigateThroughPoses(
    core_goals, std::move(cancel_checker), []() { return rclcpp::ok(); },
    timeout_sec);
}

std::optional<nav_msgs::msg::Path> TaskManager::lastPath() const
{
  if (!core_) {
    return std::nullopt;
  }
  if (auto path = core_->GetLastPath()) {
    return conversions::toRos(*path);
  }
  return std::nullopt;
}

void TaskManager::setControllerEnabled(const bool enabled)
{
  controller_enabled_.store(enabled);
  if (core_) {
    core_->SetControllerEnabled(enabled);
  }
  if (!enabled && stop_motion_) {
    stop_motion_();
  }
}

bool TaskManager::hasOdometry() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_odom_ != nullptr;
}

void TaskManager::setProgress(const float progress)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.progress = progress;
}

void TaskManager::publishEvent(const uint8_t event_type, const std::string & message)
{
  autonomy_msgs::msg::Event event;
  event.header.stamp = node_.now();
  event.event_type = event_type;
  event.message = message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    event.task_id = status_.task_id;
    event.status = status_;
    event.status.header.stamp = node_.now();
  }
  event_pub_->publish(event);
}

void TaskManager::publishStatus()
{
  status_pub_->publish(getStatus());
}

}  // namespace autonomy_ros::navigation
