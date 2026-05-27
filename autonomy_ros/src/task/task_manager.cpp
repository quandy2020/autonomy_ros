#include "autonomy_ros/task/task_manager.hpp"

#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/conversions/conversions.hpp"

#include <algorithm>
#include <vector>

#include "autonomy/system/autonomy.hpp"
#include "autonomy_msgs/msg/task_state.hpp"
#include "autonomy_msgs/msg/task_type.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros::task
{

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
  status_.battery.percentage = battery_percent_;
  status_.battery.low_battery = false;

  node_.declare_parameter<float>("task.battery_percent", battery_percent_);
  battery_percent_ = static_cast<float>(node_.get_parameter("task.battery_percent").as_double());

  status_pub_ = node_.create_publisher<autonomy_msgs::msg::TaskStatus>(
    constants::topics::kTaskStatus, 10);
  battery_pub_ = node_.create_publisher<autonomy_msgs::msg::BatteryStatus>(
    constants::topics::kBatteryStatus, 10);
  event_pub_ = node_.create_publisher<autonomy_msgs::msg::Event>(
    constants::topics::kEvents, 10);
  status_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&TaskManager::publishStatus, this));
  RCLCPP_INFO(
    node_.get_logger(),
    "[task] status -> autonomy/status, events -> autonomy/events, battery -> autonomy/battery");
}

bool TaskManager::canBeginTask(
  const std::string & task_id, uint8_t task_type, bool force_preempt) const
{
  if (estop_.load()) {
    return false;
  }
  return muxer_.canAcquire(task_id, task_type, force_preempt);
}

bool TaskManager::beginTask(
  const std::string & task_id, uint8_t task_type, const std::string & tour_id,
  bool force_preempt)
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
        node_.get_logger(), "[task] muxer preempted task '%s' for '%s'",
        preempted->c_str(), task_id.c_str());
      publishEvent(
        autonomy_msgs::msg::Event::TASK_PREEMPTED,
        "preempted_by=" + task_id + ",preempted=" + *preempted);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  status_.task_id = task_id;
  status_.tour_id = tour_id;
  status_.task_type.value = task_type;
  status_.task_state.value = autonomy_msgs::msg::TaskState::RUNNING;
  status_.paused = false;
  status_.progress = 0.0f;
  status_.error = makeError(autonomy_msgs::msg::Error::NONE, "");
  status_.description = "running";
  status_.battery.percentage = battery_percent_;
  status_.battery.low_battery = isLowBattery(20.0f);
  paused_.store(false);
  continue_requested_.store(false);
  skip_exhibit_index_.reset();
  skip_exhibit_id_.reset();
  publishStatus();
  return true;
}

void TaskManager::endTask(uint8_t state, const std::string & task_id)
{
  endTask(state, makeError(autonomy_msgs::msg::Error::NONE, ""), task_id);
}

void TaskManager::endTask(
  uint8_t state, const autonomy_msgs::msg::Error & error, const std::string & task_id)
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
    status_.progress = (state == autonomy_msgs::msg::TaskState::SUCCEEDED) ? 1.0f : status_.progress;
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

autonomy_msgs::msg::Error TaskManager::makeError(uint16_t code, const std::string & msg) const
{
  autonomy_msgs::msg::Error e;
  e.error_code = code;
  e.error_msg = msg;
  return e;
}

autonomy_msgs::msg::TaskStatus TaskManager::getStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto s = status_;
  s.header.stamp = node_.now();
  if (latest_odom_) {
    s.current_pose.header = latest_odom_->header;
    s.current_pose.pose = latest_odom_->pose.pose;
  }
  s.battery.percentage = battery_percent_;
  s.battery.low_battery = isLowBattery(20.0f);
  return s;
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
    out_status.battery.percentage = battery_percent_;
    out_status.battery.low_battery = isLowBattery(20.0f);
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
  std::string desc;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.paused = true;
    desc = reason.empty() ? "paused" : reason;
    status_.description = desc;
  }
  publishEvent(autonomy_msgs::msg::Event::TOUR_PAUSED, desc);
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
  publishEvent(autonomy_msgs::msg::Event::TOUR_RESUMED);
  return true;
}

bool TaskManager::cancelTask(
  const std::string & task_id, bool cancel_all, uint8_t filter_task_type)
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

void TaskManager::requestContinueTour() { continue_requested_.store(true); }

bool TaskManager::consumeContinueTour()
{
  return continue_requested_.exchange(false);
}

void TaskManager::requestSkipExhibit(uint32_t index, const std::string & exhibit_id)
{
  skip_exhibit_index_ = index;
  if (!exhibit_id.empty()) {
    skip_exhibit_id_ = exhibit_id;
  }
}

std::optional<uint32_t> TaskManager::consumeSkipExhibitForTour(
  const std::vector<std::string> & exhibit_ids)
{
  if (!skip_exhibit_index_ && !skip_exhibit_id_) {
    return std::nullopt;
  }
  uint32_t resolved = skip_exhibit_index_.value_or(0);
  if (skip_exhibit_id_) {
    bool found = false;
    for (size_t i = 0; i < exhibit_ids.size(); ++i) {
      if (exhibit_ids[i] == *skip_exhibit_id_) {
        resolved = static_cast<uint32_t>(i);
        found = true;
        break;
      }
    }
    if (!found) {
      skip_exhibit_index_.reset();
      skip_exhibit_id_.reset();
      return std::nullopt;
    }
  }
  skip_exhibit_index_.reset();
  skip_exhibit_id_.reset();
  return resolved;
}

void TaskManager::setBatteryPercent(float percent)
{
  battery_percent_ = std::clamp(percent, 0.0f, 100.0f);
  std::lock_guard<std::mutex> lock(mutex_);
  status_.battery.percentage = battery_percent_;
  status_.battery.low_battery = isLowBattery(20.0f);
}

bool TaskManager::isLowBattery(float threshold_percent) const
{
  return battery_percent_ < threshold_percent;
}

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

void TaskManager::replanToGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (core_) {
    core_->ReplanToGoal(conversions::fromRos(goal));
  }
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

bool TaskManager::teleopDrive(
  const double time_allowance_sec,
  const double max_linear_vel,
  const double max_angular_vel,
  std::function<bool()> cancel_checker)
{
  if (!core_) {
    return false;
  }
  beginTeleop(max_linear_vel, max_angular_vel);
  const double allowance =
    time_allowance_sec > 0.0 ? time_allowance_sec : 3600.0;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(allowance);

  while (rclcpp::ok()) {
    if (cancel_checker && cancel_checker()) {
      endTeleop();
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      endTeleop();
      return true;
    }
    rclcpp::sleep_for(
      std::chrono::milliseconds(constants::defaults::kBtWaitPollMs));
  }
  endTeleop();
  return false;
}

void TaskManager::beginTeleop(const double max_linear_vel, const double max_angular_vel)
{
  teleop_active_.store(true);
  max_teleop_linear_ =
    max_linear_vel > 0.0 ? max_linear_vel : core_options_.max_linear_vel;
  max_teleop_angular_ = max_angular_vel > 0.0 ? max_angular_vel : 1.5;
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    teleop_command_ = ::autonomy::commsgs::geometry_msgs::TwistStamped{};
  }
  setControllerEnabled(false);
}

void TaskManager::endTeleop()
{
  teleop_active_.store(false);
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    teleop_command_ = ::autonomy::commsgs::geometry_msgs::TwistStamped{};
  }
  if (stop_motion_) {
    stop_motion_();
  }
}

bool TaskManager::isTeleopActive() const
{
  return teleop_active_.load();
}

void TaskManager::updateTeleopCommand(const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!teleop_active_.load()) {
    return;
  }
  auto twist = conversions::fromRos(cmd);
  if (max_teleop_linear_ > 0.0) {
    twist.twist.linear.x = std::clamp(
      twist.twist.linear.x, -max_teleop_linear_, max_teleop_linear_);
  }
  if (max_teleop_angular_ > 0.0) {
    twist.twist.angular.z = std::clamp(
      twist.twist.angular.z, -max_teleop_angular_, max_teleop_angular_);
  }
  std::lock_guard<std::mutex> lock(teleop_mutex_);
  teleop_command_ = twist;
}

std::optional<::autonomy::commsgs::geometry_msgs::TwistStamped>
TaskManager::teleopCommand() const
{
  if (!teleop_active_.load()) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(teleop_mutex_);
  return teleop_command_;
}

bool TaskManager::transformPoseToGlobalFrame(
  ::autonomy::commsgs::geometry_msgs::PoseStamped & pose)
{
  return core_ && core_->TransformPoseToGlobalFrame(pose);
}

void TaskManager::setProgress(float progress)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.progress = progress;
}

void TaskManager::setNarration(
  const std::string & exhibit_id, const std::string & narration_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.current_exhibit_id = exhibit_id;
  status_.current_narration_id = narration_id;
}

void TaskManager::setWaitingForContinue(bool waiting)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.description = waiting ? "waiting_for_continue" : "moving";
}

void TaskManager::publishEvent(uint8_t event_type, const std::string & message)
{
  autonomy_msgs::msg::Event ev;
  ev.header.stamp = node_.now();
  ev.event_type = event_type;
  ev.message = message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ev.tour_id = status_.tour_id;
    ev.task_id = status_.task_id;
    ev.exhibit_id = status_.current_exhibit_id;
    ev.narration_id = status_.current_narration_id;
    ev.status = status_;
    ev.status.header.stamp = node_.now();
  }
  event_pub_->publish(ev);
}

void TaskManager::publishStatus()
{
  autonomy_msgs::msg::BatteryStatus bat;
  bat.header.stamp = node_.now();
  bat.percentage = battery_percent_;
  bat.low_battery = isLowBattery(20.0f);
  battery_pub_->publish(bat);
  status_pub_->publish(getStatus());
}

}  // namespace autonomy_ros::task
