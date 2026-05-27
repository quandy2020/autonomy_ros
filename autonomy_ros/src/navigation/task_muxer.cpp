#include "autonomy_ros/navigation/task_muxer.hpp"

#include "autonomy_msgs/msg/task_type.hpp"

namespace autonomy_ros::navigation
{

uint8_t TaskMuxer::priorityFor(const uint8_t task_type, const bool /*force_preempt*/)
{
  using autonomy_msgs::msg::TaskType;
  switch (task_type) {
    case TaskType::WAYPOINTS:
      return 55;
    case TaskType::NAVIGATION:
      return 50;
    default:
      return 0;
  }
}

bool TaskMuxer::canAcquire(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const uint8_t priority = priorityFor(task_type, force_preempt);
  if (!active_) {
    return true;
  }
  if (active_->task_id == task_id) {
    return true;
  }
  return force_preempt || priority > active_->priority;
}

TaskMuxer::AcquireResult TaskMuxer::acquire(
  const std::string & task_id, const uint8_t task_type, const bool force_preempt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_preempted_id_.reset();
  const uint8_t priority = priorityFor(task_type, force_preempt);

  if (!active_) {
    active_ = ActiveTask{task_id, task_type, priority};
    return AcquireResult::Acquired;
  }

  if (active_->task_id == task_id) {
    active_->task_type = task_type;
    active_->priority = priority;
    return AcquireResult::Acquired;
  }

  const bool can_preempt = force_preempt || priority > active_->priority;
  if (can_preempt) {
    last_preempted_id_ = active_->task_id;
    active_ = ActiveTask{task_id, task_type, priority};
    return AcquireResult::PreemptedPrevious;
  }

  return AcquireResult::RejectedBusy;
}

void TaskMuxer::release(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_ && active_->task_id == task_id) {
    active_.reset();
  }
}

void TaskMuxer::cancelAll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_.reset();
}

bool TaskMuxer::ownsTask(const std::string & task_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_ && active_->task_id == task_id;
}

bool TaskMuxer::hasActiveTask() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_.has_value();
}

std::optional<TaskMuxer::ActiveTask> TaskMuxer::activeTask() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

std::optional<std::string> TaskMuxer::consumeLastPreempted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto id = last_preempted_id_;
  last_preempted_id_.reset();
  return id;
}

}  // namespace autonomy_ros::navigation
