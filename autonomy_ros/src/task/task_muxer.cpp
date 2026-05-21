#include "autonomy_ros/task/task_muxer.hpp"

#include "autonomy_msgs/msg/task_type.hpp"

namespace autonomy_ros::task
{

uint8_t TaskMuxer::priorityFor(uint8_t task_type, bool force_preempt)
{
  using autonomy_msgs::msg::TaskType;
  switch (task_type) {
    case TaskType::TELEOP:
      return force_preempt ? 100 : 85;
    case TaskType::DOCK:
      return 75;
    case TaskType::GUIDED_TOUR:
      return 65;
    case TaskType::WAYPOINTS:
      return 55;
    case TaskType::NAVIGATION:
      return 50;
    case TaskType::FOLLOW:
      return 45;
    default:
      return 0;
  }
}

bool TaskMuxer::canAcquire(
  const std::string & task_id, uint8_t task_type, bool force_preempt) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const uint8_t pri = priorityFor(task_type, force_preempt);
  if (!active_) {
    return true;
  }
  if (active_->task_id == task_id) {
    return true;
  }
  return force_preempt || pri > active_->priority;
}

TaskMuxer::AcquireResult TaskMuxer::acquire(
  const std::string & task_id, uint8_t task_type, bool force_preempt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_preempted_id_.reset();
  const uint8_t pri = priorityFor(task_type, force_preempt);

  if (!active_) {
    active_ = ActiveTask{task_id, task_type, pri};
    return AcquireResult::Acquired;
  }

  if (active_->task_id == task_id) {
    active_->task_type = task_type;
    active_->priority = pri;
    return AcquireResult::Acquired;
  }

  const bool can_preempt = force_preempt || pri > active_->priority;
  if (can_preempt) {
    last_preempted_id_ = active_->task_id;
    active_ = ActiveTask{task_id, task_type, pri};
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

}  // namespace autonomy_ros::task
