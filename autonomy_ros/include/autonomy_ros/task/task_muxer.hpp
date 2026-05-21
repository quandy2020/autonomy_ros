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

#ifndef AUTONOMY_ROS__TASK__TASK_MUXER_HPP_
#define AUTONOMY_ROS__TASK__TASK_MUXER_HPP_

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace autonomy_ros::task
{

/**
 * @class autonomy_ros::task::TaskMuxer
 * @brief Single-slot task arbitrator with fixed priority ordering
 *
 * Only one external task may own execution at a time. A higher-priority
 * task (or force_preempt, e.g. Teleop) can replace the active owner;
 * the previous owner must detect loss via ownsTask() and exit cleanly.
 *
 * Default priority (high to low): TELEOP(100/85) > DOCK > GUIDED_TOUR >
 * WAYPOINTS > NAVIGATION > FOLLOW.
 */
class TaskMuxer
{
public:
  /**
   * @brief Outcome of acquire()
   */
  enum class AcquireResult : uint8_t
  {
    Acquired = 0,           ///< Slot was free or same task_id refreshed
    PreemptedPrevious = 1,  ///< Replaced an existing lower-priority task
    RejectedBusy = 2,       ///< Active task blocks this request
  };

  /**
   * @brief Metadata for the task currently holding the slot
   */
  struct ActiveTask
  {
    std::string task_id;   ///< Client-supplied id from action goal
    uint8_t task_type{0}; ///< autonomy_msgs/TaskType value
    uint8_t priority{0};  ///< Internal arbitration rank
  };

  /**
   * @brief Take ownership of the execution slot
   * @param task_id Unique task identifier from the client
   * @param task_type autonomy_msgs/TaskType constant
   * @param force_preempt If true, preempt equal or lower priority (Teleop)
   * @return Acquired, PreemptedPrevious, or RejectedBusy
   */
  AcquireResult acquire(
    const std::string & task_id, uint8_t task_type, bool force_preempt = false);

  /**
   * @brief Preview acquire without changing state (for action goal validation)
   * @param task_id Task identifier to check
   * @param task_type Task type for priority lookup
   * @param force_preempt Teleop-style preemption flag
   * @return True if acquire would not return RejectedBusy
   */
  bool canAcquire(
    const std::string & task_id, uint8_t task_type, bool force_preempt = false) const;

  /**
   * @brief Release slot when task_id matches active owner
   * @param task_id Task finishing or canceled
   */
  void release(const std::string & task_id);

  /**
   * @brief Force-clear slot (CancelTask service)
   */
  void cancelAll();

  /**
   * @brief Check if task_id is the current owner
   * @param task_id Task to test
   * @return True if active and ids match
   */
  bool ownsTask(const std::string & task_id) const;

  /**
   * @brief Whether any task holds the slot
   * @return True if active_ is set
   */
  bool hasActiveTask() const;

  /**
   * @brief Read-only view of active task
   * @return ActiveTask or nullopt
   */
  std::optional<ActiveTask> activeTask() const;

  /**
   * @brief Id of task preempted by the last PreemptedPrevious acquire (one-shot)
   * @return Preempted task_id, then clears internal storage
   */
  std::optional<std::string> consumeLastPreempted();

private:
  /**
   * @brief Map TaskType to numeric priority
   * @param task_type autonomy_msgs/TaskType value
   * @param force_preempt Use maximum priority for teleop preemption
   * @return Priority byte (higher wins)
   */
  static uint8_t priorityFor(uint8_t task_type, bool force_preempt);

  mutable std::mutex mutex_;
  std::optional<ActiveTask> active_;
  std::optional<std::string> last_preempted_id_;
};

}  // namespace autonomy_ros::task

#endif  // AUTONOMY_ROS__TASK__TASK_MUXER_HPP_
