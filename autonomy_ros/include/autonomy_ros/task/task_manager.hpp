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

#ifndef AUTONOMY_ROS__TASK__TASK_MANAGER_HPP_
#define AUTONOMY_ROS__TASK__TASK_MANAGER_HPP_

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy_msgs/msg/battery_status.hpp"
#include "autonomy_ros/options.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "autonomy_msgs/msg/error.hpp"
#include "autonomy_msgs/msg/event.hpp"
#include "autonomy_msgs/msg/task_status.hpp"
#include "autonomy_ros/task/task_muxer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy::system
{
class Autonomy;
}

namespace autonomy_ros::task
{

/**
 * @class autonomy_ros::task::TaskManager
 * @brief Central task lifecycle, safety flags, and outward-facing status/events
 *
 * Combines TaskMuxer arbitration with autonomy_msgs/TaskStatus publishing.
 * Used by CommandInterface for all action/service handlers.
 *
 * Published topics:
 * - autonomy/status (TaskStatus, ~5 Hz)
 * - autonomy/events (Event, on state edges)
 * - autonomy/battery (BatteryStatus, ~5 Hz, from parameter task.battery_percent)
 *
 * Thread-safe status reads via getStatus(); mutation under mutex_.
 */
class TaskManager
{
public:
  using StopMotionFn = std::function<void()>;

  /**
   * @brief Construct TaskManager: declare parameters, publishers, and status timer.
   * @param node Parent node for publishers, timers, and parameters
   * @param core Autonomy core for motion / navigation execution
   * @param core_options Planner / controller options mirrored from ROS params
   * @param stop_motion Optional hook when controller is disabled (e.g. publish zero cmd_vel)
   */
  TaskManager(
    rclcpp::Node & node,
    ::autonomy::system::Autonomy * core,
    const AutonomyCoreOptions & core_options,
    StopMotionFn stop_motion = {});

  /**
   * @brief Check estop and muxer without acquiring (action goal callback)
   * @param task_id Proposed task id
   * @param task_type autonomy_msgs/TaskType value
   * @param force_preempt Allow teleop-style preemption
   * @return True if beginTask would likely succeed
   */
  bool canBeginTask(
    const std::string & task_id, uint8_t task_type, bool force_preempt = false) const;

  /**
   * @brief Start a task: acquire muxer and set RUNNING status
   * @param task_id Client task id (must match action goal)
   * @param task_type Task category for status and priority
   * @param tour_id Guided tour id, empty for non-tour tasks
   * @param force_preempt Preempt lower priority if true
   * @return False on estop, RejectedBusy, or invalid state
   */
  bool beginTask(
    const std::string & task_id,
    uint8_t task_type,
    const std::string & tour_id = "",
    bool force_preempt = false);

  /**
   * @brief End task with success and no error payload
   * @param state Terminal TaskState (SUCCEEDED, FAILED, CANCELED)
   * @param task_id Must match the task being finished
   */
  void endTask(uint8_t state, const std::string & task_id);

  /**
   * @brief End task with explicit error; releases muxer always
   * @param state Terminal TaskState
   * @param error Populated Error message
   * @param task_id Task id; status updated only if still active owner
   */
  void endTask(
    uint8_t state, const autonomy_msgs::msg::Error & error, const std::string & task_id);

  /**
   * @brief Whether task_id still holds the muxer slot
   * @param task_id Task from action execution thread
   * @return True if not preempted
   */
  bool ownsTask(const std::string & task_id) const;

  /**
   * @brief Whether muxer has an active owner
   * @return True if a task is running
   */
  bool hasActiveTask() const;

  /**
   * @brief Current TaskStatus with odom pose and battery filled in
   * @return Snapshot safe for external clients
   */
  autonomy_msgs::msg::TaskStatus getStatus() const;

  /**
   * @brief Query status for a specific task_id
   * @param task_id Empty string returns active task; otherwise must match active id
   * @param out_status Filled on success
   * @return False if task_id unknown (not the active task)
   */
  bool getStatusFor(
    const std::string & task_id, autonomy_msgs::msg::TaskStatus & out_status) const;

  /**
   * @brief Factory for autonomy_msgs/Error
   * @param code Error.msg constant (e.g. Error::TASK_CONFLICT)
   * @param msg Human-readable detail
   * @return Error message
   */
  autonomy_msgs::msg::Error makeError(uint16_t code, const std::string & msg) const;

  /**
   * @brief Pause flag for execution loops
   * @return True if PauseTask was called
   */
  bool isPaused() const;

  /**
   * @brief Emergency stop engaged
   * @return True after TriggerEmergencyStop(engage=true)
   */
  bool isEstop() const;

  /**
   * @brief Pause active task (requires hasActiveTask)
   * @param reason Stored in status.description; publishes TOUR_PAUSED
   * @return False if no active task
   */
  bool pauseTask(const std::string & reason);

  /**
   * @brief Clear pause flag (requires active task, not estop)
   * @return False if no active task or estop engaged
   */
  bool resumeTask();

  /**
   * @brief Cancel task and clear muxer
   * @param task_id Target id; empty matches active only
   * @param cancel_all Ignore task_id mismatch
   * @param filter_task_type TaskType value to match, 0 or 255 = any
   * @return False if filter/id does not match
   */
  bool cancelTask(
    const std::string & task_id, bool cancel_all, uint8_t filter_task_type = 255);

  /**
   * @brief Latch estop, pause, and publish EMERGENCY_STOP event
   * @param reason Logged in status and event.message
   */
  void triggerEstop(const std::string & reason);

  /**
   * @brief Clear estop latch (does not auto-resume motion)
   */
  void releaseEstop();

  /**
   * @brief Visitor/staff pressed continue at exhibit (ContinueTour service)
   */
  void requestContinueTour();

  /**
   * @brief Take continue latch (guided tour wait loop)
   * @return True once per requestContinueTour call
   */
  bool consumeContinueTour();

  /**
   * @brief Queue skip to exhibit by index and/or id
   * @param index Used if exhibit_id empty or not found in tour list
   * @param exhibit_id Preferred match in consumeSkipExhibitForTour
   */
  void requestSkipExhibit(uint32_t index, const std::string & exhibit_id);

  /**
   * @brief Resolve and consume skip request during GuidedTour
   * @param exhibit_ids Parallel list of exhibit_id from tour goal
   * @return Index to jump to, or nullopt if no pending skip
   */
  std::optional<uint32_t> consumeSkipExhibitForTour(
    const std::vector<std::string> & exhibit_ids);

  /**
   * @brief Set simulated/report battery level
   * @param percent 0–100, reflected in status and autonomy/battery
   */
  void setBatteryPercent(float percent);

  /**
   * @brief Compare battery to threshold
   * @param threshold_percent Low-battery threshold (e.g. tour goal field)
   * @return True if current percent is below threshold
   */
  bool isLowBattery(float threshold_percent) const;

  /**
   * @brief Fuse odometry into TaskStatus.current_pose
   * @param odom Latest /odom from CommandInterface subscription
   */
  void updateOdom(const nav_msgs::msg::Odometry & odom);

  const AutonomyCoreOptions & coreOptions() const { return core_options_; }

  bool navigateToPose(
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker = nullptr,
    double timeout_sec = 0.0);

  bool navigateThroughPoses(
    const std::vector<geometry_msgs::msg::PoseStamped> & goals,
    std::function<bool()> cancel_checker = nullptr,
    double timeout_sec = 0.0);

  void replanToGoal(const geometry_msgs::msg::PoseStamped & goal);

  std::optional<nav_msgs::msg::Path> lastPath() const;

  void setControllerEnabled(bool enabled);

  bool isControllerEnabled() const { return controller_enabled_.load(); }

  bool hasOdometry() const;

  bool teleopDrive(
    double time_allowance_sec,
    double max_linear_vel,
    double max_angular_vel,
    std::function<bool()> cancel_checker = nullptr);

  void beginTeleop(double max_linear_vel = 0.0, double max_angular_vel = 0.0);

  void endTeleop();

  bool isTeleopActive() const;

  void updateTeleopCommand(const geometry_msgs::msg::TwistStamped & cmd);

  std::optional<::autonomy::commsgs::geometry_msgs::TwistStamped> teleopCommand() const;

  bool transformPoseToGlobalFrame(
    ::autonomy::commsgs::geometry_msgs::PoseStamped & pose);

  /**
   * @brief Update normalized progress field in status
   * @param progress Value in [0, 1]
   */
  void setProgress(float progress);

  /**
   * @brief Set exhibit and narration ids for AV integration
   * @param exhibit_id Current exhibit
   * @param narration_id TTS/content id at exhibit
   */
  void setNarration(const std::string & exhibit_id, const std::string & narration_id);

  /**
   * @brief Update status.description for wait-at-exhibit semantics
   * @param waiting True sets waiting_for_continue description
   */
  void setWaitingForContinue(bool waiting);

  /**
   * @brief Publish autonomy_msgs/Event
   * @param event_type Constant from Event.msg
   * @param message Optional detail (e.g. preempted task ids)
   */
  void publishEvent(uint8_t event_type, const std::string & message = "");

private:
  /**
   * @brief Publish battery topic and autonomy/status
   */
  void publishStatus();

  rclcpp::Node & node_;
  ::autonomy::system::Autonomy * core_{nullptr};
  AutonomyCoreOptions core_options_;
  StopMotionFn stop_motion_;
  TaskMuxer muxer_;
  mutable std::mutex mutex_;
  autonomy_msgs::msg::TaskStatus status_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  std::atomic<bool> paused_{false};
  std::atomic<bool> estop_{false};
  std::atomic<bool> teleop_active_{false};
  std::atomic<bool> controller_enabled_{true};
  std::atomic<bool> continue_requested_{false};
  std::optional<uint32_t> skip_exhibit_index_;
  std::optional<std::string> skip_exhibit_id_;
  float battery_percent_{100.0f};

  mutable std::mutex teleop_mutex_;
  ::autonomy::commsgs::geometry_msgs::TwistStamped teleop_command_;
  double max_teleop_linear_{0.0};
  double max_teleop_angular_{0.0};

  rclcpp::Publisher<autonomy_msgs::msg::TaskStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<autonomy_msgs::msg::BatteryStatus>::SharedPtr battery_pub_;
  rclcpp::Publisher<autonomy_msgs::msg::Event>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace autonomy_ros::task

#endif  // AUTONOMY_ROS__TASK__TASK_MANAGER_HPP_
