/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_ROS__MANAGER_HPP_
#define AUTONOMY_ROS__MANAGER_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy_ros/options.hpp"
#include "autonomy_msgs/msg/error.hpp"
#include "autonomy_msgs/msg/event.hpp"
#include "autonomy_msgs/msg/task_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy::system
{
class Autonomy;
}

namespace autonomy_ros
{

/** @brief Navigation task lifecycle, arbitration, and status publishing. */
class TaskManager
{
public:
  using StopMotionFn = std::function<void()>;

  TaskManager(
    rclcpp::Node & node,
    ::autonomy::system::Autonomy * core,
    const CoreOptions & core_options,
    StopMotionFn stop_motion = {});

  bool CanBeginTask(
    const std::string & task_id, uint8_t task_type, bool force_preempt = false) const;

  bool BeginTask(const std::string & task_id, uint8_t task_type, bool force_preempt = false);

  void EndTask(uint8_t state, const std::string & task_id);

  void EndTask(
    uint8_t state, const autonomy_msgs::msg::Error & error, const std::string & task_id);

  bool OwnsTask(const std::string & task_id) const;

  bool HasActiveTask() const;

  autonomy_msgs::msg::TaskStatus GetStatus() const;

  bool GetStatusFor(
    const std::string & task_id, autonomy_msgs::msg::TaskStatus & out_status) const;

  autonomy_msgs::msg::Error MakeError(uint16_t code, const std::string & msg) const;

  bool IsPaused() const;

  bool IsEstop() const;

  bool PauseTask(const std::string & reason);

  bool ResumeTask();

  bool CancelTask(
    const std::string & task_id, bool cancel_all, uint8_t filter_task_type = 255);

  void TriggerEstop(const std::string & reason);

  void ReleaseEstop();

  void UpdateOdom(const nav_msgs::msg::Odometry & odom);

  const CoreOptions & GetCoreOptions() const { return core_options_; }

  bool NavigateToPose(
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker = nullptr,
    double timeout_sec = 0.0);

  bool NavigateThroughPoses(
    const std::vector<geometry_msgs::msg::PoseStamped> & goals,
    std::function<bool()> cancel_checker = nullptr,
    double timeout_sec = 0.0);

  std::optional<nav_msgs::msg::Path> LastPath() const;

  void SetControllerEnabled(bool enabled);

  bool IsControllerEnabled() const { return controller_enabled_.load(); }

  bool HasOdometry() const;

  void SetProgress(float progress);

  void PublishEvent(uint8_t event_type, const std::string & message = "");

private:
  enum class AcquireResult : uint8_t
  {
    kAcquired = 0,
    kPreemptedPrevious = 1,
    kRejectedBusy = 2,
  };

  struct ActiveTask
  {
    std::string task_id;
    uint8_t task_type{0};
    uint8_t priority{0};
  };

  static uint8_t PriorityFor(uint8_t task_type);

  bool CanAcquire(
    const std::string & task_id, uint8_t task_type, bool force_preempt) const;

  AcquireResult Acquire(
    const std::string & task_id, uint8_t task_type, bool force_preempt);

  void Release(const std::string & task_id);

  void CancelAllTasks();

  std::optional<std::string> ConsumeLastPreempted();

  void PublishStatus();

  rclcpp::Node & node_;
  ::autonomy::system::Autonomy * core_{nullptr};
  CoreOptions core_options_;
  StopMotionFn stop_motion_;

  mutable std::mutex muxer_mutex_;
  std::optional<ActiveTask> active_task_;
  std::optional<std::string> last_preempted_id_;

  mutable std::mutex status_mutex_;
  autonomy_msgs::msg::TaskStatus status_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  std::atomic<bool> paused_{false};
  std::atomic<bool> estop_{false};
  std::atomic<bool> controller_enabled_{true};

  rclcpp::Publisher<autonomy_msgs::msg::TaskStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<autonomy_msgs::msg::Event>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__MANAGER_HPP_
