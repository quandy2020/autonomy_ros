// Copyright 2025 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__NAVIGATION__TASK_MANAGER_HPP_
#define AUTONOMY_ROS__NAVIGATION__TASK_MANAGER_HPP_

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy_ros/system/options.hpp"
#include "autonomy_msgs/msg/error.hpp"
#include "autonomy_msgs/msg/event.hpp"
#include "autonomy_msgs/msg/task_status.hpp"
#include "autonomy_ros/navigation/task_muxer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy::system
{
class Autonomy;
}

namespace autonomy_ros::navigation
{

using system::AutonomyCoreOptions;

/** @brief Navigation task lifecycle and status publishing. */
class TaskManager
{
public:
  using StopMotionFn = std::function<void()>;

  TaskManager(
    rclcpp::Node & node,
    ::autonomy::system::Autonomy * core,
    const AutonomyCoreOptions & core_options,
    StopMotionFn stop_motion = {});

  bool canBeginTask(
    const std::string & task_id, uint8_t task_type, bool force_preempt = false) const;

  bool beginTask(
    const std::string & task_id,
    uint8_t task_type,
    bool force_preempt = false);

  void endTask(uint8_t state, const std::string & task_id);

  void endTask(
    uint8_t state, const autonomy_msgs::msg::Error & error, const std::string & task_id);

  bool ownsTask(const std::string & task_id) const;

  bool hasActiveTask() const;

  autonomy_msgs::msg::TaskStatus getStatus() const;

  bool getStatusFor(
    const std::string & task_id, autonomy_msgs::msg::TaskStatus & out_status) const;

  autonomy_msgs::msg::Error makeError(uint16_t code, const std::string & msg) const;

  bool isPaused() const;

  bool isEstop() const;

  bool pauseTask(const std::string & reason);

  bool resumeTask();

  bool cancelTask(
    const std::string & task_id, bool cancel_all, uint8_t filter_task_type = 255);

  void triggerEstop(const std::string & reason);

  void releaseEstop();

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

  std::optional<nav_msgs::msg::Path> lastPath() const;

  void setControllerEnabled(bool enabled);

  bool isControllerEnabled() const { return controller_enabled_.load(); }

  bool hasOdometry() const;

  void setProgress(float progress);

  void publishEvent(uint8_t event_type, const std::string & message = "");

private:
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
  std::atomic<bool> controller_enabled_{true};

  rclcpp::Publisher<autonomy_msgs::msg::TaskStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<autonomy_msgs::msg::Event>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace autonomy_ros::navigation

#endif  // AUTONOMY_ROS__NAVIGATION__TASK_MANAGER_HPP_
