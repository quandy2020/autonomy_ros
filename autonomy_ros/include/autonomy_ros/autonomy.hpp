// Copyright 2026 autonomy_ros contributors
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

#ifndef AUTONOMY_ROS__AUTONOMY_HPP_
#define AUTONOMY_ROS__AUTONOMY_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/system/system.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/scheduler/task_scheduler.hpp"
#include "autonomy_ros/constants.hpp"
#include "autonomy_ros/bridge/map_bridge.hpp"
#include "autonomy_ros/bridge/platform_bridge.hpp"
#include "autonomy_ros/bridge/tf_bridge.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{

/**
 * @class autonomy_ros::Autonomy
 * @brief ROS 2 facade for the autonomy core: planning, control, maps, and outbound I/O.
 *
 * Owns a single ::autonomy::system::AutonomyNode instance loaded from Lua config
 * (autonomy.config_directory / autonomy.config_file). Bridges connect ROS topics to
 * the core; CommandInterface and other modules call into this class for navigation.
 *
 * Lifecycle:
 * - Constructor: declare and read autonomy.* parameters (loadParameters).
 * - start(): load core, optionally attach TaskScheduler (BT), start Tf/Map/Platform
 *   bridges, publishers, scan/speed-limit subscriptions, and the 20 Hz control timer.
 * - shutdown(): reverse order; stops cmd_vel and cancels in-flight navigation.
 *
 * Navigation (navigateToPose):
 * - When autonomy.use_bt_navigation is true and TaskScheduler is initialized, runs
 *   NavigateToPose via behavior tree (autonomy.enable_bt_tasks must be true).
 * - Otherwise plans with PlannerServer, follows path with ControllerServer, and polls
 *   until goal tolerance, failure, cancel, or timeout (0 means 300 s default cap).
 *
 * ROS parameters (see configs/autonomy_params.yaml):
 * - autonomy.config_directory, autonomy.config_file
 * - autonomy.enable_bt_tasks, autonomy.use_bt_navigation
 * - autonomy.planner.planner_id, autonomy.planner.global_frame
 * - autonomy.controller.controller_id, goal_checker_id, progress_checker_id,
 *   autonomy.controller.goal_tolerance
 * - autonomy.enable_scan_bridge, autonomy.scan_topic
 * - autonomy.publish_costmaps, autonomy.costmap_publish_hz
 * - autonomy.publish_diagnostics
 * - autonomy.enable_speed_limit_topic, autonomy.speed_limit_topic,
 *   autonomy.speed_limit_percentage
 *
 * Outbound topics (relative to node namespace unless noted):
 * - plan (nav_msgs/Path)
 * - global_costmap, local_costmap (nav_msgs/OccupancyGrid, transient_local)
 * - /diagnostics (diagnostic_msgs/DiagnosticArray)
 *
 * Platform I/O (odom, cmd_vel, map, TF) is wired in bridge::*; topic names come from
 * autonomy_node parameters outside this class.
 */
class Autonomy
{
public:
  /**
   * @brief Construct facade and load autonomy.* parameters from the node.
   * @param node Parent ROS node (typically autonomy_node); not owned.
   */
  explicit Autonomy(rclcpp::Node & node);

  /**
   * @brief Start core, optional TaskScheduler, bridges, and outbound I/O.
   * No-op if already running. Logs an error if core fails to load.
   */
  void start();

  /**
   * @brief Stop timers, bridges, TaskScheduler, and core; safe to call when idle.
   */
  void shutdown();

  /** @return True after a successful start() until shutdown(). */
  bool isRunning() const { return running_; }

  /** @brief Mutable reference to the loaded autonomy core node. */
  ::autonomy::system::AutonomyNode & core();

  /** @brief Const reference to the loaded autonomy core node. */
  const ::autonomy::system::AutonomyNode & core() const;

  /** @brief Task context from core, or nullptr if core is not loaded. */
  ::autonomy::tasks::common::TaskContext * taskContext();

  /** @brief Const task context from core, or nullptr if core is not loaded. */
  const ::autonomy::tasks::common::TaskContext * taskContext() const;

  /** @brief Attached behavior-tree scheduler, or nullptr if BT tasks are disabled. */
  ::autonomy::tasks::scheduler::TaskScheduler * taskScheduler();

  /** @brief Const scheduler pointer; nullptr when BT path is unavailable. */
  const ::autonomy::tasks::scheduler::TaskScheduler * taskScheduler() const;

  /**
   * @brief True when navigateToPose will use TaskScheduler BT navigation.
   * Requires use_bt_navigation, enable_bt_tasks, and initialized scheduler.
   */
  bool useBehaviorTreeNavigation() const;

  /** @return True if TaskScheduler exists and passed InitializeAttached. */
  bool hasTaskScheduler() const;

  /**
   * @brief Navigate to a pose (blocking until done, canceled, or timed out).
   *
   * BT mode: runs TaskScheduler::NavigateToPose; honors cancel_checker while waiting.
   * Direct mode: planToGoal + controller follow; polls at 10 Hz with goal_tolerance_.
   *
   * @param goal Target in global_frame_ (or frame filled by caller).
   * @param cancel_checker Optional; return true to abort (also sets cancel flag in BT mode).
   * @param timeout_sec Direct-mode only; 0 uses an internal 300 s cap.
   * @return True on success (BT SUCCEEDED or direct path reached).
   */
  bool navigateToPose(
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker = nullptr,
    double timeout_sec = 0.0);

  /**
   * @brief Plan to goal and begin controller path follow (does not block).
   * Enables controller on success via setControllerEnabled(true) inside setNavigationGoal.
   */
  void setNavigationGoal(const geometry_msgs::msg::PoseStamped & goal);

  /** @brief Last planned path from planToGoal / notifyPlan, if any. */
  std::optional<nav_msgs::msg::Path> lastPath() const;

  /**
   * @brief Enable or disable autonomous cmd_vel from the controller.
   * When false, ends follow path, requests navigation cancel, and publishes zero cmd_vel.
   */
  void setControllerEnabled(bool enabled);

  /** @return Whether controlStep may tick ControllerServer and publish cmd_vel. */
  bool controllerEnabled() const;

  /** @return Goal checker tolerance from autonomy.controller.goal_tolerance (meters). */
  double goalTolerance() const;

  /** @brief Apply a speed cap to ControllerServer (absolute or percentage per param). */
  void applyControllerSpeedLimit(
    const ::autonomy::commsgs::planning_msgs::SpeedLimit & limit);

  /** @brief Clear controller speed limit (restore default max linear velocity). */
  void clearControllerSpeedLimit();

  /** @brief Set atomic flag and cancel BT task if scheduler is active. */
  void requestCancelNavigation();

  /** @return Planner global frame (autonomy.planner.global_frame). */
  const std::string & globalFrame() const { return global_frame_; }

  /**
   * @brief Register a callback invoked on each odom message from PlatformBridge.
   * @param listener Called with latest nav_msgs/Odometry; may be nullptr (ignored).
   */
  void addOdomListener(
    std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)> listener);

  /**
   * @brief Register a callback invoked when a new plan is produced.
   * @param listener Called with nav_msgs/Path after planning.
   */
  void addPlanListener(std::function<void(const nav_msgs::msg::Path &)> listener);

  /** @return True while ControllerServer is following a planned path. */
  bool isFollowingPath() const { return following_path_.load(); }

  /** @return True if OdomSmoother has received at least one odom update. */
  bool hasOdom() const;

  /** @return True if the last control tick reported FollowPath succeeded. */
  bool lastFollowPathSucceeded() const;

  /** @return True if the last control tick reported FollowPath failed. */
  bool lastFollowPathFailed() const;

  /**
   * @brief Build diagnostic snapshot (core health, controller, planner, BT state).
   * @return diagnostic_msgs/DiagnosticArray suitable for /diagnostics publisher.
   */
  diagnostic_msgs::msg::DiagnosticArray buildDiagnostics() const;

private:
  /** @brief Declare and read all autonomy.* ROS parameters. */
  void loadParameters();

  /**
   * @brief Resolve config directory: parameter value or share/autonomy/config.
   * @return Empty string on failure (core will not start).
   */
  std::string resolveConfigDirectory() const;

  /** @brief Create and Start() AutonomyNode from Lua options. */
  void startCore();

  /** @brief Attach TaskScheduler to planner/controller when enable_bt_tasks is true. */
  void startTaskScheduler();

  /** @brief Start TfBridge, MapBridge, PlatformBridge, and outbound I/O. */
  void startRosBridges();

  /** @brief Subscribe scan / speed limit; create costmap and diagnostics timers. */
  void startOutboundIo();

  /** @brief Tear down outbound subscriptions and publishers. */
  void stopOutboundIo();

  /**
   * @brief 20 Hz wall timer: tick controller when enabled, update last_follow_result_.
   */
  void controlStep();

  /** @brief LaserScan callback; feeds planner costmap via feedScanToCostmap. */
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /** @brief Float32 speed limit topic; forwards to applyControllerSpeedLimit. */
  void onSpeedLimitTopic(const std_msgs::msg::Float32::SharedPtr msg);

  /** @brief Publish global/local costmap snapshots when enabled. */
  void publishCostmaps();

  /** @brief Publish buildDiagnostics() when diagnostics are enabled. */
  void publishDiagnostics();

  /**
   * @brief Plan from current odom to goal and call BeginFollowPath.
   * @return True if a non-empty path was planned and follow started.
   */
  bool planToGoal(const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Poll follow state until success, failure, cancel, or timeout.
   * @param goal Used for planar distance check in addition to controller result.
   * @param tolerance Arrival radius (meters).
   * @param cancel_checker Optional external cancel predicate.
   * @param timeout_sec Max wait; values <= 0 use 300 s.
   * @return True if goal reached while following_path_ was active.
   */
  bool waitForDirectNavigation(
    const geometry_msgs::msg::PoseStamped & goal, double tolerance,
    std::function<bool()> cancel_checker, double timeout_sec);

  /** @return Checker that reads cancel_navigation_ atomic. */
  std::function<bool()> navigationCancelChecker();

  /** @brief Non-owning planner costmap wrapper, or nullptr. */
  ::autonomy::map::costmap_2d::Costmap2DWrapper * plannerCostmap();

  /** @brief Apply static map to planner costmap when map server publishes. */
  void applyMapToCostmap(const ::autonomy::commsgs::map_msgs::OccupancyGrid::SharedPtr & map);

  /** @brief Insert laser scan into rolling costmap. */
  void feedScanToCostmap(const ::autonomy::commsgs::sensor_msgs::LaserScan & scan);

  /**
   * @brief Copy current costmap layer to commsg grid.
   * @param grid Output occupancy grid.
   * @return False if no costmap wrapper is available.
   */
  bool snapshotCostmap(::autonomy::commsgs::map_msgs::OccupancyGrid & grid);

  /** @brief Update odom smoother and invoke registered odom listeners. */
  void dispatchOdom(const nav_msgs::msg::Odometry::SharedPtr & msg);

  /** @brief Store path, publish plan topic, and invoke plan listeners. */
  void notifyPlan(const nav_msgs::msg::Path & path);

  rclcpp::Node & node_;
  bool running_{false};

  // autonomy.config_* (Lua bootstrap)
  std::string config_directory_;
  std::string config_file_{constants::defaults::kAutonomyLuaConfig};
  bool enable_bt_tasks_{constants::defaults::kAutonomyEnableBtTasks};
  bool use_bt_navigation_{constants::defaults::kAutonomyUseBtNavigation};

  // autonomy.planner.* / autonomy.controller.*
  std::string planner_id_;
  std::string controller_id_{constants::defaults::kAutonomyControllerId};
  std::string goal_checker_id_{constants::defaults::kAutonomyGoalCheckerId};
  std::string progress_checker_id_{constants::defaults::kAutonomyProgressCheckerId};
  std::string global_frame_{constants::defaults::kAutonomyGlobalFrame};
  double goal_tolerance_{constants::defaults::kAutonomyGoalTolerance};

  ::autonomy::system::AutonomyNode::UniquePtr core_;
  std::shared_ptr<::autonomy::control::utils::OdomSmoother> odom_smoother_;
  std::unique_ptr<::autonomy::tasks::scheduler::TaskScheduler> task_scheduler_;

  std::unique_ptr<bridge::TfBridge> tf_bridge_;
  std::unique_ptr<bridge::MapBridge> map_bridge_;
  std::unique_ptr<bridge::PlatformBridge> platform_bridge_;

  // autonomy.enable_scan_bridge, scan_topic, publish_costmaps, ...
  bool scan_enabled_{constants::defaults::kAutonomyScanEnabled};
  std::string scan_topic_{constants::topics::kScan};
  bool costmaps_enabled_{constants::defaults::kAutonomyCostmapsEnabled};
  double costmap_hz_{constants::defaults::kAutonomyCostmapPublishHz};
  bool diagnostics_enabled_{constants::defaults::kAutonomyDiagnosticsEnabled};
  bool speed_limit_enabled_{constants::defaults::kAutonomySpeedLimitEnabled};
  bool speed_limit_percentage_{constants::defaults::kAutonomySpeedLimitPercentage};
  std::string speed_limit_topic_{constants::topics::kSpeedLimit};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_limit_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  rclcpp::TimerBase::SharedPtr costmap_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::atomic<bool> controller_enabled_{false};
  std::atomic<bool> bt_navigation_active_{false};
  std::atomic<bool> following_path_{false};
  std::atomic<bool> cancel_navigation_{false};
  std::atomic<int> last_follow_result_{0};

  mutable std::mutex odom_mutex_;
  std::vector<std::function<void(const nav_msgs::msg::Odometry::SharedPtr &)>> odom_listeners_;
  std::vector<std::function<void(const nav_msgs::msg::Path &)>> plan_listeners_;

  mutable std::mutex path_mutex_;
  std::optional<nav_msgs::msg::Path> last_path_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__AUTONOMY_HPP_
