/*
 * Copyright 2026 autonomy_ros contributors
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

/**
 * @file
 * @brief ROS 2 node for interactive global planner simulation.
 */

#ifndef AUTONOMY_PLANNER_PLANNER_SIM_NODE_HPP_
#define AUTONOMY_PLANNER_PLANNER_SIM_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace autonomy_planner {

/**
 * @class PlannerSimNode
 * @brief Loads a static map, fuses obstacle clouds, and plans when both
 *        /initialpose and /goal_pose have been received.
 *
 * RViz **2D Pose Estimate** sets the robot start (fake_robot + TF).
 * RViz **2D Nav Goal** sets the target. Planning runs once both are set,
 * then the node waits for the next pair of poses.
 */
class PlannerSimNode : public rclcpp::Node
{
public:
  /** @brief Declares parameters, loads map/planners, and starts timers. */
  PlannerSimNode();

  /** @brief Stops costmap and cancels periodic timer. */
  ~PlannerSimNode() override;

private:
  struct RobotPoseState {
    bool pending{false};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct PlanRequest {
    bool have_initial{false};
    bool have_goal{false};
    autonomy::commsgs::geometry_msgs::PoseStamped initial;
    autonomy::commsgs::geometry_msgs::PoseStamped goal;
  };

  void DeclareParameters();
  void LoadParameters();
  void InitTransform();
  void SetupRosInterfaces();
  void SetupCostmap();
  void SetupPlanners();

  bool LoadStaticMap(const std::string & map_file);
  void PublishStaticMap();
  void PublishMapToOdomTf();

  void PublishRobotPose(double x, double y, double yaw);
  void SyncRobotState(double x, double y, double yaw);

  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void OnInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OnTick();

  void ApplyMapGridMetadata(nav_msgs::msg::OccupancyGrid * msg) const;

  bool TransformPoseToFrame(
    autonomy::commsgs::geometry_msgs::PoseStamped * pose,
    const std::string & target_frame) const;

  void TryPlanIfReady();
  void ResetPlanRequest();
  void UpdateAndPublishCostmap();
  bool PlanToGoal(
    const autonomy::commsgs::geometry_msgs::PoseStamped & start,
    const autonomy::commsgs::geometry_msgs::PoseStamped & goal);
  autonomy::planning::common::GlobalPlanner::SharedPtr GetActivePlanner() const;

  std::string frame_id_;
  std::string base_frame_;
  std::string map_frame_;
  std::string planner_id_;
  std::string configuration_directory_;
  std::string map_topic_;
  double costmap_publish_hz_{5.0};

  bool have_static_map_{false};
  autonomy::commsgs::map_msgs::OccupancyGrid static_map_;

  autonomy::planning::proto::PlannerOptions planner_options_;
  std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<autonomy::map::costmap_2d::Costmap2DWrapper> costmap_;
  std::unordered_map<std::string, autonomy::planning::common::GlobalPlanner::SharedPtr>
  planners_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};

  std::mutex cloud_mutex_;
  autonomy::commsgs::sensor_msgs::PointCloud2 latest_cloud_;
  bool have_cloud_{false};

  std::mutex costmap_mutex_;
  std::mutex plan_mutex_;
  std::mutex plan_request_mutex_;
  autonomy::commsgs::planning_msgs::Path latest_plan_;
  PlanRequest plan_request_;
  RobotPoseState robot_pose_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr set_pose_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace autonomy_planner

#endif  // AUTONOMY_PLANNER_PLANNER_SIM_NODE_HPP_
