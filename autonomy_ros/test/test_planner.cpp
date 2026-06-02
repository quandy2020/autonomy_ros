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

#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "autolink/action/create_client.hpp"
#include "autolink/action/types.hpp"
#include "autolink/autolink.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/planning/constants.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/navigator/navigators/action_type.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"
#include "autonomy_ros/conversions/map_msgs.hpp"
#include "autonomy_ros/conversions/planning_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace autonomy_ros
{
namespace
{

std::string ResolveMapYamlPath(
  const std::string & configuration_directory,
  const std::string & map_yaml_override)
{
  const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
    configuration_directory);
  ::autonomy::common::ConfigurationFileResolver resolver(dirs);

  if (map_yaml_override.empty()) {
    return resolver.GetFullPathOrDie("data/map.yaml");
  }
  if (!map_yaml_override.empty() && map_yaml_override.front() == '/') {
    return map_yaml_override;
  }

  std::string relative = map_yaml_override;
  constexpr const char * kConfigPrefix = "config/";
  if (relative.rfind(kConfigPrefix, 0) == 0) {
    relative = relative.substr(std::char_traits<char>::length(kConfigPrefix));
  }
  return resolver.GetFullPathOrDie(relative);
}

}  // namespace

class PlannerRosTester : public rclcpp::Node
{
public:
  PlannerRosTester()
  : Node("planner_ros_tester")
  {
    declare_parameter<std::string>("configuration_directory", "");
    declare_parameter<std::string>("map_yaml", "");
    declare_parameter<std::string>("planner_id", "");

    const std::string config_dir = get_parameter("configuration_directory").as_string();
    if (config_dir.empty()) {
      throw std::runtime_error("parameter 'configuration_directory' is required");
    }

    auto options = autonomy::planning::CreateOptions(config_dir);
    planner_ = std::make_unique<autonomy::planning::PlannerServer>(options);

    frame_id_ = options.costmap().frame_id().empty() ? "map" : options.costmap().frame_id();
    const std::string planner_id_param = get_parameter("planner_id").as_string();
    if (!planner_id_param.empty()) {
      planner_id_ = planner_id_param;
    } else {
      planner_id_ = options.default_planner_id().empty()
        ? "navfn_planner" : options.default_planner_id();
    }

    costmap_wrapper_ = planner_->GetCostmapWrapper();
    auto * wrapper = costmap_wrapper_.get();
    auto * costmap = wrapper ? wrapper->getCostmap() : nullptr;
    if (costmap == nullptr) {
      throw std::runtime_error("PlannerServer costmap is null");
    }

    const std::string map_yaml =
      ResolveMapYamlPath(config_dir, get_parameter("map_yaml").as_string());
    if (autonomy::map::costmap_2d::loadMapFromYaml(map_yaml, raw_map_) !=
      autonomy::map::costmap_2d::LOAD_MAP_SUCCESS)
    {
      throw std::runtime_error("failed to load raw map yaml: " + map_yaml);
    }
    has_raw_map_ = true;
    if (!wrapper->loadMap(map_yaml)) {
      throw std::runtime_error("failed to load map yaml: " + map_yaml);
    }
    wrapper->updateMap();
    wrapper->Stop();
    RCLCPP_INFO(get_logger(), "Loaded map yaml: %s", map_yaml.c_str());

    path_pub_ = create_publisher<nav_msgs::msg::Path>("planner_test/path", 10);
    auto durable_qos = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", durable_qos);
    global_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "global_costmap", durable_qos);
    start_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("planner_test/start", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("planner_test/goal", 10);

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      std::bind(&PlannerRosTester::OnGoalPose, this, std::placeholders::_1));

    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    PublishStaticBaseTransform();

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PlannerRosTester::PublishMapAndCostmap, this));

    client_node_ = autolink::CreateNode("planner_ros_tester_client");
    if (!client_node_) {
      throw std::runtime_error("Failed to create autolink client node");
    }
    compute_path_client_ = autolink::action::CreateClient<
      autonomy::navigator::behavior_tree::ComputePathToPoseActionTraits>(
      client_node_, autonomy::planning::kComputePathToPoseActionName);

    RCLCPP_INFO(
      get_logger(),
      "Planner tester started. frame=%s planner_id=%s path_topic=%s input_topic=%s action=%s",
      frame_id_.c_str(), planner_id_.c_str(), "planner_test/path", "goal_pose",
      autonomy::planning::kComputePathToPoseActionName);
  }

private:
  void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    auto pose = autonomy_ros::fromRos(*msg);
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = frame_id_;
    }
    ++goal_pose_count_;

    if (goal_pose_count_ == 1) {
      start_pose_ = pose;
      PublishPose(start_pub_, start_pose_);
      RCLCPP_INFO(
        get_logger(),
        "Received first goal_pose as start: (%.3f, %.3f) frame=%s",
        start_pose_.pose.position.x, start_pose_.pose.position.y,
        start_pose_.header.frame_id.c_str());
      return;
    }

    goal_pose_ = pose;
    PublishPose(goal_pub_, goal_pose_);
    RCLCPP_INFO(
      get_logger(),
      "Received second goal_pose as goal: (%.3f, %.3f) frame=%s",
      goal_pose_.pose.position.x, goal_pose_.pose.position.y,
      goal_pose_.header.frame_id.c_str());

    PlanOnceWithClient();
    goal_pose_count_ = 0;
  }

  void PublishPathResult(const autonomy::commsgs::planning_msgs::Path & path)
  {
    nav_msgs::msg::Path ros_path = autonomy_ros::toRos(path);
    ros_path.header.stamp = now();
    if (ros_path.header.frame_id.empty()) {
      ros_path.header.frame_id = frame_id_;
    }
    path_pub_->publish(ros_path);
    PublishPose(start_pub_, start_pose_);
    PublishPose(goal_pub_, goal_pose_);
  }

  void PlanOnceWithClient()
  {
    if (goal_pose_count_ < 2) {
      return;
    }
    if (!compute_path_client_) {
      RCLCPP_ERROR(get_logger(), "compute_path_to_pose action client is null");
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (autolink::OK() && !compute_path_client_->ActionServerIsReady()) {
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_ERROR(get_logger(), "compute_path_to_pose action server not ready");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    using ActionTraits = autonomy::navigator::behavior_tree::ComputePathToPoseActionTraits;
    using GoalHandle = autolink::action::ClientGoalHandle<ActionTraits>;
    namespace task_proto = autonomy::navigator::behavior_tree::task_proto;

    ActionTraits::Goal goal;
    goal.set_use_start(true);
    goal.set_planner_id(planner_id_);
    *goal.mutable_start() = autonomy::commsgs::geometry_msgs::ToProto(start_pose_);
    *goal.mutable_goal() = autonomy::commsgs::geometry_msgs::ToProto(goal_pose_);

    constexpr auto kAcceptTimeout = std::chrono::seconds(30);
    constexpr auto kResultTimeout = std::chrono::seconds(60);

    const auto accepted_future = compute_path_client_->AsyncSendGoal(goal);
    if (accepted_future.wait_for(kAcceptTimeout) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for goal acceptance");
      return;
    }

    const std::shared_ptr<GoalHandle> handle = accepted_future.get();
    if (!handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected by compute_path_to_pose action server");
      return;
    }

    auto result_future = handle->AsyncGetResult();
    if (result_future.wait_for(kResultTimeout) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for compute_path_to_pose result");
      return;
    }

    const typename GoalHandle::WrappedResult wrapped = result_future.get();
    if (wrapped.code != autolink::action::ResultCode::SUCCEEDED || !wrapped.result) {
      const std::string detail =
        wrapped.result && !wrapped.result->error_msg().empty()
        ? (" — " + wrapped.result->error_msg()) : "";
      RCLCPP_ERROR(
        get_logger(), "compute_path_to_pose failed: %s%s",
        autolink::action::ToString(wrapped.code).c_str(), detail.c_str());
      return;
    }

    if (wrapped.result->path().poses_size() < 2) {
      RCLCPP_ERROR(get_logger(), "compute_path_to_pose returned invalid path");
      return;
    }
    if (wrapped.result->error_code() != task_proto::COMPUTE_PATH_TO_POSE_NONE) {
      RCLCPP_ERROR(
        get_logger(), "compute_path_to_pose error_code=%d msg=%s",
        wrapped.result->error_code(), wrapped.result->error_msg().c_str());
      return;
    }

    const auto path =
      autonomy::commsgs::planning_msgs::FromProto(wrapped.result->path());
    PublishPathResult(path);
    RCLCPP_INFO(
      get_logger(),
      "Planned via action client. path poses=%zu",
      path.poses.size());
  }
  void PublishMapAndCostmap()
  {
    if (has_raw_map_) {
      auto ros_map = autonomy_ros::toRos(raw_map_);
      ros_map.header.stamp = now();
      if (ros_map.header.frame_id.empty()) {
        ros_map.header.frame_id = frame_id_;
      }
      map_pub_->publish(ros_map);
    }

    nav_msgs::msg::OccupancyGrid ros_grid;
    bool has_snapshot = false;
    if (costmap_wrapper_) {
      autonomy::commsgs::map_msgs::OccupancyGrid grid;
      if (costmap_wrapper_->snapshotOccupancyGrid(grid)) {
        ros_grid = autonomy_ros::toRos(grid);
        has_snapshot = true;
      }
    }
    if (!has_snapshot && has_raw_map_) {
      ros_grid = autonomy_ros::toRos(raw_map_);
    }
    if (!has_snapshot && !has_raw_map_) {
      return;
    }

    ros_grid.header.stamp = now();
    if (ros_grid.header.frame_id.empty()) {
      ros_grid.header.frame_id = frame_id_;
    }

    global_costmap_pub_->publish(ros_grid);
  }

  void PublishStaticBaseTransform()
  {
    if (!static_tf_broadcaster_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf);
  }

  void PublishPose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
    const autonomy::commsgs::geometry_msgs::PoseStamped & pose)
  {
    auto ros_pose = autonomy_ros::toRos(pose);
    ros_pose.header.stamp = now();
    pub->publish(ros_pose);
  }

  std::unique_ptr<autonomy::planning::PlannerServer> planner_;
  std::shared_ptr<autolink::Node> client_node_;
  std::shared_ptr<autolink::action::Client<
    autonomy::navigator::behavior_tree::ComputePathToPoseActionTraits>> compute_path_client_;
  autonomy::map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_;
  autonomy::commsgs::map_msgs::OccupancyGrid raw_map_;
  bool has_raw_map_{false};
  std::string frame_id_;
  std::string planner_id_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  autonomy::commsgs::geometry_msgs::PoseStamped start_pose_;
  autonomy::commsgs::geometry_msgs::PoseStamped goal_pose_;
  int goal_pose_count_{0};
};
}  // namespace autonomy_ros

int main(int argc, char ** argv)
{
  if (!autolink::Init(argv[0])) {
    return 1;
  }

  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<autonomy_ros::PlannerRosTester>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    std::cerr << "planner_ros_tester error: " << ex.what() << std::endl;
    rc = 1;
  }

  rclcpp::shutdown();
  autolink::Clear();
  return rc;
}
