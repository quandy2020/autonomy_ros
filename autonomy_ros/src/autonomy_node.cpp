/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy_ros/autonomy_node.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy_ros/node_constants.hpp"
#include "autonomy_ros/messages_conversion/map_msgs_converter.hpp"

namespace autonomy_ros {

namespace {

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(const std::string&, const typename MessageType::ConstSharedPtr&), const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, Node* const node) {
    return node_handle->create_subscription<MessageType>(topic, rclcpp::SensorDataQoS(), 
        [node, handler, topic](const typename MessageType::ConstSharedPtr msg) {
        (node->*handler)(topic, msg);
    });
}
    
}  // namespace

Node::Node(const NodeOptions& node_options,
    std::unique_ptr<AutonomyBridge> autonomy,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node,
    bool collect_metrics)
    : node_options_{node_options},
      node_{node},
      autonomy_builder_{std::move(autonomy)}
{
    occupancy_grid_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        kOccupancyGridTopic, rclcpp::QoS(1).transient_local());

    global_trajectory_publisher_ = node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
        kGlobalPlanTopic, rclcpp::QoS(1).transient_local());

    local_trajectory_publisher_ = node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
        kLocalPlanTopic, rclcpp::QoS(1).transient_local());

    point_cloud_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(int(kOccupancyGridPublishPeriodSec * 1000)), [this]() {
            PublishEnvPointCloudData();
    });

    occupancy_grid_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(int(kOccupancyGridPublishPeriodSec * 1000)), [this]() {
            PublishOccupancyGridMap2D();
    });

    global_trajectory_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(int(kGlobalTrajectoryPublishPeriodSec * 1000)), [this]() {
            PublishGlobalTrajectory();
    });

    local_trajectory_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(int(kLocalTrajectoryPublishPeriodSec * 1000)), [this]() {
            PublishLocalTrajectory();
    });
}

std::set<::autonomy::sensor::SensorId> Node::ComputeExpectedSensorIds(const NodeOptions& options) const
{
    using SensorId = autonomy::sensor::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> expected_topics;
    // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
    for (const std::string& topic : ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    if (options.use_imu_data) {
        expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
    }
    // Odometry is optional.
    if (options.use_odometry) {
        expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
    }
    // NavSatFix is optional.
    if (options.use_nav_sat) {
        expected_topics.insert(SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
    }
    return expected_topics;
}

void Node::StartupWithDefaultTopics()
{
    for (const std::string& topic :
            ComputeRepeatedTopicNames(kLaserScanTopic, node_options_.num_laser_scans)) {
        subscribers_.push_back({SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
            &Node::HandleLaserScanMessage, topic, node_, this),
            topic
        });
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(
            kMultiEchoLaserScanTopic, node_options_.num_multi_echo_laser_scans)) {
        subscribers_.push_back({SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
            &Node::HandleMultiEchoLaserScanMessage, topic, node_, this),
            topic
        });
    }
    for (const std::string& topic :
            ComputeRepeatedTopicNames(kPointCloud2Topic, node_options_.num_point_clouds)) {
        subscribers_.push_back({SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
            &Node::HandlePointCloud2Message, topic, node_, this),
            topic});
    }
    if (node_options_.use_imu_data) {
        subscribers_.push_back({
            SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage, kImuTopic, node_, this),
            kImuTopic
        });
    }
    if (node_options_.use_odometry) {
        subscribers_.push_back({
            SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage, kOdometryTopic, node_, this),
            kOdometryTopic
        });
    }
    if (node_options_.use_nav_sat) {
        subscribers_.push_back({  
            SubscribeWithHandler<sensor_msgs::msg::NavSatFix>( &Node::HandleNavSatFixMessage, kNavSatFixTopic, node_, this),
            kNavSatFixTopic
        });
    }
    auto expected_sensor_ids = ComputeExpectedSensorIds(node_options_);
    maybe_warn_about_topic_mismatch_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(int(kTopicMismatchCheckDelaySec * 1000)), [this]() {
            MaybeWarnAboutTopicMismatch();
        });
    for (const auto& sensor_id : expected_sensor_ids) {
        subscribed_topics_.insert(sensor_id.id);
    }
}

void Node::HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    // if (msg == nullptr || !sensor_samplers_.odometry_sampler.Pulse()) {
    //     return;
    // }

    auto sensor_bridge_ptr = autonomy_builder_->sensor_bridge();
    auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
    sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const std::string& sensor_id, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg)
{

}

void Node::HandleImuMessage(const std::string& sensor_id, const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{

}

void Node::HandleLaserScanMessage(const std::string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{

}

void Node::HandleMultiEchoLaserScanMessage(const std::string& sensor_id, const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg)
{

}

void Node::HandlePointCloud2Message(const std::string& sensor_id, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{

}

// TODO: find ROS equivalent to ros::master::getTopics
void Node::MaybeWarnAboutTopicMismatch() 
{
    //  ::ros::master::V_TopicInfo ros_topics;
    //  ::ros::master::getTopics(ros_topics);
    //  std::set<std::string> published_topics;
    //  std::stringstream published_topics_string;
    //  for (const auto& it : ros_topics) {
    //    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    //    published_topics.insert(resolved_topic);
    //    published_topics_string << resolved_topic << ",";
    //  }
    //  bool print_topics = false;
    //  for (const auto& entry : subscribers_) {
    //    int trajectory_id = entry.first;
    //    for (const auto& subscriber : entry.second) {
    //      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
    //      if (published_topics.count(resolved_topic) == 0) {
    //        LOG(WARNING) << "Expected topic \"" << subscriber.topic
    //                     << "\" (trajectory " << trajectory_id << ")"
    //                     << " (resolved topic \"" << resolved_topic << "\")"
    //                     << " but no publisher is currently active.";
    //        print_topics = true;
    //      }
    //    }
    //  }
    //  if (print_topics) {
    //    LOG(WARNING) << "Currently available topics are: "
    //                 << published_topics_string.str();
    //  }
}

void Node::PublishOccupancyGridMap2D()
{
    auto data = autonomy_builder_->AutonomySystemNode()->map_server()->occupancy_grid_map_data();
    if (data == nullptr) {
        LOG(ERROR) << "Publish OccupancyGrid(format) map 2D error, OccupancyGrid map data is nullptr.";
        return;
    }

    LOG(INFO) << "Publishing occupancy grid topic " << kOccupancyGridTopic
             << " (frame_id: " << node_options_.map_frame<< ").";

    occupancy_grid_publisher_->publish(ToRos(*data));
}

void Node::PublishGlobalTrajectory()
{   
    // auto data = autonomy_builder_->AutonomySystemNode()->planner_server()->trajectory_plan();
    // if (data == nullptr) {
    //     LOG(ERROR) << "Publish Global plan path error,  path data is nullptr.";
    //     return;
    // }
}   

void Node::PublishLocalTrajectory()
{

}

void Node::PublishEnvPointCloudData()
{

}

void Node::AddSensorSamplers(const NodeOptions& options)
{
//     sensor_samplers_.emplace(
//         std::piecewise_construct, std::forward_as_tuple(trajectory_id),
//         std::forward_as_tuple(
//             options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
//             options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
//             options.landmarks_sampling_ratio));
}

}  // namespace autonomy_ros
