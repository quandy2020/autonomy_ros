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

namespace autonomy_ros {

Node::Node(const NodeOptions& node_options,
    std::unique_ptr<::autonomy::system::AutonomyNode> autonomy,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node,
    bool collect_metrics)
    : node_{node}
{
    occupancy_grid_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        kOccupancyGridTopic, rclcpp::QoS(1).transient_local());

    global_trajectory_publisher_ = node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
        kGlobalPlanTopic, rclcpp::QoS(1).transient_local());

    local_trajectory_publisher_ = node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
        kLocalPlanTopic, rclcpp::QoS(1).transient_local());

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

void Node::StartupWithDefaultTopics()
{

}

void Node::HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    if (msg == nullptr) {
        return;
    }

    // if (!sensor_samplers_.odometry_sampler.Pulse()) {
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

void Node::PublishOccupancyGridMap2D()
{
    auto data = autonomy_builder_->AutonomySystemNode()->map_server()->occupancy_grid_map_data();
    if (data == nullptr) {
        LOG(ERROR) << "Publish OccupancyGrid(format) map 2D error, OccupancyGrid map data is nullptr.";
        return;
    }

    // LOG(INFO) << "Publishing occupancy grid topic " << kOccupancyGridTopic
    //         << " (frame_id: " << map_frame_id
    //         << ", resolution:" << std::to_string(resolution) << ").";

    // occupancy_grid_publisher_->publish(ToRos(*data));
}

void Node::PublishGlobalTrajectory()
{   

}

void Node::PublishLocalTrajectory()
{

}

void Node::PublishEnvPointCloudData()
{

}



}  // namespace autonomy_ros
