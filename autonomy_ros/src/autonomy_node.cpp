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

namespace autonomy_ros {

Node::Node(const NodeOptions& node_options,
    std::unique_ptr<::autonomy::system::AutonomyNode> autonomy,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node,
    bool collect_metrics)
{

}

void Node::StartupWithDefaultTopics()
{

}

void Node::HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{

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

void Node::PublishTrajectoryList()
{

}

void Node::PublishEnvPointCloudData()
{

}

}  // namespace autonomy_ros
