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


#pragma once 

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonomy/common/macros.hpp"
#include "autonomy/system/system.hpp"
#include "autonomy_ros/autonomoy_bridge.hpp"
#include "autonomy_ros/node_constants.hpp"
#include "autonomy_ros/node_options.hpp"

namespace autonomy_ros {

class Node
{
public:

    /**
     * @brief Construct a new Node object
     * 
     * @param node_options 
     * @param autonomy 
     * @param tf_buffer 
     * @param node 
     * @param collect_metrics 
     */
    Node(const NodeOptions& node_options,
        std::unique_ptr<::autonomy::system::AutonomyNode> autonomy,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        rclcpp::Node::SharedPtr node,
        bool collect_metrics);

    ~Node() = default;

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    /**
     * @brief Starts with the default topics.
     * 
     * @param options 
     */
    void StartupWithDefaultTopics();

    // The following functions handle adding sensor data.
    void HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

    void HandleNavSatFixMessage(const std::string& sensor_id, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);

    void HandleImuMessage(const std::string& sensor_id, const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    void HandleLaserScanMessage(const std::string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

    void HandleMultiEchoLaserScanMessage(const std::string& sensor_id, const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg);

    void HandlePointCloud2Message(const std::string& sensor_id, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
        
private:
    struct Subscriber 
    {
        rclcpp::SubscriptionBase::SharedPtr subscriber;

        // ::ros::Subscriber::getTopic() does not necessarily return the same
        // std::string
        // it was given in its constructor. Since we rely on the topic name as the
        // unique identifier of a subscriber, we remember it ourselves.
        std::string topic;
    };


    void PublishTrajectoryList();
    void PublishEnvPointCloudData();

    std::unique_ptr<AutonomyBridge> autonomy_builder_{nullptr};

    // // ROS2 Node
    rclcpp::Node::SharedPtr node_{nullptr};
    std::vector<std::vector<Subscriber>> subscribers_;

    // timers
    ::rclcpp::TimerBase::SharedPtr trajectory_list_timer_{nullptr};
    ::rclcpp::TimerBase::SharedPtr env_point_cloud_data_timer_{nullptr};
};

}  // namespace autonomy_ros