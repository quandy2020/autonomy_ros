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
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/fixed_ratio_sampler.hpp"
#include "autonomy/sensor/data.hpp"
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
        std::unique_ptr<AutonomyBridge> autonomy,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        rclcpp::Node::SharedPtr node,
        bool collect_metrics);
    

    /**
     * @brief Destroy the Node object
     * 
     */
    ~Node() = default;

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    // Returns the set of SensorIds expected for a autonomy.
    // 'SensorId::id' is the expected ROS topic name.
    std::set<::autonomy::sensor::SensorId> ComputeExpectedSensorIds(const NodeOptions& options) const;

    /**
     * @brief Starts with the default topics.
     * 
     * @param options 
     */
    void StartupWithDefaultTopics();

    /**
     * @brief The following functions handle adding sensor data(odom).
     * 
     * @param sensor_id 
     * @param msg 
     */
    void HandleOdometryMessage(const std::string& sensor_id, 
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

    /**
     * @brief The following functions handle adding sensor data(NavSatFix).
     * 
     * @param sensor_id 
     * @param msg 
     */
    void HandleNavSatFixMessage(const std::string& sensor_id, 
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);

    /**
     * @brief The following functions handle adding sensor data(Imu).
     * 
     * @param sensor_id 
     * @param msg 
     */
    void HandleImuMessage(const std::string& sensor_id, 
        const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

     /**
     * @brief The following functions handle adding sensor data(LaserScan).
     * 
     * @param sensor_id 
     * @param msg 
     */
    void HandleLaserScanMessage(const std::string& sensor_id, 
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

    /**
     * @brief The following functions handle adding sensor data.
     * 
     * @param sensor_id 
     * @param msg 
     */
    void HandleMultiEchoLaserScanMessage(const std::string& sensor_id, 
        const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg);

    /**
     * @brief The following functions handle adding sensor data(PointCloud2).
     * 
     * @param sensor_id 
     * @param msg 
     */
    void HandlePointCloud2Message(const std::string& sensor_id, 
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    /**
     * @brief Warn topics mismatch
     * 
     */
    void MaybeWarnAboutTopicMismatch();
        
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

    /**
     * @brief Publish OccupancyGrid
     * 
     */
    void PublishOccupancyGridMap2D();

    /**
     * @brief Publish global path
     * 
     */
    void PublishGlobalTrajectory();

    /**
     * @brief Publish local path data
     * 
     */
    void PublishLocalTrajectory();

    /**
     * @brief Publish PointCloud2 data
     * 
     */
    void PublishEnvPointCloudData();

    /**
     * @brief Add sensor ros options
     * 
     * @param options 
     */
    void AddSensorSamplers(const NodeOptions& options);

    // AutonomyBridge
    std::unique_ptr<AutonomyBridge> autonomy_builder_{nullptr};

    // ROS2 Node
    ::rclcpp::Node::SharedPtr node_{nullptr};

    // visualization for map 2d
    ::rclcpp::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_ {nullptr};

    // visualization for global_trajectory
    ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr global_trajectory_publisher_{nullptr};

    // visualization for local_trajectory
    ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr local_trajectory_publisher_{nullptr};

    // visualization for global env 3D 
    ::rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr env_point_cloud_publisher_;

    struct AutonomySensorSamplers 
    {
        AutonomySensorSamplers(const double rangefinder_sampling_ratio,
                                 const double odometry_sampling_ratio,
                                 const double fixed_frame_pose_sampling_ratio,
                                 const double imu_sampling_ratio,
                                 const double landmark_sampling_ratio)
            : rangefinder_sampler(rangefinder_sampling_ratio),
              odometry_sampler(odometry_sampling_ratio),
              fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
              imu_sampler(imu_sampling_ratio),
              landmark_sampler(landmark_sampling_ratio) {}
    
        ::autonomy::common::FixedRatioSampler rangefinder_sampler;
        ::autonomy::common::FixedRatioSampler odometry_sampler;
        ::autonomy::common::FixedRatioSampler fixed_frame_pose_sampler;
        ::autonomy::common::FixedRatioSampler imu_sampler;
        ::autonomy::common::FixedRatioSampler landmark_sampler;
    };

    const NodeOptions node_options_;
    
    // Topics subscribers
    std::vector<Subscriber> subscribers_;
    std::unordered_set<std::string> subscribed_topics_;
    // AutonomySensorSamplers sensor_samplers_;

    // timers
    ::rclcpp::TimerBase::SharedPtr point_cloud_timer_{nullptr};
    ::rclcpp::TimerBase::SharedPtr occupancy_grid_timer_{nullptr};
    ::rclcpp::TimerBase::SharedPtr global_trajectory_timer_{nullptr};
    ::rclcpp::TimerBase::SharedPtr local_trajectory_timer_{nullptr};
    ::rclcpp::TimerBase::SharedPtr maybe_warn_about_topic_mismatch_timer_{nullptr};
};

}  // namespace autonomy_ros