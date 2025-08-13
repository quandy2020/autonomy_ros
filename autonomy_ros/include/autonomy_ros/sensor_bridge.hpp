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

#include <memory>


#include "autonomy/sensor/imu_data.hpp"
#include "autonomy/sensor/odometry_data.hpp"
#include "autonomy_ros/tf_bridge.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autonomy_ros {
 
// Converts ROS messages into SensorData in tracking frame for the Autonomy.
class SensorBridge 
{
public:
    // explicit SensorBridge(
    //     int num_subdivisions_per_laser_scan, const std::string& tracking_frame,
    //     double lookup_transform_timeout_sec, tf2_ros::Buffer* tf_buffer,
    //     ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    std::unique_ptr<::autonomy::sensor::OdometryData> ToOdometryData(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    std::unique_ptr<::autonomy::sensor::ImuData> ToImuData(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

    void HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void HandleNavSatFixMessage(const std::string& sensor_id, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
    void HandleImuMessage(const std::string& sensor_id, const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void HandleLaserScanMessage(const std::string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void HandleMultiEchoLaserScanMessage(const std::string& sensor_id, const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg);
    void HandlePointCloud2Message(const std::string& sensor_id, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    const TfBridge& tf_bridge() const;

private:
    // void HandleLaserScan(const std::string& sensor_id, 
    //     ::autonomy::common::Time start_time, const std::string& frame_id,
    //     const ::autonomy::sensor::PointCloudWithIntensities& points);

    // void HandleRangefinder(const std::string& sensor_id,
    //                     ::autonomy::common::Time time,
    //                     const std::string& frame_id,
    //                     const ::autonomy::sensor::TimedPointCloud& ranges);

     const int num_subdivisions_per_laser_scan_;
    //  std::map<std::string, ::autonomy::common::Time> sensor_to_previous_subdivision_time_;

     const TfBridge tf_bridge_;
};

}  // namespace autonomy_ros