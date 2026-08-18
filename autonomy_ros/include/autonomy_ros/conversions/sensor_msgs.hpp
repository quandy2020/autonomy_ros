/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 * email: quandy2020@126.com
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

// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__SENSOR_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__SENSOR_MSGS_HPP_

/// @file sensor_msgs.hpp
/// @brief Converts ROS 2 sensor_msgs and automsgs::msgs::sensor_msgs.
///
/// Proto schema: automsgs/msgs/sensor_msgs/
///
/// @par Overview
/// Common in simulation and hardware drivers: LaserScan, Imu, Image, PointCloud2,
/// Joy. CameraInfo has minor field-name differences vs automsgs; see sensor_msgs.cpp.
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) in sensor bridges:
/// - fromRos() on subscription callbacks.
/// - toRos() before publishing driver or synthetic sensor data.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/sensor_msgs.hpp"
/// auto scan = autonomy_ros::fromRos(*laser_msg);
/// cloud_pub_->publish(autonomy_ros::toRos(core_cloud));
/// @endcode

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/channel_float32.pb.h>
#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/illuminance.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/joy.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>
#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include <automsgs/msgs/sensor_msgs/region_of_interest.pb.h>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

namespace autonomy_ros
{

using RegionOfInterest = ::automsgs::msgs::sensor_msgs::RegionOfInterest;
using CameraInfo = ::automsgs::msgs::sensor_msgs::CameraInfo;
using ChannelFloat32 = ::automsgs::msgs::sensor_msgs::ChannelFloat32;
using CompressedImage = ::automsgs::msgs::sensor_msgs::CompressedImage;
using Illuminance = ::automsgs::msgs::sensor_msgs::Illuminance;
using Image = ::automsgs::msgs::sensor_msgs::Image;
using Imu = ::automsgs::msgs::sensor_msgs::Imu;
using LaserScan = ::automsgs::msgs::sensor_msgs::LaserScan;
using PointCloud = ::automsgs::msgs::sensor_msgs::PointCloud;
using PointField = ::automsgs::msgs::sensor_msgs::PointField;
using PointCloud2 = ::automsgs::msgs::sensor_msgs::PointCloud2;
using Range = ::automsgs::msgs::sensor_msgs::Range;
using Joy = ::automsgs::msgs::sensor_msgs::Joy;

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::RegionOfInterest and automsgs RegionOfInterest.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::RegionOfInterest). Fields are copied without coordinate transforms.
 * @return automsgs RegionOfInterest for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs RegionOfInterest from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::RegionOfInterest ready for rclcpp publish() or subscribe() adapters.
 */
RegionOfInterest fromRos(const sensor_msgs::msg::RegionOfInterest & from);
sensor_msgs::msg::RegionOfInterest toRos(const RegionOfInterest & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::CameraInfo and automsgs CameraInfo.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::CameraInfo). Fields are copied without coordinate transforms.
 * @return automsgs CameraInfo for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs CameraInfo from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::CameraInfo ready for rclcpp publish() or subscribe() adapters.
 */
CameraInfo fromRos(const sensor_msgs::msg::CameraInfo & from);
sensor_msgs::msg::CameraInfo toRos(const CameraInfo & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::ChannelFloat32 and automsgs ChannelFloat32.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::ChannelFloat32). Fields are copied without coordinate transforms.
 * @return automsgs ChannelFloat32 for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs ChannelFloat32 from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::ChannelFloat32 ready for rclcpp publish() or subscribe() adapters.
 */
ChannelFloat32 fromRos(const sensor_msgs::msg::ChannelFloat32 & from);
sensor_msgs::msg::ChannelFloat32 toRos(const ChannelFloat32 & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::CompressedImage and automsgs CompressedImage.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::CompressedImage). Fields are copied without coordinate transforms.
 * @return automsgs CompressedImage for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs CompressedImage from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::CompressedImage ready for rclcpp publish() or subscribe() adapters.
 */
CompressedImage fromRos(const sensor_msgs::msg::CompressedImage & from);
sensor_msgs::msg::CompressedImage toRos(const CompressedImage & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Illuminance and automsgs Illuminance.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Illuminance). Fields are copied without coordinate transforms.
 * @return automsgs Illuminance for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Illuminance from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Illuminance ready for rclcpp publish() or subscribe() adapters.
 */
Illuminance fromRos(const sensor_msgs::msg::Illuminance & from);
sensor_msgs::msg::Illuminance toRos(const Illuminance & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Image and automsgs Image.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Image). Fields are copied without coordinate transforms.
 * @return automsgs Image for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Image from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Image ready for rclcpp publish() or subscribe() adapters.
 */
Image fromRos(const sensor_msgs::msg::Image & from);
sensor_msgs::msg::Image toRos(const Image & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Imu and automsgs Imu.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Imu). Fields are copied without coordinate transforms.
 * @return automsgs Imu for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Imu from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Imu ready for rclcpp publish() or subscribe() adapters.
 */
Imu fromRos(const sensor_msgs::msg::Imu & from);
sensor_msgs::msg::Imu toRos(const Imu & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::LaserScan and automsgs LaserScan.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::LaserScan). Fields are copied without coordinate transforms.
 * @return automsgs LaserScan for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs LaserScan from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::LaserScan ready for rclcpp publish() or subscribe() adapters.
 */
LaserScan fromRos(const sensor_msgs::msg::LaserScan & from);
sensor_msgs::msg::LaserScan toRos(const LaserScan & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::PointCloud and automsgs PointCloud.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::PointCloud). Fields are copied without coordinate transforms.
 * @return automsgs PointCloud for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PointCloud from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::PointCloud ready for rclcpp publish() or subscribe() adapters.
 */
PointCloud fromRos(const sensor_msgs::msg::PointCloud & from);
sensor_msgs::msg::PointCloud toRos(const PointCloud & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::PointField and automsgs PointField.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::PointField). Fields are copied without coordinate transforms.
 * @return automsgs PointField for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PointField from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::PointField ready for rclcpp publish() or subscribe() adapters.
 */
PointField fromRos(const sensor_msgs::msg::PointField & from);
sensor_msgs::msg::PointField toRos(const PointField & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::PointCloud2 and automsgs PointCloud2.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::PointCloud2). Fields are copied without coordinate transforms.
 * @return automsgs PointCloud2 for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PointCloud2 from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::PointCloud2 ready for rclcpp publish() or subscribe() adapters.
 */
PointCloud2 fromRos(const sensor_msgs::msg::PointCloud2 & from);
sensor_msgs::msg::PointCloud2 toRos(const PointCloud2 & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Range and automsgs Range.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Range). Fields are copied without coordinate transforms.
 * @return automsgs Range for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Range from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Range ready for rclcpp publish() or subscribe() adapters.
 */
Range fromRos(const sensor_msgs::msg::Range & from);
sensor_msgs::msg::Range toRos(const Range & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Joy and automsgs Joy.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Joy). Fields are copied without coordinate transforms.
 * @return automsgs Joy for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Joy from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Joy ready for rclcpp publish() or subscribe() adapters.
 */
Joy fromRos(const sensor_msgs::msg::Joy & from);
sensor_msgs::msg::Joy toRos(const Joy & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__SENSOR_MSGS_HPP_
