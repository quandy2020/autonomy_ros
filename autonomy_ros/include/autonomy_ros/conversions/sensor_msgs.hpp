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
/// @brief Converts ROS 2 sensor_msgs and autonomy::commsgs::sensor_msgs.
///
/// Proto schema: autonomy/commsgs/proto/sensor_msgs.proto
///
/// @par Overview
/// Common in simulation and hardware drivers: LaserScan, Imu, Image, PointCloud2,
/// Joy. CameraInfo has minor field-name differences vs commsgs; see sensor_msgs.cpp.
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

#include "autonomy/commsgs/sensor_msgs.hpp"
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

using RegionOfInterest = ::autonomy::commsgs::sensor_msgs::RegionOfInterest;
using CameraInfo = ::autonomy::commsgs::sensor_msgs::CameraInfo;
using ChannelFloat32 = ::autonomy::commsgs::sensor_msgs::ChannelFloat32;
using CompressedImage = ::autonomy::commsgs::sensor_msgs::CompressedImage;
using Illuminance = ::autonomy::commsgs::sensor_msgs::Illuminance;
using Image = ::autonomy::commsgs::sensor_msgs::Image;
using Imu = ::autonomy::commsgs::sensor_msgs::Imu;
using LaserScan = ::autonomy::commsgs::sensor_msgs::LaserScan;
using PointCloud = ::autonomy::commsgs::sensor_msgs::PointCloud;
using PointField = ::autonomy::commsgs::sensor_msgs::PointField;
using PointCloud2 = ::autonomy::commsgs::sensor_msgs::PointCloud2;
using Range = ::autonomy::commsgs::sensor_msgs::Range;
using Joy = ::autonomy::commsgs::sensor_msgs::Joy;

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::RegionOfInterest and commsgs RegionOfInterest.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::RegionOfInterest). Fields are copied without coordinate transforms.
 * @return commsgs RegionOfInterest for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs RegionOfInterest from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::RegionOfInterest ready for rclcpp publish() or subscribe() adapters.
 */
RegionOfInterest fromRos(const sensor_msgs::msg::RegionOfInterest & from);
sensor_msgs::msg::RegionOfInterest toRos(const RegionOfInterest & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::CameraInfo and commsgs CameraInfo.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::CameraInfo). Fields are copied without coordinate transforms.
 * @return commsgs CameraInfo for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs CameraInfo from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::CameraInfo ready for rclcpp publish() or subscribe() adapters.
 */
CameraInfo fromRos(const sensor_msgs::msg::CameraInfo & from);
sensor_msgs::msg::CameraInfo toRos(const CameraInfo & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::ChannelFloat32 and commsgs ChannelFloat32.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::ChannelFloat32). Fields are copied without coordinate transforms.
 * @return commsgs ChannelFloat32 for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs ChannelFloat32 from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::ChannelFloat32 ready for rclcpp publish() or subscribe() adapters.
 */
ChannelFloat32 fromRos(const sensor_msgs::msg::ChannelFloat32 & from);
sensor_msgs::msg::ChannelFloat32 toRos(const ChannelFloat32 & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::CompressedImage and commsgs CompressedImage.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::CompressedImage). Fields are copied without coordinate transforms.
 * @return commsgs CompressedImage for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs CompressedImage from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::CompressedImage ready for rclcpp publish() or subscribe() adapters.
 */
CompressedImage fromRos(const sensor_msgs::msg::CompressedImage & from);
sensor_msgs::msg::CompressedImage toRos(const CompressedImage & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Illuminance and commsgs Illuminance.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Illuminance). Fields are copied without coordinate transforms.
 * @return commsgs Illuminance for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Illuminance from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Illuminance ready for rclcpp publish() or subscribe() adapters.
 */
Illuminance fromRos(const sensor_msgs::msg::Illuminance & from);
sensor_msgs::msg::Illuminance toRos(const Illuminance & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Image and commsgs Image.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Image). Fields are copied without coordinate transforms.
 * @return commsgs Image for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Image from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Image ready for rclcpp publish() or subscribe() adapters.
 */
Image fromRos(const sensor_msgs::msg::Image & from);
sensor_msgs::msg::Image toRos(const Image & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Imu and commsgs Imu.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Imu). Fields are copied without coordinate transforms.
 * @return commsgs Imu for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Imu from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Imu ready for rclcpp publish() or subscribe() adapters.
 */
Imu fromRos(const sensor_msgs::msg::Imu & from);
sensor_msgs::msg::Imu toRos(const Imu & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::LaserScan and commsgs LaserScan.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::LaserScan). Fields are copied without coordinate transforms.
 * @return commsgs LaserScan for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs LaserScan from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::LaserScan ready for rclcpp publish() or subscribe() adapters.
 */
LaserScan fromRos(const sensor_msgs::msg::LaserScan & from);
sensor_msgs::msg::LaserScan toRos(const LaserScan & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::PointCloud and commsgs PointCloud.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::PointCloud). Fields are copied without coordinate transforms.
 * @return commsgs PointCloud for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PointCloud from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::PointCloud ready for rclcpp publish() or subscribe() adapters.
 */
PointCloud fromRos(const sensor_msgs::msg::PointCloud & from);
sensor_msgs::msg::PointCloud toRos(const PointCloud & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::PointField and commsgs PointField.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::PointField). Fields are copied without coordinate transforms.
 * @return commsgs PointField for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PointField from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::PointField ready for rclcpp publish() or subscribe() adapters.
 */
PointField fromRos(const sensor_msgs::msg::PointField & from);
sensor_msgs::msg::PointField toRos(const PointField & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::PointCloud2 and commsgs PointCloud2.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::PointCloud2). Fields are copied without coordinate transforms.
 * @return commsgs PointCloud2 for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PointCloud2 from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::PointCloud2 ready for rclcpp publish() or subscribe() adapters.
 */
PointCloud2 fromRos(const sensor_msgs::msg::PointCloud2 & from);
sensor_msgs::msg::PointCloud2 toRos(const PointCloud2 & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Range and commsgs Range.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Range). Fields are copied without coordinate transforms.
 * @return commsgs Range for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Range from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Range ready for rclcpp publish() or subscribe() adapters.
 */
Range fromRos(const sensor_msgs::msg::Range & from);
sensor_msgs::msg::Range toRos(const Range & from);

/**
 * @brief Bidirectional conversion between ROS sensor_msgs::msg::Joy and commsgs Joy.
 *
 * @par fromRos
 * @param from Input ROS message (sensor_msgs::msg::Joy). Fields are copied without coordinate transforms.
 * @return commsgs Joy for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Joy from planners, bridges, or drivers.
 * @return ROS sensor_msgs::msg::Joy ready for rclcpp publish() or subscribe() adapters.
 */
Joy fromRos(const sensor_msgs::msg::Joy & from);
sensor_msgs::msg::Joy toRos(const Joy & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__SENSOR_MSGS_HPP_
