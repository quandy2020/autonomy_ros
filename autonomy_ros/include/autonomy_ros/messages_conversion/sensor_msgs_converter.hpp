/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include <string>
#include <tuple>

// sensor_msgs
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

#include "autonomy/commsgs/sensor_msgs.hpp"

namespace autonomy_ros {

// ChannelFloat32
sensor_msgs::msg::ChannelFloat32 ToRos(const ::autonomy::commsgs::sensor_msgs::ChannelFloat32& data);
::autonomy::commsgs::sensor_msgs::ChannelFloat32 FromRos(const sensor_msgs::msg::ChannelFloat32& ros);

// PointField
sensor_msgs::msg::PointField ToRos(const ::autonomy::commsgs::sensor_msgs::PointField& data);
::autonomy::commsgs::sensor_msgs::PointField FromRos(const sensor_msgs::msg::PointField& ros);

// PointCloud
sensor_msgs::msg::PointCloud ToRos(const ::autonomy::commsgs::sensor_msgs::PointCloud& data);
::autonomy::commsgs::sensor_msgs::PointCloud FromRos(const sensor_msgs::msg::PointCloud& ros);

// PointCloud2
sensor_msgs::msg::PointCloud2 ToRos(const ::autonomy::commsgs::sensor_msgs::PointCloud2& data);
::autonomy::commsgs::sensor_msgs::PointCloud2 FromRos(const sensor_msgs::msg::PointCloud2& ros);

}  // namespace autonomy_ros
