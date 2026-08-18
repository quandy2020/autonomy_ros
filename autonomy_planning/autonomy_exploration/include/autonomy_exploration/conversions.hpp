/*
 * Copyright 2026 The Openbot Authors
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

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace autonomy_exploration {
namespace convert {

void CopyHeader(const std_msgs::msg::Header & from,
                automsgs::msgs::std_msgs::Header * to);
void CopyHeader(const automsgs::msgs::std_msgs::Header & from,
                std_msgs::msg::Header * to);

automsgs::msgs::nav_msgs::Odometry FromRos(
    const nav_msgs::msg::Odometry & msg);
automsgs::msgs::sensor_msgs::Image FromRos(
    const sensor_msgs::msg::Image & msg);
automsgs::msgs::sensor_msgs::CameraInfo FromRos(
    const sensor_msgs::msg::CameraInfo & msg);
automsgs::msgs::geometry_msgs::TransformStamped FromRos(
    const geometry_msgs::msg::TransformStamped & msg);

geometry_msgs::msg::PoseStamped ToRos(
    const automsgs::msgs::geometry_msgs::PoseStamped & msg);
nav_msgs::msg::Path ToRos(const automsgs::msgs::nav_msgs::Path & msg);
nav_msgs::msg::OccupancyGrid ToRos(
    const automsgs::msgs::map_msgs::OccupancyGrid & msg);

}  // namespace convert
}  // namespace autonomy_exploration
