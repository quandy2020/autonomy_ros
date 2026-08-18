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

#include "autonomy_exploration/conversions.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_exploration {
namespace convert {

void CopyHeader(const std_msgs::msg::Header & from,
                automsgs::msgs::std_msgs::Header * to)
{
  autonomy_ros::copyHeader(from, *to);
}

void CopyHeader(const automsgs::msgs::std_msgs::Header & from,
                std_msgs::msg::Header * to)
{
  autonomy_ros::copyHeader(from, *to);
}

automsgs::msgs::nav_msgs::Odometry FromRos(
    const nav_msgs::msg::Odometry & msg)
{
  return autonomy_ros::fromRos(msg);
}

automsgs::msgs::sensor_msgs::Image FromRos(
    const sensor_msgs::msg::Image & msg)
{
  return autonomy_ros::fromRos(msg);
}

automsgs::msgs::sensor_msgs::CameraInfo FromRos(
    const sensor_msgs::msg::CameraInfo & msg)
{
  return autonomy_ros::fromRos(msg);
}

automsgs::msgs::geometry_msgs::TransformStamped FromRos(
    const geometry_msgs::msg::TransformStamped & msg)
{
  return autonomy_ros::fromRos(msg);
}

geometry_msgs::msg::PoseStamped ToRos(
    const automsgs::msgs::geometry_msgs::PoseStamped & msg)
{
  return autonomy_ros::toRos(msg);
}

nav_msgs::msg::Path ToRos(const automsgs::msgs::nav_msgs::Path & msg)
{
  return autonomy_ros::toRos(msg);
}

nav_msgs::msg::OccupancyGrid ToRos(
    const automsgs::msgs::map_msgs::OccupancyGrid & msg)
{
  return autonomy_ros::toRos(msg);
}

}  // namespace convert
}  // namespace autonomy_exploration
