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

#include <algorithm>

namespace autonomy_exploration {
namespace convert {
namespace {

void CopyPose(const geometry_msgs::msg::Pose & from,
              autonomy::commsgs::geometry_msgs::Pose * to)
{
  to->position.x = from.position.x;
  to->position.y = from.position.y;
  to->position.z = from.position.z;
  to->orientation.x = from.orientation.x;
  to->orientation.y = from.orientation.y;
  to->orientation.z = from.orientation.z;
  to->orientation.w = from.orientation.w;
}

void CopyPose(const autonomy::commsgs::geometry_msgs::Pose & from,
              geometry_msgs::msg::Pose * to)
{
  to->position.x = from.position.x;
  to->position.y = from.position.y;
  to->position.z = from.position.z;
  to->orientation.x = from.orientation.x;
  to->orientation.y = from.orientation.y;
  to->orientation.z = from.orientation.z;
  to->orientation.w = from.orientation.w;
}

}  // namespace

void CopyHeader(const std_msgs::msg::Header & from,
                autonomy::commsgs::std_msgs::Header * to)
{
  to->stamp.sec = from.stamp.sec;
  to->stamp.nanosec = from.stamp.nanosec;
  to->frame_id = from.frame_id;
}

void CopyHeader(const autonomy::commsgs::std_msgs::Header & from,
                std_msgs::msg::Header * to)
{
  to->stamp.sec = from.stamp.sec;
  to->stamp.nanosec = from.stamp.nanosec;
  to->frame_id = from.frame_id;
}

autonomy::commsgs::planning_msgs::Odometry FromRos(
    const nav_msgs::msg::Odometry & msg)
{
  autonomy::commsgs::planning_msgs::Odometry out;
  CopyHeader(msg.header, &out.header);
  out.child_frame_id = msg.child_frame_id;
  CopyPose(msg.pose.pose, &out.pose.pose);
  out.twist.twist.linear.x = static_cast<float>(msg.twist.twist.linear.x);
  out.twist.twist.linear.y = static_cast<float>(msg.twist.twist.linear.y);
  out.twist.twist.linear.z = static_cast<float>(msg.twist.twist.linear.z);
  out.twist.twist.angular.x = static_cast<float>(msg.twist.twist.angular.x);
  out.twist.twist.angular.y = static_cast<float>(msg.twist.twist.angular.y);
  out.twist.twist.angular.z = static_cast<float>(msg.twist.twist.angular.z);
  return out;
}

autonomy::commsgs::sensor_msgs::Image FromRos(
    const sensor_msgs::msg::Image & msg)
{
  autonomy::commsgs::sensor_msgs::Image out;
  CopyHeader(msg.header, &out.header);
  out.height = msg.height;
  out.width = msg.width;
  out.encoding = msg.encoding;
  out.is_bigendian = msg.is_bigendian;
  out.step = msg.step;
  out.data.assign(msg.data.begin(), msg.data.end());
  return out;
}

autonomy::commsgs::sensor_msgs::CameraInfo FromRos(
    const sensor_msgs::msg::CameraInfo & msg)
{
  autonomy::commsgs::sensor_msgs::CameraInfo out;
  CopyHeader(msg.header, &out.header);
  out.height = msg.height;
  out.width = msg.width;
  out.distortion_model = msg.distortion_model;
  out.d.assign(msg.d.begin(), msg.d.end());
  out.k.assign(msg.k.begin(), msg.k.end());
  out.r.assign(msg.r.begin(), msg.r.end());
  out.p.assign(msg.p.begin(), msg.p.end());
  out.binning_x0 = msg.binning_x;
  out.binning_y1 = msg.binning_y;
  out.roi2.x_offset = msg.roi.x_offset;
  out.roi2.y_offset = msg.roi.y_offset;
  out.roi2.height = msg.roi.height;
  out.roi2.width = msg.roi.width;
  out.roi2.do_rectify = msg.roi.do_rectify;
  return out;
}

autonomy::commsgs::geometry_msgs::TransformStamped FromRos(
    const geometry_msgs::msg::TransformStamped & msg)
{
  autonomy::commsgs::geometry_msgs::TransformStamped out;
  CopyHeader(msg.header, &out.header);
  out.child_frame_id = msg.child_frame_id;
  out.transform.translation.x = msg.transform.translation.x;
  out.transform.translation.y = msg.transform.translation.y;
  out.transform.translation.z = msg.transform.translation.z;
  out.transform.rotation.x = msg.transform.rotation.x;
  out.transform.rotation.y = msg.transform.rotation.y;
  out.transform.rotation.z = msg.transform.rotation.z;
  out.transform.rotation.w = msg.transform.rotation.w;
  return out;
}

geometry_msgs::msg::PoseStamped ToRos(
    const autonomy::commsgs::geometry_msgs::PoseStamped & msg)
{
  geometry_msgs::msg::PoseStamped out;
  CopyHeader(msg.header, &out.header);
  CopyPose(msg.pose, &out.pose);
  return out;
}

nav_msgs::msg::Path ToRos(const autonomy::commsgs::planning_msgs::Path & msg)
{
  nav_msgs::msg::Path out;
  CopyHeader(msg.header, &out.header);
  out.poses.reserve(msg.poses.size());
  for (const auto & p : msg.poses) {
    out.poses.push_back(ToRos(p));
  }
  return out;
}

nav_msgs::msg::OccupancyGrid ToRos(
    const autonomy::commsgs::map_msgs::OccupancyGrid & msg)
{
  nav_msgs::msg::OccupancyGrid out;
  CopyHeader(msg.header, &out.header);
  out.info.map_load_time.sec = msg.info.map_load_time.sec;
  out.info.map_load_time.nanosec = msg.info.map_load_time.nanosec;
  out.info.resolution = static_cast<float>(msg.info.resolution);
  out.info.width = msg.info.width;
  out.info.height = msg.info.height;
  out.info.origin.position.x = msg.info.origin.position.x;
  out.info.origin.position.y = msg.info.origin.position.y;
  out.info.origin.position.z = msg.info.origin.position.z;
  out.info.origin.orientation.x = msg.info.origin.orientation.x;
  out.info.origin.orientation.y = msg.info.origin.orientation.y;
  out.info.origin.orientation.z = msg.info.origin.orientation.z;
  out.info.origin.orientation.w = msg.info.origin.orientation.w;
  out.data.resize(msg.data.size());
  for (size_t i = 0; i < msg.data.size(); ++i) {
    const int v = static_cast<int>(msg.data[i]);
    out.data[i] = static_cast<int8_t>(std::clamp(v, -1, 100));
  }
  return out;
}

}  // namespace convert
}  // namespace autonomy_exploration
