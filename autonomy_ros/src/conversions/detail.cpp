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

#include "autonomy_ros/conversions/detail.hpp"

#include <algorithm>

namespace autonomy_ros
{

void copyTime(
  const builtin_interfaces::msg::Time & from,
  ::autonomy::commsgs::builtin_interfaces::Time & to)
{
  to.sec = from.sec;
  to.nanosec = from.nanosec;
}

void copyTime(
  const rclcpp::Time & from,
  ::autonomy::commsgs::builtin_interfaces::Time & to)
{
  const int64_t ns = from.nanoseconds();
  to.sec = static_cast<int32_t>(ns / 1000000000LL);
  to.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

builtin_interfaces::msg::Time toRosTime(
  const ::autonomy::commsgs::builtin_interfaces::Time & from)
{
  builtin_interfaces::msg::Time to;
  to.sec = from.sec;
  to.nanosec = from.nanosec;
  return to;
}

void copyHeader(
  const std_msgs::msg::Header & from,
  ::autonomy::commsgs::std_msgs::Header & to)
{
  copyTime(from.stamp, to.stamp);
  to.frame_id = from.frame_id;
}

void copyHeader(
  const ::autonomy::commsgs::std_msgs::Header & from,
  std_msgs::msg::Header & to)
{
  to.stamp = toRosTime(from.stamp);
  to.frame_id = from.frame_id;
}

void copyQuaternion(
  const geometry_msgs::msg::Quaternion & from,
  ::autonomy::commsgs::geometry_msgs::Quaternion & to)
{
  to.x = static_cast<float>(from.x);
  to.y = static_cast<float>(from.y);
  to.z = static_cast<float>(from.z);
  to.w = static_cast<float>(from.w);
}

void copyQuaternion(
  const ::autonomy::commsgs::geometry_msgs::Quaternion & from,
  geometry_msgs::msg::Quaternion & to)
{
  to.x = from.x;
  to.y = from.y;
  to.z = from.z;
  to.w = from.w;
}

void copyPoint(
  const geometry_msgs::msg::Point & from,
  ::autonomy::commsgs::geometry_msgs::Point & to)
{
  to.x = from.x;
  to.y = from.y;
  to.z = from.z;
}

void copyPoint(
  const ::autonomy::commsgs::geometry_msgs::Point & from,
  geometry_msgs::msg::Point & to)
{
  to.x = from.x;
  to.y = from.y;
  to.z = from.z;
}

void copyPose(
  const geometry_msgs::msg::Pose & from,
  ::autonomy::commsgs::geometry_msgs::Pose & to)
{
  copyPoint(from.position, to.position);
  copyQuaternion(from.orientation, to.orientation);
}

void copyPose(
  const ::autonomy::commsgs::geometry_msgs::Pose & from,
  geometry_msgs::msg::Pose & to)
{
  copyPoint(from.position, to.position);
  copyQuaternion(from.orientation, to.orientation);
}

void copyDuration(
  const builtin_interfaces::msg::Duration & from,
  ::autonomy::commsgs::builtin_interfaces::Duration & to)
{
  const int64_t ns =
    static_cast<int64_t>(from.sec) * 1000000000LL + static_cast<int64_t>(from.nanosec);
  to = ::autonomy::commsgs::builtin_interfaces::Duration::FromNanoseconds(ns);
}

void copyDuration(
  const ::autonomy::commsgs::builtin_interfaces::Duration & from,
  builtin_interfaces::msg::Duration & to)
{
  const int64_t ns = from.Nanoseconds();
  const int64_t sec = ns / 1000000000LL;
  int64_t nsec = ns % 1000000000LL;
  if (nsec < 0) {
    nsec += 1000000000LL;
  }
  to.sec = static_cast<int32_t>(sec);
  to.nanosec = static_cast<uint32_t>(nsec);
}

void copyVector3(
  const geometry_msgs::msg::Vector3 & from,
  ::autonomy::commsgs::geometry_msgs::Vector3 & to)
{
  to.x = static_cast<float>(from.x);
  to.y = static_cast<float>(from.y);
  to.z = static_cast<float>(from.z);
}

void copyVector3(
  const ::autonomy::commsgs::geometry_msgs::Vector3 & from,
  geometry_msgs::msg::Vector3 & to)
{
  to.x = from.x;
  to.y = from.y;
  to.z = from.z;
}

void copyTransform(
  const geometry_msgs::msg::Transform & from,
  ::autonomy::commsgs::geometry_msgs::Transform & to)
{
  copyVector3(from.translation, to.translation);
  copyQuaternion(from.rotation, to.rotation);
}

void copyTransform(
  const ::autonomy::commsgs::geometry_msgs::Transform & from,
  geometry_msgs::msg::Transform & to)
{
  copyVector3(from.translation, to.translation);
  copyQuaternion(from.rotation, to.rotation);
}

void copyColorRGBA(
  const std_msgs::msg::ColorRGBA & from,
  ::autonomy::commsgs::std_msgs::ColorRGBA & to)
{
  to.r = from.r;
  to.g = from.g;
  to.b = from.b;
  to.a = from.a;
}

void copyColorRGBA(
  const ::autonomy::commsgs::std_msgs::ColorRGBA & from,
  std_msgs::msg::ColorRGBA & to)
{
  to.r = from.r;
  to.g = from.g;
  to.b = from.b;
  to.a = from.a;
}

void copyPoint32(
  const geometry_msgs::msg::Point32 & from,
  ::autonomy::commsgs::geometry_msgs::Point32 & to)
{
  to.x = from.x;
  to.y = from.y;
  to.z = from.z;
}

void copyPoint32(
  const ::autonomy::commsgs::geometry_msgs::Point32 & from,
  geometry_msgs::msg::Point32 & to)
{
  to.x = from.x;
  to.y = from.y;
  to.z = from.z;
}

void copyPose2D(
  const geometry_msgs::msg::Pose2D & from,
  ::autonomy::commsgs::geometry_msgs::Pose2D & to)
{
  to.x = from.x;
  to.y = from.y;
  to.theta = from.theta;
}

void copyPose2D(
  const ::autonomy::commsgs::geometry_msgs::Pose2D & from,
  geometry_msgs::msg::Pose2D & to)
{
  to.x = from.x;
  to.y = from.y;
  to.theta = from.theta;
}

void copyTwist(
  const geometry_msgs::msg::Twist & from,
  ::autonomy::commsgs::geometry_msgs::Twist & to)
{
  copyVector3(from.linear, to.linear);
  copyVector3(from.angular, to.angular);
}

void copyTwist(
  const ::autonomy::commsgs::geometry_msgs::Twist & from,
  geometry_msgs::msg::Twist & to)
{
  copyVector3(from.linear, to.linear);
  copyVector3(from.angular, to.angular);
}

void copyAccel(
  const geometry_msgs::msg::Accel & from,
  ::autonomy::commsgs::geometry_msgs::Accel & to)
{
  copyVector3(from.linear, to.linear);
  copyVector3(from.angular, to.angular);
}

void copyAccel(
  const ::autonomy::commsgs::geometry_msgs::Accel & from,
  geometry_msgs::msg::Accel & to)
{
  copyVector3(from.linear, to.linear);
  copyVector3(from.angular, to.angular);
}

void copyWrench(
  const geometry_msgs::msg::Wrench & from,
  ::autonomy::commsgs::geometry_msgs::Wrench & to)
{
  copyVector3(from.force, to.force);
  copyVector3(from.torque, to.torque);
}

void copyWrench(
  const ::autonomy::commsgs::geometry_msgs::Wrench & from,
  geometry_msgs::msg::Wrench & to)
{
  copyVector3(from.force, to.force);
  copyVector3(from.torque, to.torque);
}

void copyInertia(
  const geometry_msgs::msg::Inertia & from,
  ::autonomy::commsgs::geometry_msgs::Inertia & to)
{
  to.m = static_cast<float>(from.m);
  copyVector3(from.com, to.com);
  to.ixx = static_cast<float>(from.ixx);
  to.ixy = static_cast<float>(from.ixy);
  to.ixz = static_cast<float>(from.ixz);
  to.iyy = static_cast<float>(from.iyy);
  to.iyz = static_cast<float>(from.iyz);
  to.izz = static_cast<float>(from.izz);
}

void copyInertia(
  const ::autonomy::commsgs::geometry_msgs::Inertia & from,
  geometry_msgs::msg::Inertia & to)
{
  to.m = from.m;
  copyVector3(from.com, to.com);
  to.ixx = from.ixx;
  to.ixy = from.ixy;
  to.ixz = from.ixz;
  to.iyy = from.iyy;
  to.iyz = from.iyz;
  to.izz = from.izz;
}

void copyPolygon(
  const geometry_msgs::msg::Polygon & from,
  ::autonomy::commsgs::geometry_msgs::Polygon & to)
{
  to.points.reserve(from.points.size());
  for (const auto & p : from.points) {
    ::autonomy::commsgs::geometry_msgs::Point32 pt;
    copyPoint32(p, pt);
    to.points.push_back(pt);
  }
}

void copyPolygon(
  const ::autonomy::commsgs::geometry_msgs::Polygon & from,
  geometry_msgs::msg::Polygon & to)
{
  to.points.reserve(from.points.size());
  for (const auto & p : from.points) {
    geometry_msgs::msg::Point32 pt;
    copyPoint32(p, pt);
    to.points.push_back(pt);
  }
}

void copyCovariance6(
  const std::array<double, 36> & from,
  std::vector<double> & to)
{
  to.assign(from.begin(), from.end());
}

void copyCovariance6(
  const std::vector<double> & from,
  std::array<double, 36> & to)
{
  if (from.size() >= to.size()) {
    std::copy_n(from.begin(), to.size(), to.begin());
  }
}

}  // namespace autonomy_ros
