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

#include "autonomy_ros/conversions/detail.hpp"

#include <algorithm>

#include <automsgs/msgs/time_utils.hpp>

namespace autonomy_ros
{

namespace bi = automsgs::msgs::builtin_interfaces;
namespace geo = automsgs::msgs::geometry_msgs;
namespace stdm = automsgs::msgs::std_msgs;

void copyTime(const builtin_interfaces::msg::Time & from, bi::Time & to)
{
  to.set_sec(from.sec);
  to.set_nanosec(from.nanosec);
}

void copyTime(const rclcpp::Time & from, bi::Time & to)
{
  const int64_t ns = from.nanoseconds();
  to.set_sec(static_cast<int32_t>(ns / 1'000'000'000LL));
  to.set_nanosec(static_cast<uint32_t>(ns % 1'000'000'000LL));
}

builtin_interfaces::msg::Time toRosTime(const bi::Time & from)
{
  builtin_interfaces::msg::Time to;
  to.sec = from.sec();
  to.nanosec = from.nanosec();
  return to;
}

void copyHeader(const std_msgs::msg::Header & from, stdm::Header & to)
{
  copyTime(from.stamp, *to.mutable_stamp());
  to.set_frame_id(from.frame_id);
}

void copyHeader(const stdm::Header & from, std_msgs::msg::Header & to)
{
  to.stamp = toRosTime(from.stamp());
  to.frame_id = from.frame_id();
}

void copyQuaternion(
  const geometry_msgs::msg::Quaternion & from, geo::Quaternion & to)
{
  to.set_x(static_cast<float>(from.x));
  to.set_y(static_cast<float>(from.y));
  to.set_z(static_cast<float>(from.z));
  to.set_w(static_cast<float>(from.w));
}

void copyQuaternion(
  const geo::Quaternion & from, geometry_msgs::msg::Quaternion & to)
{
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
  to.w = from.w();
}

void copyPoint(const geometry_msgs::msg::Point & from, geo::Point & to)
{
  to.set_x(from.x);
  to.set_y(from.y);
  to.set_z(from.z);
}

void copyPoint(const geo::Point & from, geometry_msgs::msg::Point & to)
{
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
}

void copyPose(const geometry_msgs::msg::Pose & from, geo::Pose & to)
{
  copyPoint(from.position, *to.mutable_position());
  copyQuaternion(from.orientation, *to.mutable_orientation());
}

void copyPose(const geo::Pose & from, geometry_msgs::msg::Pose & to)
{
  copyPoint(from.position(), to.position);
  copyQuaternion(from.orientation(), to.orientation);
}

void copyDuration(const builtin_interfaces::msg::Duration & from, bi::Duration & to)
{
  const int64_t ns =
    static_cast<int64_t>(from.sec) * 1'000'000'000LL +
    static_cast<int64_t>(from.nanosec);
  to = bi::DurationFromNanoseconds(ns);
}

void copyDuration(const bi::Duration & from, builtin_interfaces::msg::Duration & to)
{
  const int64_t ns = static_cast<int64_t>(from.sec()) * 1'000'000'000LL +
                     static_cast<int64_t>(from.nanosec());
  const int64_t sec = ns / 1'000'000'000LL;
  int64_t nsec = ns % 1'000'000'000LL;
  if (nsec < 0) {
    nsec += 1'000'000'000LL;
  }
  to.sec = static_cast<int32_t>(sec);
  to.nanosec = static_cast<uint32_t>(nsec);
}

void copyVector3(const geometry_msgs::msg::Vector3 & from, geo::Vector3 & to)
{
  to.set_x(static_cast<float>(from.x));
  to.set_y(static_cast<float>(from.y));
  to.set_z(static_cast<float>(from.z));
}

void copyVector3(const geo::Vector3 & from, geometry_msgs::msg::Vector3 & to)
{
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
}

void copyTransform(
  const geometry_msgs::msg::Transform & from, geo::Transform & to)
{
  copyVector3(from.translation, *to.mutable_translation());
  copyQuaternion(from.rotation, *to.mutable_rotation());
}

void copyTransform(
  const geo::Transform & from, geometry_msgs::msg::Transform & to)
{
  copyVector3(from.translation(), to.translation);
  copyQuaternion(from.rotation(), to.rotation);
}

void copyColorRGBA(const std_msgs::msg::ColorRGBA & from, stdm::ColorRGBA & to)
{
  to.set_r(from.r);
  to.set_g(from.g);
  to.set_b(from.b);
  to.set_a(from.a);
}

void copyColorRGBA(const stdm::ColorRGBA & from, std_msgs::msg::ColorRGBA & to)
{
  to.r = from.r();
  to.g = from.g();
  to.b = from.b();
  to.a = from.a();
}

void copyPoint32(const geometry_msgs::msg::Point32 & from, geo::Point32 & to)
{
  to.set_x(from.x);
  to.set_y(from.y);
  to.set_z(from.z);
}

void copyPoint32(const geo::Point32 & from, geometry_msgs::msg::Point32 & to)
{
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
}

void copyPose2D(const geometry_msgs::msg::Pose2D & from, geo::Pose2D & to)
{
  to.set_x(from.x);
  to.set_y(from.y);
  to.set_theta(from.theta);
}

void copyPose2D(const geo::Pose2D & from, geometry_msgs::msg::Pose2D & to)
{
  to.x = from.x();
  to.y = from.y();
  to.theta = from.theta();
}

void copyTwist(const geometry_msgs::msg::Twist & from, geo::Twist & to)
{
  copyVector3(from.linear, *to.mutable_linear());
  copyVector3(from.angular, *to.mutable_angular());
}

void copyTwist(const geo::Twist & from, geometry_msgs::msg::Twist & to)
{
  copyVector3(from.linear(), to.linear);
  copyVector3(from.angular(), to.angular);
}

void copyAccel(const geometry_msgs::msg::Accel & from, geo::Accel & to)
{
  copyVector3(from.linear, *to.mutable_linear());
  copyVector3(from.angular, *to.mutable_angular());
}

void copyAccel(const geo::Accel & from, geometry_msgs::msg::Accel & to)
{
  copyVector3(from.linear(), to.linear);
  copyVector3(from.angular(), to.angular);
}

void copyWrench(const geometry_msgs::msg::Wrench & from, geo::Wrench & to)
{
  copyVector3(from.force, *to.mutable_force());
  copyVector3(from.torque, *to.mutable_torque());
}

void copyWrench(const geo::Wrench & from, geometry_msgs::msg::Wrench & to)
{
  copyVector3(from.force(), to.force);
  copyVector3(from.torque(), to.torque);
}

void copyInertia(const geometry_msgs::msg::Inertia & from, geo::Inertia & to)
{
  to.set_m(static_cast<float>(from.m));
  copyVector3(from.com, *to.mutable_com());
  to.set_ixx(static_cast<float>(from.ixx));
  to.set_ixy(static_cast<float>(from.ixy));
  to.set_ixz(static_cast<float>(from.ixz));
  to.set_iyy(static_cast<float>(from.iyy));
  to.set_iyz(static_cast<float>(from.iyz));
  to.set_izz(static_cast<float>(from.izz));
}

void copyInertia(const geo::Inertia & from, geometry_msgs::msg::Inertia & to)
{
  to.m = from.m();
  copyVector3(from.com(), to.com);
  to.ixx = from.ixx();
  to.ixy = from.ixy();
  to.ixz = from.ixz();
  to.iyy = from.iyy();
  to.iyz = from.iyz();
  to.izz = from.izz();
}

void copyPolygon(const geometry_msgs::msg::Polygon & from, geo::Polygon & to)
{
  to.clear_points();
  for (const auto & p : from.points) {
    copyPoint32(p, *to.add_points());
  }
}

void copyPolygon(const geo::Polygon & from, geometry_msgs::msg::Polygon & to)
{
  to.points.reserve(static_cast<size_t>(from.points_size()));
  for (const auto & p : from.points()) {
    geometry_msgs::msg::Point32 pt;
    copyPoint32(p, pt);
    to.points.push_back(pt);
  }
}

void copyCovariance6(
  const std::array<double, 36> & from,
  google::protobuf::RepeatedField<double> * to)
{
  to->Clear();
  to->Reserve(static_cast<int>(from.size()));
  for (double value : from) {
    to->Add(value);
  }
}

void copyCovariance6(
  const google::protobuf::RepeatedField<double> & from,
  std::array<double, 36> & to)
{
  if (from.size() >= static_cast<int>(to.size())) {
    std::copy_n(from.begin(), static_cast<std::ptrdiff_t>(to.size()), to.begin());
  }
}

}  // namespace autonomy_ros
