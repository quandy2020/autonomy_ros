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

#include "autonomy_ros/conversions/geometry_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros
{

Vector3 fromRos(const geometry_msgs::msg::Vector3 & from)
{
  Vector3 to;
  copyVector3(from, to);
  return to;
}

geometry_msgs::msg::Vector3 toRos(const Vector3 & from)
{
  geometry_msgs::msg::Vector3 to;
  copyVector3(from, to);
  return to;
}

Point fromRos(const geometry_msgs::msg::Point & from)
{
  Point to;
  copyPoint(from, to);
  return to;
}

geometry_msgs::msg::Point toRos(const Point & from)
{
  geometry_msgs::msg::Point to;
  copyPoint(from, to);
  return to;
}

Point32 fromRos(const geometry_msgs::msg::Point32 & from)
{
  Point32 to;
  copyPoint32(from, to);
  return to;
}

geometry_msgs::msg::Point32 toRos(const Point32 & from)
{
  geometry_msgs::msg::Point32 to;
  copyPoint32(from, to);
  return to;
}

Quaternion fromRos(const geometry_msgs::msg::Quaternion & from)
{
  Quaternion to;
  copyQuaternion(from, to);
  return to;
}

geometry_msgs::msg::Quaternion toRos(const Quaternion & from)
{
  geometry_msgs::msg::Quaternion to;
  copyQuaternion(from, to);
  return to;
}

Pose fromRos(const geometry_msgs::msg::Pose & from)
{
  Pose to;
  copyPose(from, to);
  return to;
}

geometry_msgs::msg::Pose toRos(const Pose & from)
{
  geometry_msgs::msg::Pose to;
  copyPose(from, to);
  return to;
}

Pose2D fromRos(const geometry_msgs::msg::Pose2D & from)
{
  Pose2D to;
  copyPose2D(from, to);
  return to;
}

geometry_msgs::msg::Pose2D toRos(const Pose2D & from)
{
  geometry_msgs::msg::Pose2D to;
  copyPose2D(from, to);
  return to;
}

PoseArray fromRos(const geometry_msgs::msg::PoseArray & from)
{
  PoseArray to;
  copyHeader(from.header, *to.mutable_header());
  for (const auto & pose : from.poses) {
    copyPose(pose, *to.add_poses());
  }
  return to;
}

geometry_msgs::msg::PoseArray toRos(const PoseArray & from)
{
  geometry_msgs::msg::PoseArray to;
  copyHeader(from.header(), to.header);
  to.poses.reserve(static_cast<size_t>(from.poses_size()));
  for (const auto & pose : from.poses()) {
    to.poses.push_back(toRos(pose));
  }
  return to;
}

PoseStamped fromRos(const geometry_msgs::msg::PoseStamped & from)
{
  PoseStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyPose(from.pose, *to.mutable_pose());
  return to;
}

geometry_msgs::msg::PoseStamped toRos(const PoseStamped & from)
{
  geometry_msgs::msg::PoseStamped to;
  copyHeader(from.header(), to.header);
  copyPose(from.pose(), to.pose);
  return to;
}

PoseWithCovariance fromRos(const geometry_msgs::msg::PoseWithCovariance & from)
{
  PoseWithCovariance to;
  copyPose(from.pose, *to.mutable_pose()->mutable_pose());
  copyCovariance6(from.covariance, to.mutable_covariance());
  return to;
}

geometry_msgs::msg::PoseWithCovariance toRos(const PoseWithCovariance & from)
{
  geometry_msgs::msg::PoseWithCovariance to;
  copyPose(from.pose().pose(), to.pose);
  copyCovariance6(from.covariance(), to.covariance);
  return to;
}

PoseWithCovarianceStamped fromRos(
  const geometry_msgs::msg::PoseWithCovarianceStamped & from)
{
  PoseWithCovarianceStamped to;
  copyHeader(from.header, *to.mutable_header());
  *to.mutable_pose() = fromRos(from.pose);
  return to;
}

geometry_msgs::msg::PoseWithCovarianceStamped toRos(
  const PoseWithCovarianceStamped & from)
{
  geometry_msgs::msg::PoseWithCovarianceStamped to;
  copyHeader(from.header(), to.header);
  to.pose = toRos(from.pose());
  return to;
}

QuaternionStamped fromRos(const geometry_msgs::msg::QuaternionStamped & from)
{
  QuaternionStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyQuaternion(from.quaternion, *to.mutable_quaternion());
  return to;
}

geometry_msgs::msg::QuaternionStamped toRos(const QuaternionStamped & from)
{
  geometry_msgs::msg::QuaternionStamped to;
  copyHeader(from.header(), to.header);
  copyQuaternion(from.quaternion(), to.quaternion);
  return to;
}

Transform fromRos(const geometry_msgs::msg::Transform & from)
{
  Transform to;
  copyTransform(from, to);
  return to;
}

geometry_msgs::msg::Transform toRos(const Transform & from)
{
  geometry_msgs::msg::Transform to;
  copyTransform(from, to);
  return to;
}

TransformStamped fromRos(const geometry_msgs::msg::TransformStamped & from)
{
  TransformStamped to;
  copyHeader(from.header, *to.mutable_header());
  to.set_child_frame_id(from.child_frame_id);
  copyTransform(from.transform, *to.mutable_transform());
  return to;
}

geometry_msgs::msg::TransformStamped toRos(const TransformStamped & from)
{
  geometry_msgs::msg::TransformStamped to;
  copyHeader(from.header(), to.header);
  to.child_frame_id = from.child_frame_id();
  copyTransform(from.transform(), to.transform);
  return to;
}

Twist fromRos(const geometry_msgs::msg::Twist & from)
{
  Twist to;
  copyTwist(from, to);
  return to;
}

geometry_msgs::msg::Twist toRos(const Twist & from)
{
  geometry_msgs::msg::Twist to;
  copyTwist(from, to);
  return to;
}

TwistStamped fromRos(const geometry_msgs::msg::TwistStamped & from)
{
  TwistStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyTwist(from.twist, *to.mutable_twist());
  return to;
}

geometry_msgs::msg::TwistStamped toRos(const TwistStamped & from)
{
  geometry_msgs::msg::TwistStamped to;
  copyHeader(from.header(), to.header);
  copyTwist(from.twist(), to.twist);
  return to;
}

TwistWithCovariance fromRos(const geometry_msgs::msg::TwistWithCovariance & from)
{
  TwistWithCovariance to;
  copyTwist(from.twist, *to.mutable_twist());
  copyCovariance6(from.covariance, to.mutable_covariance());
  return to;
}

geometry_msgs::msg::TwistWithCovariance toRos(const TwistWithCovariance & from)
{
  geometry_msgs::msg::TwistWithCovariance to;
  copyTwist(from.twist(), to.twist);
  copyCovariance6(from.covariance(), to.covariance);
  return to;
}

TwistWithCovarianceStamped fromRos(
  const geometry_msgs::msg::TwistWithCovarianceStamped & from)
{
  TwistWithCovarianceStamped to;
  copyHeader(from.header, *to.mutable_header());
  *to.mutable_twist() = fromRos(from.twist);
  return to;
}

geometry_msgs::msg::TwistWithCovarianceStamped toRos(
  const TwistWithCovarianceStamped & from)
{
  geometry_msgs::msg::TwistWithCovarianceStamped to;
  copyHeader(from.header(), to.header);
  to.twist = toRos(from.twist());
  return to;
}

Accel fromRos(const geometry_msgs::msg::Accel & from)
{
  Accel to;
  copyAccel(from, to);
  return to;
}

geometry_msgs::msg::Accel toRos(const Accel & from)
{
  geometry_msgs::msg::Accel to;
  copyAccel(from, to);
  return to;
}

AccelStamped fromRos(const geometry_msgs::msg::AccelStamped & from)
{
  AccelStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyAccel(from.accel, *to.mutable_accel());
  return to;
}

geometry_msgs::msg::AccelStamped toRos(const AccelStamped & from)
{
  geometry_msgs::msg::AccelStamped to;
  copyHeader(from.header(), to.header);
  copyAccel(from.accel(), to.accel);
  return to;
}

AccelWithCovariance fromRos(const geometry_msgs::msg::AccelWithCovariance & from)
{
  AccelWithCovariance to;
  copyAccel(from.accel, *to.mutable_accel());
  copyCovariance6(from.covariance, to.mutable_covariance());
  return to;
}

geometry_msgs::msg::AccelWithCovariance toRos(const AccelWithCovariance & from)
{
  geometry_msgs::msg::AccelWithCovariance to;
  copyAccel(from.accel(), to.accel);
  copyCovariance6(from.covariance(), to.covariance);
  return to;
}

AccelWithCovarianceStamped fromRos(
  const geometry_msgs::msg::AccelWithCovarianceStamped & from)
{
  AccelWithCovarianceStamped to;
  copyHeader(from.header, *to.mutable_header());
  *to.mutable_accel() = fromRos(from.accel);
  return to;
}

geometry_msgs::msg::AccelWithCovarianceStamped toRos(
  const AccelWithCovarianceStamped & from)
{
  geometry_msgs::msg::AccelWithCovarianceStamped to;
  copyHeader(from.header(), to.header);
  to.accel = toRos(from.accel());
  return to;
}

Inertia fromRos(const geometry_msgs::msg::Inertia & from)
{
  Inertia to;
  copyInertia(from, to);
  return to;
}

geometry_msgs::msg::Inertia toRos(const Inertia & from)
{
  geometry_msgs::msg::Inertia to;
  copyInertia(from, to);
  return to;
}

InertiaStamped fromRos(const geometry_msgs::msg::InertiaStamped & from)
{
  InertiaStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyInertia(from.inertia, *to.mutable_inertia());
  return to;
}

geometry_msgs::msg::InertiaStamped toRos(const InertiaStamped & from)
{
  geometry_msgs::msg::InertiaStamped to;
  copyHeader(from.header(), to.header);
  copyInertia(from.inertia(), to.inertia);
  return to;
}

PointStamped fromRos(const geometry_msgs::msg::PointStamped & from)
{
  PointStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyPoint(from.point, *to.mutable_point());
  return to;
}

geometry_msgs::msg::PointStamped toRos(const PointStamped & from)
{
  geometry_msgs::msg::PointStamped to;
  copyHeader(from.header(), to.header);
  copyPoint(from.point(), to.point);
  return to;
}

Polygon fromRos(const geometry_msgs::msg::Polygon & from)
{
  Polygon to;
  copyPolygon(from, to);
  return to;
}

geometry_msgs::msg::Polygon toRos(const Polygon & from)
{
  geometry_msgs::msg::Polygon to;
  copyPolygon(from, to);
  return to;
}

PolygonStamped fromRos(const geometry_msgs::msg::PolygonStamped & from)
{
  PolygonStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyPolygon(from.polygon, *to.mutable_polygon());
  return to;
}

geometry_msgs::msg::PolygonStamped toRos(const PolygonStamped & from)
{
  geometry_msgs::msg::PolygonStamped to;
  copyHeader(from.header(), to.header);
  copyPolygon(from.polygon(), to.polygon);
  return to;
}

Vector3Stamped fromRos(const geometry_msgs::msg::Vector3Stamped & from)
{
  Vector3Stamped to;
  copyHeader(from.header, *to.mutable_header());
  copyVector3(from.vector, *to.mutable_vector());
  return to;
}

geometry_msgs::msg::Vector3Stamped toRos(const Vector3Stamped & from)
{
  geometry_msgs::msg::Vector3Stamped to;
  copyHeader(from.header(), to.header);
  copyVector3(from.vector(), to.vector);
  return to;
}

Wrench fromRos(const geometry_msgs::msg::Wrench & from)
{
  Wrench to;
  copyWrench(from, to);
  return to;
}

geometry_msgs::msg::Wrench toRos(const Wrench & from)
{
  geometry_msgs::msg::Wrench to;
  copyWrench(from, to);
  return to;
}

WrenchStamped fromRos(const geometry_msgs::msg::WrenchStamped & from)
{
  WrenchStamped to;
  copyHeader(from.header, *to.mutable_header());
  copyWrench(from.wrench, *to.mutable_wrench());
  return to;
}

geometry_msgs::msg::WrenchStamped toRos(const WrenchStamped & from)
{
  geometry_msgs::msg::WrenchStamped to;
  copyHeader(from.header(), to.header);
  copyWrench(from.wrench(), to.wrench);
  return to;
}

}  // namespace autonomy_ros
