// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/geometry_msgs.hpp"

#include <algorithm>

#include "autonomy_ros/conversions/detail.hpp"

namespace autonomy_ros::conversions
{

Vector3 fromRos(const geometry_msgs::msg::Vector3 & from)
{
  Vector3 to;
  detail::copyVector3(from, to);
  return to;
}

geometry_msgs::msg::Vector3 toRos(const Vector3 & from)
{
  geometry_msgs::msg::Vector3 to;
  detail::copyVector3(from, to);
  return to;
}

Point fromRos(const geometry_msgs::msg::Point & from)
{
  Point to;
  detail::copyPoint(from, to);
  return to;
}

geometry_msgs::msg::Point toRos(const Point & from)
{
  geometry_msgs::msg::Point to;
  detail::copyPoint(from, to);
  return to;
}

Point32 fromRos(const geometry_msgs::msg::Point32 & from)
{
  Point32 to;
  detail::copyPoint32(from, to);
  return to;
}

geometry_msgs::msg::Point32 toRos(const Point32 & from)
{
  geometry_msgs::msg::Point32 to;
  detail::copyPoint32(from, to);
  return to;
}

Quaternion fromRos(const geometry_msgs::msg::Quaternion & from)
{
  Quaternion to;
  detail::copyQuaternion(from, to);
  return to;
}

geometry_msgs::msg::Quaternion toRos(const Quaternion & from)
{
  geometry_msgs::msg::Quaternion to;
  detail::copyQuaternion(from, to);
  return to;
}

Pose fromRos(const geometry_msgs::msg::Pose & from)
{
  Pose to;
  detail::copyPose(from, to);
  return to;
}

geometry_msgs::msg::Pose toRos(const Pose & from)
{
  geometry_msgs::msg::Pose to;
  detail::copyPose(from, to);
  return to;
}

Pose2D fromRos(const geometry_msgs::msg::Pose2D & from)
{
  Pose2D to;
  detail::copyPose2D(from, to);
  return to;
}

geometry_msgs::msg::Pose2D toRos(const Pose2D & from)
{
  geometry_msgs::msg::Pose2D to;
  detail::copyPose2D(from, to);
  return to;
}

PoseArray fromRos(const geometry_msgs::msg::PoseArray & from)
{
  PoseArray to;
  detail::copyHeader(from.header, to.header);
  to.poses.reserve(from.poses.size());
  for (const auto & pose : from.poses) {
    to.poses.push_back(fromRos(pose));
  }
  return to;
}

geometry_msgs::msg::PoseArray toRos(const PoseArray & from)
{
  geometry_msgs::msg::PoseArray to;
  detail::copyHeader(from.header, to.header);
  to.poses.reserve(from.poses.size());
  for (const auto & pose : from.poses) {
    to.poses.push_back(toRos(pose));
  }
  return to;
}

PoseStamped fromRos(const geometry_msgs::msg::PoseStamped & from)
{
  PoseStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyPose(from.pose, to.pose);
  return to;
}

geometry_msgs::msg::PoseStamped toRos(const PoseStamped & from)
{
  geometry_msgs::msg::PoseStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyPose(from.pose, to.pose);
  return to;
}

PoseWithCovariance fromRos(const geometry_msgs::msg::PoseWithCovariance & from)
{
  PoseWithCovariance to;
  detail::copyPose(from.pose, to.pose);
  detail::copyCovariance6(from.covariance, to.covariance);
  return to;
}

geometry_msgs::msg::PoseWithCovariance toRos(const PoseWithCovariance & from)
{
  geometry_msgs::msg::PoseWithCovariance to;
  detail::copyPose(from.pose, to.pose);
  detail::copyCovariance6(from.covariance, to.covariance);
  return to;
}

PoseWithCovarianceStamped fromRos(
  const geometry_msgs::msg::PoseWithCovarianceStamped & from)
{
  PoseWithCovarianceStamped to;
  detail::copyHeader(from.header, to.header);
  to.pose = fromRos(from.pose);
  return to;
}

geometry_msgs::msg::PoseWithCovarianceStamped toRos(
  const PoseWithCovarianceStamped & from)
{
  geometry_msgs::msg::PoseWithCovarianceStamped to;
  detail::copyHeader(from.header, to.header);
  to.pose = toRos(from.pose);
  return to;
}

QuaternionStamped fromRos(const geometry_msgs::msg::QuaternionStamped & from)
{
  QuaternionStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyQuaternion(from.quaternion, to.quaternion);
  return to;
}

geometry_msgs::msg::QuaternionStamped toRos(const QuaternionStamped & from)
{
  geometry_msgs::msg::QuaternionStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyQuaternion(from.quaternion, to.quaternion);
  return to;
}

Transform fromRos(const geometry_msgs::msg::Transform & from)
{
  Transform to;
  detail::copyTransform(from, to);
  return to;
}

geometry_msgs::msg::Transform toRos(const Transform & from)
{
  geometry_msgs::msg::Transform to;
  detail::copyTransform(from, to);
  return to;
}

TransformStamped fromRos(const geometry_msgs::msg::TransformStamped & from)
{
  TransformStamped to;
  detail::copyHeader(from.header, to.header);
  to.child_frame_id = from.child_frame_id;
  detail::copyTransform(from.transform, to.transform);
  return to;
}

geometry_msgs::msg::TransformStamped toRos(const TransformStamped & from)
{
  geometry_msgs::msg::TransformStamped to;
  detail::copyHeader(from.header, to.header);
  to.child_frame_id = from.child_frame_id;
  detail::copyTransform(from.transform, to.transform);
  return to;
}

Twist fromRos(const geometry_msgs::msg::Twist & from)
{
  Twist to;
  detail::copyTwist(from, to);
  return to;
}

geometry_msgs::msg::Twist toRos(const Twist & from)
{
  geometry_msgs::msg::Twist to;
  detail::copyTwist(from, to);
  return to;
}

TwistStamped fromRos(const geometry_msgs::msg::TwistStamped & from)
{
  TwistStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyTwist(from.twist, to.twist);
  return to;
}

geometry_msgs::msg::TwistStamped toRos(const TwistStamped & from)
{
  geometry_msgs::msg::TwistStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyTwist(from.twist, to.twist);
  return to;
}

TwistWithCovariance fromRos(const geometry_msgs::msg::TwistWithCovariance & from)
{
  TwistWithCovariance to;
  detail::copyTwist(from.twist, to.twist);
  detail::copyCovariance6(from.covariance, to.covariance);
  return to;
}

geometry_msgs::msg::TwistWithCovariance toRos(const TwistWithCovariance & from)
{
  geometry_msgs::msg::TwistWithCovariance to;
  detail::copyTwist(from.twist, to.twist);
  detail::copyCovariance6(from.covariance, to.covariance);
  return to;
}

TwistWithCovarianceStamped fromRos(
  const geometry_msgs::msg::TwistWithCovarianceStamped & from)
{
  TwistWithCovarianceStamped to;
  detail::copyHeader(from.header, to.header);
  to.twist = fromRos(from.twist);
  return to;
}

geometry_msgs::msg::TwistWithCovarianceStamped toRos(
  const TwistWithCovarianceStamped & from)
{
  geometry_msgs::msg::TwistWithCovarianceStamped to;
  detail::copyHeader(from.header, to.header);
  to.twist = toRos(from.twist);
  return to;
}

Accel fromRos(const geometry_msgs::msg::Accel & from)
{
  Accel to;
  detail::copyAccel(from, to);
  return to;
}

geometry_msgs::msg::Accel toRos(const Accel & from)
{
  geometry_msgs::msg::Accel to;
  detail::copyAccel(from, to);
  return to;
}

AccelStamped fromRos(const geometry_msgs::msg::AccelStamped & from)
{
  AccelStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyAccel(from.accel, to.accel);
  return to;
}

geometry_msgs::msg::AccelStamped toRos(const AccelStamped & from)
{
  geometry_msgs::msg::AccelStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyAccel(from.accel, to.accel);
  return to;
}

AccelWithCovariance fromRos(const geometry_msgs::msg::AccelWithCovariance & from)
{
  AccelWithCovariance to;
  detail::copyAccel(from.accel, to.accel);
  to.covariance.resize(from.covariance.size());
  std::transform(
    from.covariance.begin(), from.covariance.end(), to.covariance.begin(),
    [](double v) { return static_cast<float>(v); });
  return to;
}

geometry_msgs::msg::AccelWithCovariance toRos(const AccelWithCovariance & from)
{
  geometry_msgs::msg::AccelWithCovariance to;
  detail::copyAccel(from.accel, to.accel);
  if (from.covariance.size() >= to.covariance.size()) {
    std::transform(
      from.covariance.begin(),
      from.covariance.begin() + static_cast<std::ptrdiff_t>(to.covariance.size()),
      to.covariance.begin(),
      [](float v) { return static_cast<double>(v); });
  }
  return to;
}

AccelWithCovarianceStamped fromRos(
  const geometry_msgs::msg::AccelWithCovarianceStamped & from)
{
  AccelWithCovarianceStamped to;
  detail::copyHeader(from.header, to.header);
  to.accel = fromRos(from.accel);
  return to;
}

geometry_msgs::msg::AccelWithCovarianceStamped toRos(
  const AccelWithCovarianceStamped & from)
{
  geometry_msgs::msg::AccelWithCovarianceStamped to;
  detail::copyHeader(from.header, to.header);
  to.accel = toRos(from.accel);
  return to;
}

Inertia fromRos(const geometry_msgs::msg::Inertia & from)
{
  Inertia to;
  detail::copyInertia(from, to);
  return to;
}

geometry_msgs::msg::Inertia toRos(const Inertia & from)
{
  geometry_msgs::msg::Inertia to;
  detail::copyInertia(from, to);
  return to;
}

InertiaStamped fromRos(const geometry_msgs::msg::InertiaStamped & from)
{
  InertiaStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyInertia(from.inertia, to.inertia);
  return to;
}

geometry_msgs::msg::InertiaStamped toRos(const InertiaStamped & from)
{
  geometry_msgs::msg::InertiaStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyInertia(from.inertia, to.inertia);
  return to;
}

PointStamped fromRos(const geometry_msgs::msg::PointStamped & from)
{
  PointStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyPoint(from.point, to.point);
  return to;
}

geometry_msgs::msg::PointStamped toRos(const PointStamped & from)
{
  geometry_msgs::msg::PointStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyPoint(from.point, to.point);
  return to;
}

Polygon fromRos(const geometry_msgs::msg::Polygon & from)
{
  Polygon to;
  detail::copyPolygon(from, to);
  return to;
}

geometry_msgs::msg::Polygon toRos(const Polygon & from)
{
  geometry_msgs::msg::Polygon to;
  detail::copyPolygon(from, to);
  return to;
}

PolygonStamped fromRos(const geometry_msgs::msg::PolygonStamped & from)
{
  PolygonStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyPolygon(from.polygon, to.polygon);
  return to;
}

geometry_msgs::msg::PolygonStamped toRos(const PolygonStamped & from)
{
  geometry_msgs::msg::PolygonStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyPolygon(from.polygon, to.polygon);
  return to;
}

Vector3Stamped fromRos(const geometry_msgs::msg::Vector3Stamped & from)
{
  Vector3Stamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyVector3(from.vector, to.vector);
  return to;
}

geometry_msgs::msg::Vector3Stamped toRos(const Vector3Stamped & from)
{
  geometry_msgs::msg::Vector3Stamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyVector3(from.vector, to.vector);
  return to;
}

Wrench fromRos(const geometry_msgs::msg::Wrench & from)
{
  Wrench to;
  detail::copyWrench(from, to);
  return to;
}

geometry_msgs::msg::Wrench toRos(const Wrench & from)
{
  geometry_msgs::msg::Wrench to;
  detail::copyWrench(from, to);
  return to;
}

WrenchStamped fromRos(const geometry_msgs::msg::WrenchStamped & from)
{
  WrenchStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyWrench(from.wrench, to.wrench);
  return to;
}

geometry_msgs::msg::WrenchStamped toRos(const WrenchStamped & from)
{
  geometry_msgs::msg::WrenchStamped to;
  detail::copyHeader(from.header, to.header);
  detail::copyWrench(from.wrench, to.wrench);
  return to;
}

}  // namespace autonomy_ros::conversions
