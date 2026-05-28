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

#ifndef AUTONOMY_ROS__CONVERSIONS__GEOMETRY_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__GEOMETRY_MSGS_HPP_

/// @file geometry_msgs.hpp
/// @brief Converts ROS 2 geometry_msgs and autonomy::commsgs::geometry_msgs.
///
/// Proto schema: autonomy/commsgs/proto/geometry_msgs.proto
///
/// @par Overview
/// Every listed type provides paired fromRos() and toRos() overloads. Conversions
/// copy fields only; they do not apply TF transforms or change reference frames.
///
/// @par Not implemented (commsgs-only or other entry points)
/// VelocityStamped, PointENU, PointLLH, Twist2D, Twist2DStamped.
/// TransformStampeds is converted via tf2_msgs.hpp (TFMessage).
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) in bridges and nodes
/// that handle poses, twists, transforms, and polygons.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/geometry_msgs.hpp"
/// auto goal = autonomy_ros::fromRos(*ros_pose_stamped);
/// cmd_pub->publish(autonomy_ros::toRos(core_twist_stamped));
/// @endcode

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/inertia.hpp"
#include "geometry_msgs/msg/inertia_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace autonomy_ros
{

using Vector3 = ::autonomy::commsgs::geometry_msgs::Vector3;
using Point = ::autonomy::commsgs::geometry_msgs::Point;
using Point32 = ::autonomy::commsgs::geometry_msgs::Point32;
using Quaternion = ::autonomy::commsgs::geometry_msgs::Quaternion;
using Pose = ::autonomy::commsgs::geometry_msgs::Pose;
using Pose2D = ::autonomy::commsgs::geometry_msgs::Pose2D;
using PoseArray = ::autonomy::commsgs::geometry_msgs::PoseArray;
using PoseStamped = ::autonomy::commsgs::geometry_msgs::PoseStamped;
using PoseWithCovariance = ::autonomy::commsgs::geometry_msgs::PoseWithCovariance;
using PoseWithCovarianceStamped =
  ::autonomy::commsgs::geometry_msgs::PoseWithCovarianceStamped;
using QuaternionStamped = ::autonomy::commsgs::geometry_msgs::QuaternionStamped;
using Transform = ::autonomy::commsgs::geometry_msgs::Transform;
using TransformStamped = ::autonomy::commsgs::geometry_msgs::TransformStamped;
using Twist = ::autonomy::commsgs::geometry_msgs::Twist;
using TwistStamped = ::autonomy::commsgs::geometry_msgs::TwistStamped;
using TwistWithCovariance = ::autonomy::commsgs::geometry_msgs::TwistWithCovariance;
using TwistWithCovarianceStamped =
  ::autonomy::commsgs::geometry_msgs::TwistWithCovarianceStamped;
using Accel = ::autonomy::commsgs::geometry_msgs::Accel;
using AccelStamped = ::autonomy::commsgs::geometry_msgs::AccelStamped;
using AccelWithCovariance = ::autonomy::commsgs::geometry_msgs::AccelWithCovariance;
using AccelWithCovarianceStamped =
  ::autonomy::commsgs::geometry_msgs::AccelWithCovarianceStamped;
using Inertia = ::autonomy::commsgs::geometry_msgs::Inertia;
using InertiaStamped = ::autonomy::commsgs::geometry_msgs::InertiaStamped;
using PointStamped = ::autonomy::commsgs::geometry_msgs::PointStamped;
using Polygon = ::autonomy::commsgs::geometry_msgs::Polygon;
using PolygonStamped = ::autonomy::commsgs::geometry_msgs::PolygonStamped;
using Vector3Stamped = ::autonomy::commsgs::geometry_msgs::Vector3Stamped;
using Wrench = ::autonomy::commsgs::geometry_msgs::Wrench;
using WrenchStamped = ::autonomy::commsgs::geometry_msgs::WrenchStamped;

// --- Primitives ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Vector3 and commsgs Vector3.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Vector3). Fields are copied without coordinate transforms.
 * @return commsgs Vector3 for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Vector3 from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Vector3 ready for rclcpp publish() or subscribe() adapters.
 */
Vector3 fromRos(const geometry_msgs::msg::Vector3 & from);
geometry_msgs::msg::Vector3 toRos(const Vector3 & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Point and commsgs Point.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Point). Fields are copied without coordinate transforms.
 * @return commsgs Point for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Point from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Point ready for rclcpp publish() or subscribe() adapters.
 */
Point fromRos(const geometry_msgs::msg::Point & from);
geometry_msgs::msg::Point toRos(const Point & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Point32 and commsgs Point32.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Point32). Fields are copied without coordinate transforms.
 * @return commsgs Point32 for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Point32 from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Point32 ready for rclcpp publish() or subscribe() adapters.
 */
Point32 fromRos(const geometry_msgs::msg::Point32 & from);
geometry_msgs::msg::Point32 toRos(const Point32 & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Quaternion and commsgs Quaternion.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Quaternion). Fields are copied without coordinate transforms.
 * @return commsgs Quaternion for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Quaternion from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Quaternion ready for rclcpp publish() or subscribe() adapters.
 */
Quaternion fromRos(const geometry_msgs::msg::Quaternion & from);
geometry_msgs::msg::Quaternion toRos(const Quaternion & from);

// --- Pose ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Pose and commsgs Pose.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Pose). Fields are copied without coordinate transforms.
 * @return commsgs Pose for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Pose from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Pose ready for rclcpp publish() or subscribe() adapters.
 */
Pose fromRos(const geometry_msgs::msg::Pose & from);
geometry_msgs::msg::Pose toRos(const Pose & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Pose2D and commsgs Pose2D.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Pose2D). Fields are copied without coordinate transforms.
 * @return commsgs Pose2D for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Pose2D from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Pose2D ready for rclcpp publish() or subscribe() adapters.
 */
Pose2D fromRos(const geometry_msgs::msg::Pose2D & from);
geometry_msgs::msg::Pose2D toRos(const Pose2D & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseArray and commsgs PoseArray.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseArray). Fields are copied without coordinate transforms.
 * @return commsgs PoseArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PoseArray from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseArray ready for rclcpp publish() or subscribe() adapters.
 */
PoseArray fromRos(const geometry_msgs::msg::PoseArray & from);
geometry_msgs::msg::PoseArray toRos(const PoseArray & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseStamped and commsgs PoseStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseStamped). Fields are copied without coordinate transforms.
 * @return commsgs PoseStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PoseStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseStamped ready for rclcpp publish() or subscribe() adapters.
 */
PoseStamped fromRos(const geometry_msgs::msg::PoseStamped & from);
geometry_msgs::msg::PoseStamped toRos(const PoseStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseWithCovariance and commsgs PoseWithCovariance.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseWithCovariance). Fields are copied without coordinate transforms.
 * @return commsgs PoseWithCovariance for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PoseWithCovariance from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseWithCovariance ready for rclcpp publish() or subscribe() adapters.
 */
PoseWithCovariance fromRos(const geometry_msgs::msg::PoseWithCovariance & from);
geometry_msgs::msg::PoseWithCovariance toRos(const PoseWithCovariance & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseWithCovarianceStamped and commsgs PoseWithCovarianceStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseWithCovarianceStamped). Fields are copied without coordinate transforms.
 * @return commsgs PoseWithCovarianceStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PoseWithCovarianceStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseWithCovarianceStamped ready for rclcpp publish() or subscribe() adapters.
 */
PoseWithCovarianceStamped fromRos(const geometry_msgs::msg::PoseWithCovarianceStamped & from);
geometry_msgs::msg::PoseWithCovarianceStamped toRos(const PoseWithCovarianceStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::QuaternionStamped and commsgs QuaternionStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::QuaternionStamped). Fields are copied without coordinate transforms.
 * @return commsgs QuaternionStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs QuaternionStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::QuaternionStamped ready for rclcpp publish() or subscribe() adapters.
 */
QuaternionStamped fromRos(const geometry_msgs::msg::QuaternionStamped & from);
geometry_msgs::msg::QuaternionStamped toRos(const QuaternionStamped & from);

// --- Transform ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Transform and commsgs Transform.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Transform). Fields are copied without coordinate transforms.
 * @return commsgs Transform for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Transform from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Transform ready for rclcpp publish() or subscribe() adapters.
 */
Transform fromRos(const geometry_msgs::msg::Transform & from);
geometry_msgs::msg::Transform toRos(const Transform & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TransformStamped and commsgs TransformStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TransformStamped). Fields are copied without coordinate transforms.
 * @return commsgs TransformStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs TransformStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TransformStamped ready for rclcpp publish() or subscribe() adapters.
 */
TransformStamped fromRos(const geometry_msgs::msg::TransformStamped & from);
geometry_msgs::msg::TransformStamped toRos(const TransformStamped & from);

// --- Velocity / acceleration ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Twist and commsgs Twist.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Twist). Fields are copied without coordinate transforms.
 * @return commsgs Twist for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Twist from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Twist ready for rclcpp publish() or subscribe() adapters.
 */
Twist fromRos(const geometry_msgs::msg::Twist & from);
geometry_msgs::msg::Twist toRos(const Twist & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TwistStamped and commsgs TwistStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TwistStamped). Fields are copied without coordinate transforms.
 * @return commsgs TwistStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs TwistStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TwistStamped ready for rclcpp publish() or subscribe() adapters.
 */
TwistStamped fromRos(const geometry_msgs::msg::TwistStamped & from);
geometry_msgs::msg::TwistStamped toRos(const TwistStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TwistWithCovariance and commsgs TwistWithCovariance.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TwistWithCovariance). Fields are copied without coordinate transforms.
 * @return commsgs TwistWithCovariance for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs TwistWithCovariance from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TwistWithCovariance ready for rclcpp publish() or subscribe() adapters.
 */
TwistWithCovariance fromRos(const geometry_msgs::msg::TwistWithCovariance & from);
geometry_msgs::msg::TwistWithCovariance toRos(const TwistWithCovariance & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TwistWithCovarianceStamped and commsgs TwistWithCovarianceStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TwistWithCovarianceStamped). Fields are copied without coordinate transforms.
 * @return commsgs TwistWithCovarianceStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs TwistWithCovarianceStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TwistWithCovarianceStamped ready for rclcpp publish() or subscribe() adapters.
 */
TwistWithCovarianceStamped fromRos(const geometry_msgs::msg::TwistWithCovarianceStamped & from);
geometry_msgs::msg::TwistWithCovarianceStamped toRos(const TwistWithCovarianceStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Accel and commsgs Accel.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Accel). Fields are copied without coordinate transforms.
 * @return commsgs Accel for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Accel from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Accel ready for rclcpp publish() or subscribe() adapters.
 */
Accel fromRos(const geometry_msgs::msg::Accel & from);
geometry_msgs::msg::Accel toRos(const Accel & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::AccelStamped and commsgs AccelStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::AccelStamped). Fields are copied without coordinate transforms.
 * @return commsgs AccelStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs AccelStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::AccelStamped ready for rclcpp publish() or subscribe() adapters.
 */
AccelStamped fromRos(const geometry_msgs::msg::AccelStamped & from);
geometry_msgs::msg::AccelStamped toRos(const AccelStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::AccelWithCovariance and commsgs AccelWithCovariance.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::AccelWithCovariance). Fields are copied without coordinate transforms.
 * @return commsgs AccelWithCovariance for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs AccelWithCovariance from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::AccelWithCovariance ready for rclcpp publish() or subscribe() adapters.
 */
AccelWithCovariance fromRos(const geometry_msgs::msg::AccelWithCovariance & from);
geometry_msgs::msg::AccelWithCovariance toRos(const AccelWithCovariance & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::AccelWithCovarianceStamped and commsgs AccelWithCovarianceStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::AccelWithCovarianceStamped). Fields are copied without coordinate transforms.
 * @return commsgs AccelWithCovarianceStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs AccelWithCovarianceStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::AccelWithCovarianceStamped ready for rclcpp publish() or subscribe() adapters.
 */
AccelWithCovarianceStamped fromRos(const geometry_msgs::msg::AccelWithCovarianceStamped & from);
geometry_msgs::msg::AccelWithCovarianceStamped toRos(const AccelWithCovarianceStamped & from);

// --- Inertia / polygons / wrench ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Inertia and commsgs Inertia.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Inertia). Fields are copied without coordinate transforms.
 * @return commsgs Inertia for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Inertia from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Inertia ready for rclcpp publish() or subscribe() adapters.
 */
Inertia fromRos(const geometry_msgs::msg::Inertia & from);
geometry_msgs::msg::Inertia toRos(const Inertia & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::InertiaStamped and commsgs InertiaStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::InertiaStamped). Fields are copied without coordinate transforms.
 * @return commsgs InertiaStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs InertiaStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::InertiaStamped ready for rclcpp publish() or subscribe() adapters.
 */
InertiaStamped fromRos(const geometry_msgs::msg::InertiaStamped & from);
geometry_msgs::msg::InertiaStamped toRos(const InertiaStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PointStamped and commsgs PointStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PointStamped). Fields are copied without coordinate transforms.
 * @return commsgs PointStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PointStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PointStamped ready for rclcpp publish() or subscribe() adapters.
 */
PointStamped fromRos(const geometry_msgs::msg::PointStamped & from);
geometry_msgs::msg::PointStamped toRos(const PointStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Polygon and commsgs Polygon.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Polygon). Fields are copied without coordinate transforms.
 * @return commsgs Polygon for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Polygon from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Polygon ready for rclcpp publish() or subscribe() adapters.
 */
Polygon fromRos(const geometry_msgs::msg::Polygon & from);
geometry_msgs::msg::Polygon toRos(const Polygon & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PolygonStamped and commsgs PolygonStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PolygonStamped). Fields are copied without coordinate transforms.
 * @return commsgs PolygonStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PolygonStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PolygonStamped ready for rclcpp publish() or subscribe() adapters.
 */
PolygonStamped fromRos(const geometry_msgs::msg::PolygonStamped & from);
geometry_msgs::msg::PolygonStamped toRos(const PolygonStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Vector3Stamped and commsgs Vector3Stamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Vector3Stamped). Fields are copied without coordinate transforms.
 * @return commsgs Vector3Stamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Vector3Stamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Vector3Stamped ready for rclcpp publish() or subscribe() adapters.
 */
Vector3Stamped fromRos(const geometry_msgs::msg::Vector3Stamped & from);
geometry_msgs::msg::Vector3Stamped toRos(const Vector3Stamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Wrench and commsgs Wrench.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Wrench). Fields are copied without coordinate transforms.
 * @return commsgs Wrench for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Wrench from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Wrench ready for rclcpp publish() or subscribe() adapters.
 */
Wrench fromRos(const geometry_msgs::msg::Wrench & from);
geometry_msgs::msg::Wrench toRos(const Wrench & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::WrenchStamped and commsgs WrenchStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::WrenchStamped). Fields are copied without coordinate transforms.
 * @return commsgs WrenchStamped for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs WrenchStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::WrenchStamped ready for rclcpp publish() or subscribe() adapters.
 */
WrenchStamped fromRos(const geometry_msgs::msg::WrenchStamped & from);
geometry_msgs::msg::WrenchStamped toRos(const WrenchStamped & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__GEOMETRY_MSGS_HPP_
