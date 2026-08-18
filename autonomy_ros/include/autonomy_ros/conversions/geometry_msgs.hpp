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
/// @brief Converts ROS 2 geometry_msgs and automsgs::msgs::geometry_msgs.
///
/// Proto schema: automsgs/msgs/geometry_msgs/
///
/// @par Overview
/// Every listed type provides paired fromRos() and toRos() overloads. Conversions
/// copy fields only; they do not apply TF transforms or change reference frames.
///
/// @par Not implemented (automsgs-only or other entry points)
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

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_array.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_with_covariance.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/point32.pb.h>
#include <automsgs/msgs/geometry_msgs/point_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_with_covariance.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_with_covariance_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/accel.pb.h>
#include <automsgs/msgs/geometry_msgs/accel_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/accel_with_covariance.pb.h>
#include <automsgs/msgs/geometry_msgs/accel_with_covariance_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/inertia.pb.h>
#include <automsgs/msgs/geometry_msgs/inertia_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/wrench.pb.h>
#include <automsgs/msgs/geometry_msgs/wrench_stamped.pb.h>
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

using Vector3 = ::automsgs::msgs::geometry_msgs::Vector3;
using Point = ::automsgs::msgs::geometry_msgs::Point;
using Point32 = ::automsgs::msgs::geometry_msgs::Point32;
using Quaternion = ::automsgs::msgs::geometry_msgs::Quaternion;
using Pose = ::automsgs::msgs::geometry_msgs::Pose;
using Pose2D = ::automsgs::msgs::geometry_msgs::Pose2D;
using PoseArray = ::automsgs::msgs::geometry_msgs::PoseArray;
using PoseStamped = ::automsgs::msgs::geometry_msgs::PoseStamped;
using PoseWithCovariance = ::automsgs::msgs::geometry_msgs::PoseWithCovariance;
using PoseWithCovarianceStamped =
  ::automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped;
using QuaternionStamped = ::automsgs::msgs::geometry_msgs::QuaternionStamped;
using Transform = ::automsgs::msgs::geometry_msgs::Transform;
using TransformStamped = ::automsgs::msgs::geometry_msgs::TransformStamped;
using Twist = ::automsgs::msgs::geometry_msgs::Twist;
using TwistStamped = ::automsgs::msgs::geometry_msgs::TwistStamped;
using TwistWithCovariance = ::automsgs::msgs::geometry_msgs::TwistWithCovariance;
using TwistWithCovarianceStamped =
  ::automsgs::msgs::geometry_msgs::TwistWithCovarianceStamped;
using Accel = ::automsgs::msgs::geometry_msgs::Accel;
using AccelStamped = ::automsgs::msgs::geometry_msgs::AccelStamped;
using AccelWithCovariance = ::automsgs::msgs::geometry_msgs::AccelWithCovariance;
using AccelWithCovarianceStamped =
  ::automsgs::msgs::geometry_msgs::AccelWithCovarianceStamped;
using Inertia = ::automsgs::msgs::geometry_msgs::Inertia;
using InertiaStamped = ::automsgs::msgs::geometry_msgs::InertiaStamped;
using PointStamped = ::automsgs::msgs::geometry_msgs::PointStamped;
using Polygon = ::automsgs::msgs::geometry_msgs::Polygon;
using PolygonStamped = ::automsgs::msgs::geometry_msgs::PolygonStamped;
using Vector3Stamped = ::automsgs::msgs::geometry_msgs::Vector3Stamped;
using Wrench = ::automsgs::msgs::geometry_msgs::Wrench;
using WrenchStamped = ::automsgs::msgs::geometry_msgs::WrenchStamped;

// --- Primitives ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Vector3 and automsgs Vector3.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Vector3). Fields are copied without coordinate transforms.
 * @return automsgs Vector3 for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Vector3 from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Vector3 ready for rclcpp publish() or subscribe() adapters.
 */
Vector3 fromRos(const geometry_msgs::msg::Vector3 & from);
geometry_msgs::msg::Vector3 toRos(const Vector3 & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Point and automsgs Point.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Point). Fields are copied without coordinate transforms.
 * @return automsgs Point for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Point from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Point ready for rclcpp publish() or subscribe() adapters.
 */
Point fromRos(const geometry_msgs::msg::Point & from);
geometry_msgs::msg::Point toRos(const Point & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Point32 and automsgs Point32.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Point32). Fields are copied without coordinate transforms.
 * @return automsgs Point32 for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Point32 from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Point32 ready for rclcpp publish() or subscribe() adapters.
 */
Point32 fromRos(const geometry_msgs::msg::Point32 & from);
geometry_msgs::msg::Point32 toRos(const Point32 & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Quaternion and automsgs Quaternion.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Quaternion). Fields are copied without coordinate transforms.
 * @return automsgs Quaternion for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Quaternion from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Quaternion ready for rclcpp publish() or subscribe() adapters.
 */
Quaternion fromRos(const geometry_msgs::msg::Quaternion & from);
geometry_msgs::msg::Quaternion toRos(const Quaternion & from);

// --- Pose ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Pose and automsgs Pose.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Pose). Fields are copied without coordinate transforms.
 * @return automsgs Pose for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Pose from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Pose ready for rclcpp publish() or subscribe() adapters.
 */
Pose fromRos(const geometry_msgs::msg::Pose & from);
geometry_msgs::msg::Pose toRos(const Pose & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Pose2D and automsgs Pose2D.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Pose2D). Fields are copied without coordinate transforms.
 * @return automsgs Pose2D for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Pose2D from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Pose2D ready for rclcpp publish() or subscribe() adapters.
 */
Pose2D fromRos(const geometry_msgs::msg::Pose2D & from);
geometry_msgs::msg::Pose2D toRos(const Pose2D & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseArray and automsgs PoseArray.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseArray). Fields are copied without coordinate transforms.
 * @return automsgs PoseArray for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PoseArray from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseArray ready for rclcpp publish() or subscribe() adapters.
 */
PoseArray fromRos(const geometry_msgs::msg::PoseArray & from);
geometry_msgs::msg::PoseArray toRos(const PoseArray & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseStamped and automsgs PoseStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseStamped). Fields are copied without coordinate transforms.
 * @return automsgs PoseStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PoseStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseStamped ready for rclcpp publish() or subscribe() adapters.
 */
PoseStamped fromRos(const geometry_msgs::msg::PoseStamped & from);
geometry_msgs::msg::PoseStamped toRos(const PoseStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseWithCovariance and automsgs PoseWithCovariance.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseWithCovariance). Fields are copied without coordinate transforms.
 * @return automsgs PoseWithCovariance for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PoseWithCovariance from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseWithCovariance ready for rclcpp publish() or subscribe() adapters.
 */
PoseWithCovariance fromRos(const geometry_msgs::msg::PoseWithCovariance & from);
geometry_msgs::msg::PoseWithCovariance toRos(const PoseWithCovariance & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PoseWithCovarianceStamped and automsgs PoseWithCovarianceStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PoseWithCovarianceStamped). Fields are copied without coordinate transforms.
 * @return automsgs PoseWithCovarianceStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PoseWithCovarianceStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PoseWithCovarianceStamped ready for rclcpp publish() or subscribe() adapters.
 */
PoseWithCovarianceStamped fromRos(const geometry_msgs::msg::PoseWithCovarianceStamped & from);
geometry_msgs::msg::PoseWithCovarianceStamped toRos(const PoseWithCovarianceStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::QuaternionStamped and automsgs QuaternionStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::QuaternionStamped). Fields are copied without coordinate transforms.
 * @return automsgs QuaternionStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs QuaternionStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::QuaternionStamped ready for rclcpp publish() or subscribe() adapters.
 */
QuaternionStamped fromRos(const geometry_msgs::msg::QuaternionStamped & from);
geometry_msgs::msg::QuaternionStamped toRos(const QuaternionStamped & from);

// --- Transform ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Transform and automsgs Transform.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Transform). Fields are copied without coordinate transforms.
 * @return automsgs Transform for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Transform from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Transform ready for rclcpp publish() or subscribe() adapters.
 */
Transform fromRos(const geometry_msgs::msg::Transform & from);
geometry_msgs::msg::Transform toRos(const Transform & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TransformStamped and automsgs TransformStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TransformStamped). Fields are copied without coordinate transforms.
 * @return automsgs TransformStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs TransformStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TransformStamped ready for rclcpp publish() or subscribe() adapters.
 */
TransformStamped fromRos(const geometry_msgs::msg::TransformStamped & from);
geometry_msgs::msg::TransformStamped toRos(const TransformStamped & from);

// --- Velocity / acceleration ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Twist and automsgs Twist.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Twist). Fields are copied without coordinate transforms.
 * @return automsgs Twist for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Twist from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Twist ready for rclcpp publish() or subscribe() adapters.
 */
Twist fromRos(const geometry_msgs::msg::Twist & from);
geometry_msgs::msg::Twist toRos(const Twist & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TwistStamped and automsgs TwistStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TwistStamped). Fields are copied without coordinate transforms.
 * @return automsgs TwistStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs TwistStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TwistStamped ready for rclcpp publish() or subscribe() adapters.
 */
TwistStamped fromRos(const geometry_msgs::msg::TwistStamped & from);
geometry_msgs::msg::TwistStamped toRos(const TwistStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TwistWithCovariance and automsgs TwistWithCovariance.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TwistWithCovariance). Fields are copied without coordinate transforms.
 * @return automsgs TwistWithCovariance for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs TwistWithCovariance from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TwistWithCovariance ready for rclcpp publish() or subscribe() adapters.
 */
TwistWithCovariance fromRos(const geometry_msgs::msg::TwistWithCovariance & from);
geometry_msgs::msg::TwistWithCovariance toRos(const TwistWithCovariance & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::TwistWithCovarianceStamped and automsgs TwistWithCovarianceStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::TwistWithCovarianceStamped). Fields are copied without coordinate transforms.
 * @return automsgs TwistWithCovarianceStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs TwistWithCovarianceStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::TwistWithCovarianceStamped ready for rclcpp publish() or subscribe() adapters.
 */
TwistWithCovarianceStamped fromRos(const geometry_msgs::msg::TwistWithCovarianceStamped & from);
geometry_msgs::msg::TwistWithCovarianceStamped toRos(const TwistWithCovarianceStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Accel and automsgs Accel.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Accel). Fields are copied without coordinate transforms.
 * @return automsgs Accel for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Accel from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Accel ready for rclcpp publish() or subscribe() adapters.
 */
Accel fromRos(const geometry_msgs::msg::Accel & from);
geometry_msgs::msg::Accel toRos(const Accel & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::AccelStamped and automsgs AccelStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::AccelStamped). Fields are copied without coordinate transforms.
 * @return automsgs AccelStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs AccelStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::AccelStamped ready for rclcpp publish() or subscribe() adapters.
 */
AccelStamped fromRos(const geometry_msgs::msg::AccelStamped & from);
geometry_msgs::msg::AccelStamped toRos(const AccelStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::AccelWithCovariance and automsgs AccelWithCovariance.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::AccelWithCovariance). Fields are copied without coordinate transforms.
 * @return automsgs AccelWithCovariance for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs AccelWithCovariance from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::AccelWithCovariance ready for rclcpp publish() or subscribe() adapters.
 */
AccelWithCovariance fromRos(const geometry_msgs::msg::AccelWithCovariance & from);
geometry_msgs::msg::AccelWithCovariance toRos(const AccelWithCovariance & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::AccelWithCovarianceStamped and automsgs AccelWithCovarianceStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::AccelWithCovarianceStamped). Fields are copied without coordinate transforms.
 * @return automsgs AccelWithCovarianceStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs AccelWithCovarianceStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::AccelWithCovarianceStamped ready for rclcpp publish() or subscribe() adapters.
 */
AccelWithCovarianceStamped fromRos(const geometry_msgs::msg::AccelWithCovarianceStamped & from);
geometry_msgs::msg::AccelWithCovarianceStamped toRos(const AccelWithCovarianceStamped & from);

// --- Inertia / polygons / wrench ---

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Inertia and automsgs Inertia.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Inertia). Fields are copied without coordinate transforms.
 * @return automsgs Inertia for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Inertia from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Inertia ready for rclcpp publish() or subscribe() adapters.
 */
Inertia fromRos(const geometry_msgs::msg::Inertia & from);
geometry_msgs::msg::Inertia toRos(const Inertia & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::InertiaStamped and automsgs InertiaStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::InertiaStamped). Fields are copied without coordinate transforms.
 * @return automsgs InertiaStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs InertiaStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::InertiaStamped ready for rclcpp publish() or subscribe() adapters.
 */
InertiaStamped fromRos(const geometry_msgs::msg::InertiaStamped & from);
geometry_msgs::msg::InertiaStamped toRos(const InertiaStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PointStamped and automsgs PointStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PointStamped). Fields are copied without coordinate transforms.
 * @return automsgs PointStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PointStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PointStamped ready for rclcpp publish() or subscribe() adapters.
 */
PointStamped fromRos(const geometry_msgs::msg::PointStamped & from);
geometry_msgs::msg::PointStamped toRos(const PointStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Polygon and automsgs Polygon.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Polygon). Fields are copied without coordinate transforms.
 * @return automsgs Polygon for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Polygon from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Polygon ready for rclcpp publish() or subscribe() adapters.
 */
Polygon fromRos(const geometry_msgs::msg::Polygon & from);
geometry_msgs::msg::Polygon toRos(const Polygon & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::PolygonStamped and automsgs PolygonStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::PolygonStamped). Fields are copied without coordinate transforms.
 * @return automsgs PolygonStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs PolygonStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::PolygonStamped ready for rclcpp publish() or subscribe() adapters.
 */
PolygonStamped fromRos(const geometry_msgs::msg::PolygonStamped & from);
geometry_msgs::msg::PolygonStamped toRos(const PolygonStamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Vector3Stamped and automsgs Vector3Stamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Vector3Stamped). Fields are copied without coordinate transforms.
 * @return automsgs Vector3Stamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Vector3Stamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Vector3Stamped ready for rclcpp publish() or subscribe() adapters.
 */
Vector3Stamped fromRos(const geometry_msgs::msg::Vector3Stamped & from);
geometry_msgs::msg::Vector3Stamped toRos(const Vector3Stamped & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::Wrench and automsgs Wrench.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::Wrench). Fields are copied without coordinate transforms.
 * @return automsgs Wrench for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs Wrench from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::Wrench ready for rclcpp publish() or subscribe() adapters.
 */
Wrench fromRos(const geometry_msgs::msg::Wrench & from);
geometry_msgs::msg::Wrench toRos(const Wrench & from);

/**
 * @brief Bidirectional conversion between ROS geometry_msgs::msg::WrenchStamped and automsgs WrenchStamped.
 *
 * @par fromRos
 * @param from Input ROS message (geometry_msgs::msg::WrenchStamped). Fields are copied without coordinate transforms.
 * @return automsgs WrenchStamped for autonomy core APIs.
 *
 * @par toRos
 * @param from Input automsgs WrenchStamped from planners, bridges, or drivers.
 * @return ROS geometry_msgs::msg::WrenchStamped ready for rclcpp publish() or subscribe() adapters.
 */
WrenchStamped fromRos(const geometry_msgs::msg::WrenchStamped & from);
geometry_msgs::msg::WrenchStamped toRos(const WrenchStamped & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__GEOMETRY_MSGS_HPP_
