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


// autonomy::comsmgs::geometry_msgs
#include "autonomy/commsgs/geometry_msgs.hpp"

//ros geometry_msgs
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
#include "geometry_msgs/msg/velocity_stamped.hpp"


namespace autonomy_ros {

// Accel
geometry_msgs::msg::Accel ToRos(const ::autonomy::commsgs::geometry_msgs::Accel& data);
geometry_msgs::msg::AccelStamped ToRos(const ::autonomy::commsgs::geometry_msgs::AccelStamped& data);
geometry_msgs::msg::AccelWithCovariance ToRos(const ::autonomy::commsgs::geometry_msgs::AccelWithCovariance& data);
geometry_msgs::msg::AccelWithCovarianceStamped ToRos(const ::autonomy::commsgs::geometry_msgs::AccelWithCovarianceStamped& data);

// Inertia
geometry_msgs::msg::Inertia ToRos(const ::autonomy::commsgs::geometry_msgs::Inertia& data);
geometry_msgs::msg::InertiaStamped ToRos(const ::autonomy::commsgs::geometry_msgs::InertiaStamped& data);

// Point
geometry_msgs::msg::Point ToRos(const ::autonomy::commsgs::geometry_msgs::Point& data);
::autonomy::commsgs::geometry_msgs::Point FromRos(const geometry_msgs::msg::Point& ros);

// Point32
geometry_msgs::msg::Point32 ToRos(const ::autonomy::commsgs::geometry_msgs::Point32& data);
::autonomy::commsgs::geometry_msgs::Point32 FromRos(const geometry_msgs::msg::Point32& ros);

// PointStamped
geometry_msgs::msg::PointStamped ToRos(const ::autonomy::commsgs::geometry_msgs::PointStamped& data);
::autonomy::commsgs::geometry_msgs::PointStamped FromRos(const geometry_msgs::msg::PointStamped& ros);

// polygon
geometry_msgs::msg::Polygon ToRos(const ::autonomy::commsgs::geometry_msgs::Polygon& data);
geometry_msgs::msg::PolygonStamped ToRos(const ::autonomy::commsgs::geometry_msgs::PolygonStamped& data);

// Pose
geometry_msgs::msg::Pose ToRos(const ::autonomy::commsgs::geometry_msgs::Pose& data);
geometry_msgs::msg::Pose2D ToRos(const ::autonomy::commsgs::geometry_msgs::Pose2D& data);
geometry_msgs::msg::PoseArray ToRos(const ::autonomy::commsgs::geometry_msgs::PoseArray& data);
geometry_msgs::msg::PoseStamped ToRos(const ::autonomy::commsgs::geometry_msgs::PoseStamped& data);
geometry_msgs::msg::PoseWithCovariance ToRos(const ::autonomy::commsgs::geometry_msgs::PoseWithCovariance& data);
geometry_msgs::msg::PoseWithCovarianceStamped ToRos(const ::autonomy::commsgs::geometry_msgs::PoseWithCovarianceStamped& data);

// Quaternion
geometry_msgs::msg::Quaternion ToRos(const ::autonomy::commsgs::geometry_msgs::Quaternion& data);
geometry_msgs::msg::QuaternionStamped ToRos(const ::autonomy::commsgs::geometry_msgs::QuaternionStamped& data);

// transform
geometry_msgs::msg::Transform ToRos(const ::autonomy::commsgs::geometry_msgs::Transform& data);
geometry_msgs::msg::TransformStamped ToRos(const ::autonomy::commsgs::geometry_msgs::TransformStamped& data);

// Twist
geometry_msgs::msg::Twist ToRos(const ::autonomy::commsgs::geometry_msgs::Twist& data);
geometry_msgs::msg::TwistStamped ToRos(const ::autonomy::commsgs::geometry_msgs::TwistStamped& data);
geometry_msgs::msg::TwistWithCovariance ToRos(const ::autonomy::commsgs::geometry_msgs::TwistWithCovariance& data);
geometry_msgs::msg::TwistWithCovarianceStamped ToRos(const ::autonomy::commsgs::geometry_msgs::TwistWithCovarianceStamped& data);

// Vector3
geometry_msgs::msg::Vector3 ToRos(const ::autonomy::commsgs::geometry_msgs::Vector3& data);
geometry_msgs::msg::Vector3Stamped ToRos(const ::autonomy::commsgs::geometry_msgs::Vector3Stamped& data);
geometry_msgs::msg::VelocityStamped ToRos(const ::autonomy::commsgs::geometry_msgs::VelocityStamped& data);

}  // namespace autonomy_ros

