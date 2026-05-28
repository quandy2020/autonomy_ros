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

#ifndef AUTONOMY_ROS__CONVERSIONS__STD_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__STD_MSGS_HPP_

/// @file std_msgs.hpp
/// @brief Converts ROS 2 std_msgs and autonomy::commsgs::std_msgs types.
///
/// Proto schema: autonomy/commsgs/proto/std_msgs.proto
///
/// @par Overview
/// Header.stamp is converted via builtin_interfaces. These types are reused by
/// geometry_msgs, sensor_msgs, map_msgs, and other conversion modules.
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) and call overloads in
/// autonomy_ros::conversions:
/// - fromRos(ros_msg) copies into commsgs structs (return by value).
/// - toRos(commsgs_msg) produces ROS messages for publishers/subscribers.
///
/// @par Subscribe example
/// @code
/// #include "autonomy_ros/conversions/std_msgs.hpp"
/// void cb(const std_msgs::msg::Header::SharedPtr msg) {
///   auto h = autonomy_ros::fromRos(*msg);
/// }
/// @endcode
///
/// @par Publish example
/// @code
/// std_msgs::msg::String out = autonomy_ros::toRos(core_string);
/// pub->publish(out);
/// @endcode

#include "autonomy/commsgs/std_msgs.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/string.hpp"

namespace autonomy_ros
{

using Header = ::autonomy::commsgs::std_msgs::Header;
using ColorRGBA = ::autonomy::commsgs::std_msgs::ColorRGBA;
using MultiArrayDimension = ::autonomy::commsgs::std_msgs::MultiArrayDimension;
using MultiArrayLayout = ::autonomy::commsgs::std_msgs::MultiArrayLayout;
using Float32MultiArray = ::autonomy::commsgs::std_msgs::Float32MultiArray;
using String = ::autonomy::commsgs::std_msgs::String;

/**
 * @brief Bidirectional conversion between ROS std_msgs::msg::Header and commsgs Header.
 *
 * @par fromRos
 * @param from Input ROS message (std_msgs::msg::Header). Fields are copied without coordinate transforms.
 * @return commsgs Header for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Header from planners, bridges, or drivers.
 * @return ROS std_msgs::msg::Header ready for rclcpp publish() or subscribe() adapters.
 */
Header fromRos(const std_msgs::msg::Header & from);
std_msgs::msg::Header toRos(const Header & from);

/**
 * @brief Bidirectional conversion between ROS std_msgs::msg::ColorRGBA and commsgs ColorRGBA.
 *
 * @par fromRos
 * @param from Input ROS message (std_msgs::msg::ColorRGBA). Fields are copied without coordinate transforms.
 * @return commsgs ColorRGBA for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs ColorRGBA from planners, bridges, or drivers.
 * @return ROS std_msgs::msg::ColorRGBA ready for rclcpp publish() or subscribe() adapters.
 */
ColorRGBA fromRos(const std_msgs::msg::ColorRGBA & from);
std_msgs::msg::ColorRGBA toRos(const ColorRGBA & from);

/**
 * @brief Bidirectional conversion between ROS std_msgs::msg::MultiArrayDimension and commsgs MultiArrayDimension.
 *
 * @par fromRos
 * @param from Input ROS message (std_msgs::msg::MultiArrayDimension). Fields are copied without coordinate transforms.
 * @return commsgs MultiArrayDimension for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MultiArrayDimension from planners, bridges, or drivers.
 * @return ROS std_msgs::msg::MultiArrayDimension ready for rclcpp publish() or subscribe() adapters.
 */
MultiArrayDimension fromRos(const std_msgs::msg::MultiArrayDimension & from);
std_msgs::msg::MultiArrayDimension toRos(const MultiArrayDimension & from);

/**
 * @brief Bidirectional conversion between ROS std_msgs::msg::MultiArrayLayout and commsgs MultiArrayLayout.
 *
 * @par fromRos
 * @param from Input ROS message (std_msgs::msg::MultiArrayLayout). Fields are copied without coordinate transforms.
 * @return commsgs MultiArrayLayout for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MultiArrayLayout from planners, bridges, or drivers.
 * @return ROS std_msgs::msg::MultiArrayLayout ready for rclcpp publish() or subscribe() adapters.
 */
MultiArrayLayout fromRos(const std_msgs::msg::MultiArrayLayout & from);
std_msgs::msg::MultiArrayLayout toRos(const MultiArrayLayout & from);

/**
 * @brief Bidirectional conversion between ROS std_msgs::msg::Float32MultiArray and commsgs Float32MultiArray.
 *
 * @par fromRos
 * @param from Input ROS message (std_msgs::msg::Float32MultiArray). Fields are copied without coordinate transforms.
 * @return commsgs Float32MultiArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Float32MultiArray from planners, bridges, or drivers.
 * @return ROS std_msgs::msg::Float32MultiArray ready for rclcpp publish() or subscribe() adapters.
 */
Float32MultiArray fromRos(const std_msgs::msg::Float32MultiArray & from);
std_msgs::msg::Float32MultiArray toRos(const Float32MultiArray & from);

/**
 * @brief Bidirectional conversion between ROS std_msgs::msg::String and commsgs String.
 *
 * @par fromRos
 * @param from Input ROS message (std_msgs::msg::String). Fields are copied without coordinate transforms.
 * @return commsgs String for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs String from planners, bridges, or drivers.
 * @return ROS std_msgs::msg::String ready for rclcpp publish() or subscribe() adapters.
 */
String fromRos(const std_msgs::msg::String & from);
std_msgs::msg::String toRos(const String & from);

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__CONVERSIONS__STD_MSGS_HPP_
