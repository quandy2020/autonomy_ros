// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__TF2_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__TF2_MSGS_HPP_

/// @file tf2_msgs.hpp
/// @brief Converts tf2_msgs/TFMessage and commsgs TransformStampeds.
///
/// Commsgs type: geometry_msgs::TransformStampeds (geometry_msgs.proto).
/// Single transforms use geometry_msgs.hpp (TransformStamped).
///
/// @par Usage
/// Use when bridging /tf or /tf_static topic batches:
/// - fromRos(TFMessage) to feed autonomy core with a vector of transforms.
/// - toRos(TransformStampeds) to publish a TFMessage from commsgs data.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/tf2_msgs.hpp"
/// void on_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
///   auto stamped = autonomy_ros::conversions::fromRos(*msg);
/// }
/// tf_pub_->publish(autonomy_ros::conversions::toRos(core_transforms));
/// @endcode

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace autonomy_ros::conversions
{

using TransformStampeds = ::autonomy::commsgs::geometry_msgs::TransformStampeds;

/**
 * @brief Bidirectional conversion between ROS tf2_msgs::msg::TFMessage and commsgs TransformStampeds.
 *
 * @par fromRos
 * @param from Input ROS message (tf2_msgs::msg::TFMessage). Fields are copied without coordinate transforms.
 * @return commsgs TransformStampeds for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs TransformStampeds from planners, bridges, or drivers.
 * @return ROS tf2_msgs::msg::TFMessage ready for rclcpp publish() or subscribe() adapters.
 */
TransformStampeds fromRos(const tf2_msgs::msg::TFMessage & from);
tf2_msgs::msg::TFMessage toRos(const TransformStampeds & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__TF2_MSGS_HPP_
