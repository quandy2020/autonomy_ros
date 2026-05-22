// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__STEREO_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__STEREO_MSGS_HPP_

/// @file stereo_msgs.hpp
/// @brief Converts ROS 2 stereo_msgs and autonomy::commsgs::stereo_msgs.
///
/// Proto schema: autonomy/commsgs/proto/stereo_msgs.proto
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) for disparity image
/// pipelines. Call fromRos() on incoming DisparityImage messages and toRos()
/// before publishing commsgs results.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/stereo_msgs.hpp"
/// auto disp = autonomy_ros::conversions::fromRos(*ros_disparity);
/// disparity_pub_->publish(autonomy_ros::conversions::toRos(core_disp));
/// @endcode

#include "autonomy/commsgs/stereo_msgs.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

namespace autonomy_ros::conversions
{

using DisparityImage = ::autonomy::commsgs::stereo_msgs::DisparityImage;

/**
 * @brief Bidirectional conversion between ROS stereo_msgs::msg::DisparityImage and commsgs DisparityImage.
 *
 * @par fromRos
 * @param from Input ROS message (stereo_msgs::msg::DisparityImage). Fields are copied without coordinate transforms.
 * @return commsgs DisparityImage for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs DisparityImage from planners, bridges, or drivers.
 * @return ROS stereo_msgs::msg::DisparityImage ready for rclcpp publish() or subscribe() adapters.
 */
DisparityImage fromRos(const stereo_msgs::msg::DisparityImage & from);
stereo_msgs::msg::DisparityImage toRos(const DisparityImage & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__STEREO_MSGS_HPP_
