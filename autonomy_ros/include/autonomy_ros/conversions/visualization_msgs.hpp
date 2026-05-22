// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__VISUALIZATION_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__VISUALIZATION_MSGS_HPP_

/// @file visualization_msgs.hpp
/// @brief Converts ROS 2 visualization_msgs and commsgs visualization types.
///
/// Proto schema: autonomy/commsgs/proto/visualization_msgs.proto
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) for RViz markers and
/// interactive markers. Convert with fromRos() on incoming feedback/init messages
/// and toRos() before publishing MarkerArray or InteractiveMarker updates.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/visualization_msgs.hpp"
/// marker_pub_->publish(autonomy_ros::conversions::toRos(core_markers));
/// @endcode

#include "autonomy/commsgs/visualization_msgs.hpp"
#include "visualization_msgs/msg/image_marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/interactive_marker_init.hpp"
#include "visualization_msgs/msg/interactive_marker_pose.hpp"
#include "visualization_msgs/msg/interactive_marker_update.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/menu_entry.hpp"
#include "visualization_msgs/msg/mesh_file.hpp"
#include "visualization_msgs/msg/uv_coordinate.hpp"

namespace autonomy_ros::conversions
{

using MenuEntry = ::autonomy::commsgs::visualization_msgs::MenuEntry;
using MeshFile = ::autonomy::commsgs::visualization_msgs::MeshFile;
using UVCoordinate = ::autonomy::commsgs::visualization_msgs::UVCoordinate;
using ImageMarker = ::autonomy::commsgs::visualization_msgs::ImageMarker;
using Marker = ::autonomy::commsgs::visualization_msgs::Marker;
using MarkerArray = ::autonomy::commsgs::visualization_msgs::MarkerArray;
using InteractiveMarkerControl =
  ::autonomy::commsgs::visualization_msgs::InteractiveMarkerControl;
using InteractiveMarkerFeedback =
  ::autonomy::commsgs::visualization_msgs::InteractiveMarkerFeedback;
using InteractiveMarker = ::autonomy::commsgs::visualization_msgs::InteractiveMarker;
using InteractiveMarkerInit =
  ::autonomy::commsgs::visualization_msgs::InteractiveMarkerInit;
using InteractiveMarkerPose =
  ::autonomy::commsgs::visualization_msgs::InteractiveMarkerPose;
using InteractiveMarkerUpdate =
  ::autonomy::commsgs::visualization_msgs::InteractiveMarkerUpdate;

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::MenuEntry and commsgs MenuEntry.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::MenuEntry). Fields are copied without coordinate transforms.
 * @return commsgs MenuEntry for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MenuEntry from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::MenuEntry ready for rclcpp publish() or subscribe() adapters.
 */
MenuEntry fromRos(const visualization_msgs::msg::MenuEntry & from);
visualization_msgs::msg::MenuEntry toRos(const MenuEntry & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::MeshFile and commsgs MeshFile.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::MeshFile). Fields are copied without coordinate transforms.
 * @return commsgs MeshFile for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MeshFile from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::MeshFile ready for rclcpp publish() or subscribe() adapters.
 */
MeshFile fromRos(const visualization_msgs::msg::MeshFile & from);
visualization_msgs::msg::MeshFile toRos(const MeshFile & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::UVCoordinate and commsgs UVCoordinate.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::UVCoordinate). Fields are copied without coordinate transforms.
 * @return commsgs UVCoordinate for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs UVCoordinate from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::UVCoordinate ready for rclcpp publish() or subscribe() adapters.
 */
UVCoordinate fromRos(const visualization_msgs::msg::UVCoordinate & from);
visualization_msgs::msg::UVCoordinate toRos(const UVCoordinate & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::ImageMarker and commsgs ImageMarker.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::ImageMarker). Fields are copied without coordinate transforms.
 * @return commsgs ImageMarker for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs ImageMarker from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::ImageMarker ready for rclcpp publish() or subscribe() adapters.
 */
ImageMarker fromRos(const visualization_msgs::msg::ImageMarker & from);
visualization_msgs::msg::ImageMarker toRos(const ImageMarker & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::Marker and commsgs Marker.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::Marker). Fields are copied without coordinate transforms.
 * @return commsgs Marker for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Marker from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::Marker ready for rclcpp publish() or subscribe() adapters.
 */
Marker fromRos(const visualization_msgs::msg::Marker & from);
visualization_msgs::msg::Marker toRos(const Marker & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::MarkerArray and commsgs MarkerArray.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::MarkerArray). Fields are copied without coordinate transforms.
 * @return commsgs MarkerArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MarkerArray from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::MarkerArray ready for rclcpp publish() or subscribe() adapters.
 */
MarkerArray fromRos(const visualization_msgs::msg::MarkerArray & from);
visualization_msgs::msg::MarkerArray toRos(const MarkerArray & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::InteractiveMarkerControl and commsgs InteractiveMarkerControl.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::InteractiveMarkerControl). Fields are copied without coordinate transforms.
 * @return commsgs InteractiveMarkerControl for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs InteractiveMarkerControl from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::InteractiveMarkerControl ready for rclcpp publish() or subscribe() adapters.
 */
InteractiveMarkerControl fromRos(const visualization_msgs::msg::InteractiveMarkerControl & from);
visualization_msgs::msg::InteractiveMarkerControl toRos(const InteractiveMarkerControl & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::InteractiveMarkerFeedback and commsgs InteractiveMarkerFeedback.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::InteractiveMarkerFeedback). Fields are copied without coordinate transforms.
 * @return commsgs InteractiveMarkerFeedback for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs InteractiveMarkerFeedback from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::InteractiveMarkerFeedback ready for rclcpp publish() or subscribe() adapters.
 */
InteractiveMarkerFeedback fromRos(const visualization_msgs::msg::InteractiveMarkerFeedback & from);
visualization_msgs::msg::InteractiveMarkerFeedback toRos(const InteractiveMarkerFeedback & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::InteractiveMarker and commsgs InteractiveMarker.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::InteractiveMarker). Fields are copied without coordinate transforms.
 * @return commsgs InteractiveMarker for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs InteractiveMarker from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::InteractiveMarker ready for rclcpp publish() or subscribe() adapters.
 */
InteractiveMarker fromRos(const visualization_msgs::msg::InteractiveMarker & from);
visualization_msgs::msg::InteractiveMarker toRos(const InteractiveMarker & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::InteractiveMarkerInit and commsgs InteractiveMarkerInit.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::InteractiveMarkerInit). Fields are copied without coordinate transforms.
 * @return commsgs InteractiveMarkerInit for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs InteractiveMarkerInit from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::InteractiveMarkerInit ready for rclcpp publish() or subscribe() adapters.
 */
InteractiveMarkerInit fromRos(const visualization_msgs::msg::InteractiveMarkerInit & from);
visualization_msgs::msg::InteractiveMarkerInit toRos(const InteractiveMarkerInit & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::InteractiveMarkerPose and commsgs InteractiveMarkerPose.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::InteractiveMarkerPose). Fields are copied without coordinate transforms.
 * @return commsgs InteractiveMarkerPose for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs InteractiveMarkerPose from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::InteractiveMarkerPose ready for rclcpp publish() or subscribe() adapters.
 */
InteractiveMarkerPose fromRos(const visualization_msgs::msg::InteractiveMarkerPose & from);
visualization_msgs::msg::InteractiveMarkerPose toRos(const InteractiveMarkerPose & from);

/**
 * @brief Bidirectional conversion between ROS visualization_msgs::msg::InteractiveMarkerUpdate and commsgs InteractiveMarkerUpdate.
 *
 * @par fromRos
 * @param from Input ROS message (visualization_msgs::msg::InteractiveMarkerUpdate). Fields are copied without coordinate transforms.
 * @return commsgs InteractiveMarkerUpdate for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs InteractiveMarkerUpdate from planners, bridges, or drivers.
 * @return ROS visualization_msgs::msg::InteractiveMarkerUpdate ready for rclcpp publish() or subscribe() adapters.
 */
InteractiveMarkerUpdate fromRos(const visualization_msgs::msg::InteractiveMarkerUpdate & from);
visualization_msgs::msg::InteractiveMarkerUpdate toRos(const InteractiveMarkerUpdate & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__VISUALIZATION_MSGS_HPP_
