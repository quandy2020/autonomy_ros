// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__VISION_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__VISION_MSGS_HPP_

/// @file vision_msgs.hpp
/// @brief Converts ROS 2 vision_msgs and autonomy::commsgs::vision_msgs.
///
/// Proto schema: autonomy/commsgs/proto/vision_msgs.proto
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) for 2D/3D detections,
/// bounding boxes, and perception metadata. Each type exposes fromRos() and
/// toRos() in autonomy_ros::conversions.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/vision_msgs.hpp"
/// auto dets = autonomy_ros::conversions::fromRos(*ros_detection_array);
/// det_pub_->publish(autonomy_ros::conversions::toRos(core_detections));
/// @endcode

#include "autonomy/commsgs/vision_msgs.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "vision_msgs/msg/bounding_box3_d_array.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/label_info.hpp"
#include "vision_msgs/msg/object_hypothesis.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "vision_msgs/msg/point2_d.hpp"
#include "vision_msgs/msg/pose2_d.hpp"
#include "vision_msgs/msg/vision_class.hpp"
#include "vision_msgs/msg/vision_info.hpp"

namespace autonomy_ros::conversions
{

using Point2D = ::autonomy::commsgs::vision_msgs::Point2D;
using VisionPose2D = ::autonomy::commsgs::vision_msgs::Pose2D;
using BoundingBox2D = ::autonomy::commsgs::vision_msgs::BoundingBox2D;
using BoundingBox2DArray = ::autonomy::commsgs::vision_msgs::BoundingBox2DArray;
using BoundingBox3D = ::autonomy::commsgs::vision_msgs::BoundingBox3D;
using BoundingBox3DArray = ::autonomy::commsgs::vision_msgs::BoundingBox3DArray;
using ObjectHypothesis = ::autonomy::commsgs::vision_msgs::ObjectHypothesis;
using Classification = ::autonomy::commsgs::vision_msgs::Classification;
using ObjectHypothesisWithPose =
  ::autonomy::commsgs::vision_msgs::ObjectHypothesisWithPose;
using Detection2D = ::autonomy::commsgs::vision_msgs::Detection2D;
using Detection2DArray = ::autonomy::commsgs::vision_msgs::Detection2DArray;
using Detection3D = ::autonomy::commsgs::vision_msgs::Detection3D;
using Detection3DArray = ::autonomy::commsgs::vision_msgs::Detection3DArray;
using VisionClass = ::autonomy::commsgs::vision_msgs::VisionClass;
using LabelInfo = ::autonomy::commsgs::vision_msgs::LabelInfo;
using VisionInfo = ::autonomy::commsgs::vision_msgs::VisionInfo;

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::Point2D and commsgs Point2D.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::Point2D). Fields are copied without coordinate transforms.
 * @return commsgs Point2D for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Point2D from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::Point2D ready for rclcpp publish() or subscribe() adapters.
 */
Point2D fromRos(const vision_msgs::msg::Point2D & from);
vision_msgs::msg::Point2D toRos(const Point2D & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::Pose2D and commsgs Pose2D.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::Pose2D). Fields are copied without coordinate transforms.
 * @return commsgs VisionPose2D for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs VisionPose2D from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::Pose2D ready for rclcpp publish() or subscribe() adapters.
 */
VisionPose2D fromRos(const vision_msgs::msg::Pose2D & from);
vision_msgs::msg::Pose2D toRos(const VisionPose2D & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::BoundingBox2D and commsgs BoundingBox2D.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::BoundingBox2D). Fields are copied without coordinate transforms.
 * @return commsgs BoundingBox2D for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs BoundingBox2D from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::BoundingBox2D ready for rclcpp publish() or subscribe() adapters.
 */
BoundingBox2D fromRos(const vision_msgs::msg::BoundingBox2D & from);
vision_msgs::msg::BoundingBox2D toRos(const BoundingBox2D & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::BoundingBox2DArray and commsgs BoundingBox2DArray.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::BoundingBox2DArray). Fields are copied without coordinate transforms.
 * @return commsgs BoundingBox2DArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs BoundingBox2DArray from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::BoundingBox2DArray ready for rclcpp publish() or subscribe() adapters.
 */
BoundingBox2DArray fromRos(const vision_msgs::msg::BoundingBox2DArray & from);
vision_msgs::msg::BoundingBox2DArray toRos(const BoundingBox2DArray & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::BoundingBox3D and commsgs BoundingBox3D.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::BoundingBox3D). Fields are copied without coordinate transforms.
 * @return commsgs BoundingBox3D for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs BoundingBox3D from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::BoundingBox3D ready for rclcpp publish() or subscribe() adapters.
 */
BoundingBox3D fromRos(const vision_msgs::msg::BoundingBox3D & from);
vision_msgs::msg::BoundingBox3D toRos(const BoundingBox3D & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::BoundingBox3DArray and commsgs BoundingBox3DArray.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::BoundingBox3DArray). Fields are copied without coordinate transforms.
 * @return commsgs BoundingBox3DArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs BoundingBox3DArray from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::BoundingBox3DArray ready for rclcpp publish() or subscribe() adapters.
 */
BoundingBox3DArray fromRos(const vision_msgs::msg::BoundingBox3DArray & from);
vision_msgs::msg::BoundingBox3DArray toRos(const BoundingBox3DArray & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::ObjectHypothesis and commsgs ObjectHypothesis.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::ObjectHypothesis). Fields are copied without coordinate transforms.
 * @return commsgs ObjectHypothesis for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs ObjectHypothesis from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::ObjectHypothesis ready for rclcpp publish() or subscribe() adapters.
 */
ObjectHypothesis fromRos(const vision_msgs::msg::ObjectHypothesis & from);
vision_msgs::msg::ObjectHypothesis toRos(const ObjectHypothesis & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::Classification and commsgs Classification.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::Classification). Fields are copied without coordinate transforms.
 * @return commsgs Classification for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Classification from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::Classification ready for rclcpp publish() or subscribe() adapters.
 */
Classification fromRos(const vision_msgs::msg::Classification & from);
vision_msgs::msg::Classification toRos(const Classification & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::ObjectHypothesisWithPose and commsgs ObjectHypothesisWithPose.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::ObjectHypothesisWithPose). Fields are copied without coordinate transforms.
 * @return commsgs ObjectHypothesisWithPose for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs ObjectHypothesisWithPose from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::ObjectHypothesisWithPose ready for rclcpp publish() or subscribe() adapters.
 */
ObjectHypothesisWithPose fromRos(const vision_msgs::msg::ObjectHypothesisWithPose & from);
vision_msgs::msg::ObjectHypothesisWithPose toRos(const ObjectHypothesisWithPose & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::Detection2D and commsgs Detection2D.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::Detection2D). Fields are copied without coordinate transforms.
 * @return commsgs Detection2D for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Detection2D from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::Detection2D ready for rclcpp publish() or subscribe() adapters.
 */
Detection2D fromRos(const vision_msgs::msg::Detection2D & from);
vision_msgs::msg::Detection2D toRos(const Detection2D & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::Detection2DArray and commsgs Detection2DArray.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::Detection2DArray). Fields are copied without coordinate transforms.
 * @return commsgs Detection2DArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Detection2DArray from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::Detection2DArray ready for rclcpp publish() or subscribe() adapters.
 */
Detection2DArray fromRos(const vision_msgs::msg::Detection2DArray & from);
vision_msgs::msg::Detection2DArray toRos(const Detection2DArray & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::Detection3D and commsgs Detection3D.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::Detection3D). Fields are copied without coordinate transforms.
 * @return commsgs Detection3D for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Detection3D from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::Detection3D ready for rclcpp publish() or subscribe() adapters.
 */
Detection3D fromRos(const vision_msgs::msg::Detection3D & from);
vision_msgs::msg::Detection3D toRos(const Detection3D & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::Detection3DArray and commsgs Detection3DArray.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::Detection3DArray). Fields are copied without coordinate transforms.
 * @return commsgs Detection3DArray for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Detection3DArray from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::Detection3DArray ready for rclcpp publish() or subscribe() adapters.
 */
Detection3DArray fromRos(const vision_msgs::msg::Detection3DArray & from);
vision_msgs::msg::Detection3DArray toRos(const Detection3DArray & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::VisionClass and commsgs VisionClass.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::VisionClass). Fields are copied without coordinate transforms.
 * @return commsgs VisionClass for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs VisionClass from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::VisionClass ready for rclcpp publish() or subscribe() adapters.
 */
VisionClass fromRos(const vision_msgs::msg::VisionClass & from);
vision_msgs::msg::VisionClass toRos(const VisionClass & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::LabelInfo and commsgs LabelInfo.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::LabelInfo). Fields are copied without coordinate transforms.
 * @return commsgs LabelInfo for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs LabelInfo from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::LabelInfo ready for rclcpp publish() or subscribe() adapters.
 */
LabelInfo fromRos(const vision_msgs::msg::LabelInfo & from);
vision_msgs::msg::LabelInfo toRos(const LabelInfo & from);

/**
 * @brief Bidirectional conversion between ROS vision_msgs::msg::VisionInfo and commsgs VisionInfo.
 *
 * @par fromRos
 * @param from Input ROS message (vision_msgs::msg::VisionInfo). Fields are copied without coordinate transforms.
 * @return commsgs VisionInfo for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs VisionInfo from planners, bridges, or drivers.
 * @return ROS vision_msgs::msg::VisionInfo ready for rclcpp publish() or subscribe() adapters.
 */
VisionInfo fromRos(const vision_msgs::msg::VisionInfo & from);
vision_msgs::msg::VisionInfo toRos(const VisionInfo & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__VISION_MSGS_HPP_
