// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/conversions/vision_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"

namespace autonomy_ros::conversions
{

Point2D fromRos(const vision_msgs::msg::Point2D & from)
{
  Point2D to;
  to.x = from.x;
  to.y = from.y;
  return to;
}

vision_msgs::msg::Point2D toRos(const Point2D & from)
{
  vision_msgs::msg::Point2D to;
  to.x = from.x;
  to.y = from.y;
  return to;
}

VisionPose2D fromRos(const vision_msgs::msg::Pose2D & from)
{
  VisionPose2D to;
  to.position = fromRos(from.position);
  to.theta = from.theta;
  return to;
}

vision_msgs::msg::Pose2D toRos(const VisionPose2D & from)
{
  vision_msgs::msg::Pose2D to;
  to.position = toRos(from.position);
  to.theta = from.theta;
  return to;
}

BoundingBox2D fromRos(const vision_msgs::msg::BoundingBox2D & from)
{
  BoundingBox2D to;
  to.center = fromRos(from.center);
  to.size_x = from.size_x;
  to.size_y = from.size_y;
  return to;
}

vision_msgs::msg::BoundingBox2D toRos(const BoundingBox2D & from)
{
  vision_msgs::msg::BoundingBox2D to;
  to.center = toRos(from.center);
  to.size_x = from.size_x;
  to.size_y = from.size_y;
  return to;
}

BoundingBox2DArray fromRos(const vision_msgs::msg::BoundingBox2DArray & from)
{
  BoundingBox2DArray to;
  detail::copyHeader(from.header, to.header);
  to.boxes.reserve(from.boxes.size());
  for (const auto & box : from.boxes) {
    to.boxes.push_back(fromRos(box));
  }
  return to;
}

vision_msgs::msg::BoundingBox2DArray toRos(const BoundingBox2DArray & from)
{
  vision_msgs::msg::BoundingBox2DArray to;
  detail::copyHeader(from.header, to.header);
  to.boxes.reserve(from.boxes.size());
  for (const auto & box : from.boxes) {
    to.boxes.push_back(toRos(box));
  }
  return to;
}

BoundingBox3D fromRos(const vision_msgs::msg::BoundingBox3D & from)
{
  BoundingBox3D to;
  to.center = autonomy_ros::conversions::fromRos(from.center);
  to.size = autonomy_ros::conversions::fromRos(from.size);
  return to;
}

vision_msgs::msg::BoundingBox3D toRos(const BoundingBox3D & from)
{
  vision_msgs::msg::BoundingBox3D to;
  to.center = autonomy_ros::conversions::toRos(from.center);
  to.size = autonomy_ros::conversions::toRos(from.size);
  return to;
}

BoundingBox3DArray fromRos(const vision_msgs::msg::BoundingBox3DArray & from)
{
  BoundingBox3DArray to;
  detail::copyHeader(from.header, to.header);
  to.boxes.reserve(from.boxes.size());
  for (const auto & box : from.boxes) {
    to.boxes.push_back(fromRos(box));
  }
  return to;
}

vision_msgs::msg::BoundingBox3DArray toRos(const BoundingBox3DArray & from)
{
  vision_msgs::msg::BoundingBox3DArray to;
  detail::copyHeader(from.header, to.header);
  to.boxes.reserve(from.boxes.size());
  for (const auto & box : from.boxes) {
    to.boxes.push_back(toRos(box));
  }
  return to;
}

ObjectHypothesis fromRos(const vision_msgs::msg::ObjectHypothesis & from)
{
  ObjectHypothesis to;
  to.class_id = from.class_id;
  to.score = from.score;
  return to;
}

vision_msgs::msg::ObjectHypothesis toRos(const ObjectHypothesis & from)
{
  vision_msgs::msg::ObjectHypothesis to;
  to.class_id = from.class_id;
  to.score = from.score;
  return to;
}

Classification fromRos(const vision_msgs::msg::Classification & from)
{
  Classification to;
  detail::copyHeader(from.header, to.header);
  to.results.reserve(from.results.size());
  for (const auto & r : from.results) {
    to.results.push_back(fromRos(r));
  }
  return to;
}

vision_msgs::msg::Classification toRos(const Classification & from)
{
  vision_msgs::msg::Classification to;
  detail::copyHeader(from.header, to.header);
  to.results.reserve(from.results.size());
  for (const auto & r : from.results) {
    to.results.push_back(toRos(r));
  }
  return to;
}

ObjectHypothesisWithPose fromRos(
  const vision_msgs::msg::ObjectHypothesisWithPose & from)
{
  ObjectHypothesisWithPose to;
  to.hypothesis = fromRos(from.hypothesis);
  to.pose = autonomy_ros::conversions::fromRos(from.pose);
  return to;
}

vision_msgs::msg::ObjectHypothesisWithPose toRos(
  const ObjectHypothesisWithPose & from)
{
  vision_msgs::msg::ObjectHypothesisWithPose to;
  to.hypothesis = toRos(from.hypothesis);
  to.pose = autonomy_ros::conversions::toRos(from.pose);
  return to;
}

Detection2D fromRos(const vision_msgs::msg::Detection2D & from)
{
  Detection2D to;
  detail::copyHeader(from.header, to.header);
  to.results.reserve(from.results.size());
  for (const auto & r : from.results) {
    to.results.push_back(fromRos(r));
  }
  to.bbox = fromRos(from.bbox);
  to.id = from.id;
  return to;
}

vision_msgs::msg::Detection2D toRos(const Detection2D & from)
{
  vision_msgs::msg::Detection2D to;
  detail::copyHeader(from.header, to.header);
  to.results.reserve(from.results.size());
  for (const auto & r : from.results) {
    to.results.push_back(toRos(r));
  }
  to.bbox = toRos(from.bbox);
  to.id = from.id;
  return to;
}

Detection2DArray fromRos(const vision_msgs::msg::Detection2DArray & from)
{
  Detection2DArray to;
  detail::copyHeader(from.header, to.header);
  to.detections.reserve(from.detections.size());
  for (const auto & d : from.detections) {
    to.detections.push_back(fromRos(d));
  }
  return to;
}

vision_msgs::msg::Detection2DArray toRos(const Detection2DArray & from)
{
  vision_msgs::msg::Detection2DArray to;
  detail::copyHeader(from.header, to.header);
  to.detections.reserve(from.detections.size());
  for (const auto & d : from.detections) {
    to.detections.push_back(toRos(d));
  }
  return to;
}

Detection3D fromRos(const vision_msgs::msg::Detection3D & from)
{
  Detection3D to;
  detail::copyHeader(from.header, to.header);
  to.results.reserve(from.results.size());
  for (const auto & r : from.results) {
    to.results.push_back(fromRos(r));
  }
  to.bbox = fromRos(from.bbox);
  to.id = from.id;
  return to;
}

vision_msgs::msg::Detection3D toRos(const Detection3D & from)
{
  vision_msgs::msg::Detection3D to;
  detail::copyHeader(from.header, to.header);
  to.results.reserve(from.results.size());
  for (const auto & r : from.results) {
    to.results.push_back(toRos(r));
  }
  to.bbox = toRos(from.bbox);
  to.id = from.id;
  return to;
}

Detection3DArray fromRos(const vision_msgs::msg::Detection3DArray & from)
{
  Detection3DArray to;
  detail::copyHeader(from.header, to.header);
  to.detections.reserve(from.detections.size());
  for (const auto & d : from.detections) {
    to.detections.push_back(fromRos(d));
  }
  return to;
}

vision_msgs::msg::Detection3DArray toRos(const Detection3DArray & from)
{
  vision_msgs::msg::Detection3DArray to;
  detail::copyHeader(from.header, to.header);
  to.detections.reserve(from.detections.size());
  for (const auto & d : from.detections) {
    to.detections.push_back(toRos(d));
  }
  return to;
}

VisionClass fromRos(const vision_msgs::msg::VisionClass & from)
{
  VisionClass to;
  to.class_id = from.class_id;
  to.class_name = from.class_name;
  return to;
}

vision_msgs::msg::VisionClass toRos(const VisionClass & from)
{
  vision_msgs::msg::VisionClass to;
  to.class_id = from.class_id;
  to.class_name = from.class_name;
  return to;
}

LabelInfo fromRos(const vision_msgs::msg::LabelInfo & from)
{
  LabelInfo to;
  detail::copyHeader(from.header, to.header);
  to.class_map.reserve(from.class_map.size());
  for (const auto & vc : from.class_map) {
    to.class_map.push_back(fromRos(vc));
  }
  to.threshold = from.threshold;
  return to;
}

vision_msgs::msg::LabelInfo toRos(const LabelInfo & from)
{
  vision_msgs::msg::LabelInfo to;
  detail::copyHeader(from.header, to.header);
  to.class_map.reserve(from.class_map.size());
  for (const auto & vc : from.class_map) {
    to.class_map.push_back(toRos(vc));
  }
  to.threshold = from.threshold;
  return to;
}

VisionInfo fromRos(const vision_msgs::msg::VisionInfo & from)
{
  VisionInfo to;
  detail::copyHeader(from.header, to.header);
  to.method = from.method;
  to.database_location = from.database_location;
  to.database_version = from.database_version;
  return to;
}

vision_msgs::msg::VisionInfo toRos(const VisionInfo & from)
{
  vision_msgs::msg::VisionInfo to;
  detail::copyHeader(from.header, to.header);
  to.method = from.method;
  to.database_location = from.database_location;
  to.database_version = from.database_version;
  return to;
}

}  // namespace autonomy_ros::conversions
