// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autonomy_ros/bridge/tf_bridge.hpp"

#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy_ros/conversions/conversions.hpp"

namespace autonomy_ros::bridge
{

namespace
{

::geometry_msgs::TransformStamped toInternalTransform(
  const ::autonomy::commsgs::geometry_msgs::TransformStamped & from)
{
  ::geometry_msgs::TransformStamped internal;
  internal.header.stamp =
    static_cast<uint64_t>(from.header.stamp.sec) * 1000000000ULL +
    static_cast<uint64_t>(from.header.stamp.nanosec);
  internal.header.frame_id = from.header.frame_id;
  internal.child_frame_id = from.child_frame_id;
  internal.transform.translation.x = from.transform.translation.x;
  internal.transform.translation.y = from.transform.translation.y;
  internal.transform.translation.z = from.transform.translation.z;
  internal.transform.rotation.x = from.transform.rotation.x;
  internal.transform.rotation.y = from.transform.rotation.y;
  internal.transform.rotation.z = from.transform.rotation.z;
  internal.transform.rotation.w = from.transform.rotation.w;
  return internal;
}

}  // namespace

TfBridge::TfBridge(rclcpp::Node & node, const system::AutonomyRosOptions & ros_options)
: node_(node),
  tf_topic_(ros_options.tf_topic),
  tf_static_topic_(ros_options.tf_static_topic)
{
  ::autonomy::transform::Buffer::Instance()->Init();

  tf_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
    tf_topic_, rclcpp::QoS(100),
    [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { onTf(msg, false); });

  tf_static_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
    tf_static_topic_, rclcpp::QoS(1).transient_local(),
    [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { onTf(msg, true); });

  RCLCPP_INFO(
    node_.get_logger(), "[tf_bridge] %s + %s -> autonomy::transform::Buffer",
    tf_topic_.c_str(), tf_static_topic_.c_str());
}

TfBridge::~TfBridge()
{
  tf_sub_.reset();
  tf_static_sub_.reset();
}

void TfBridge::onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static)
{
  if (!msg) {
    return;
  }
  for (const auto & tf : msg->transforms) {
    injectTransform(tf, is_static);
  }
}

void TfBridge::injectTransform(
  const geometry_msgs::msg::TransformStamped & tf, bool is_static)
{
  const auto core_tf = autonomy_ros::conversions::fromRos(tf);
  const auto internal = toInternalTransform(core_tf);

  try {
    ::autonomy::transform::Buffer::Instance()->setTransform(
      internal, "autonomy_ros", is_static);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "[tf_bridge] setTransform failed: %s", e.what());
  }
}

}  // namespace autonomy_ros::bridge
