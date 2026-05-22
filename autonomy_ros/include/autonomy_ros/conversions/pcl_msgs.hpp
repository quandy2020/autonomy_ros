// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__PCL_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__PCL_MSGS_HPP_

/// @file pcl_msgs.hpp
/// @brief Converts ROS 2 pcl_msgs and local pcl_msgs_commsgs placeholders.
///
/// Proto schema: autonomy/commsgs/proto/pcl_msgs.proto
///
/// @par Overview
/// autonomy::commsgs::pcl_msgs is currently empty in the core library. Types in
/// namespace pcl_msgs_commsgs mirror the proto definitions and are used only for
/// ROS bridging until core commsgs structs exist.
///
/// @par Usage
/// Include this header when interfacing with PCL ROS messages (indices, meshes,
/// model coefficients). PolygonMesh conversion also uses sensor_msgs PointCloud2.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/pcl_msgs.hpp"
/// auto indices = autonomy_ros::conversions::fromRos(*ros_indices);
/// @endcode

#include <cstdint>
#include <vector>

#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "pcl_msgs/msg/model_coefficients.hpp"
#include "pcl_msgs/msg/point_indices.hpp"
#include "pcl_msgs/msg/polygon_mesh.hpp"
#include "pcl_msgs/msg/vertices.hpp"

namespace autonomy_ros::conversions
{

// commsgs::pcl_msgs is empty; types below match autonomy/commsgs/proto/pcl_msgs.proto.
namespace pcl_msgs_commsgs
{

struct Vertices
{
  std::vector<uint32_t> vertices;
};

struct ModelCoefficients
{
  ::autonomy::commsgs::std_msgs::Header header;
  std::vector<float> values;
};

struct PointIndices
{
  ::autonomy::commsgs::std_msgs::Header header;
  std::vector<int32_t> indices;
};

struct PolygonMesh
{
  ::autonomy::commsgs::std_msgs::Header header;
  ::autonomy::commsgs::sensor_msgs::PointCloud2 cloud;
  std::vector<Vertices> polygons;
};

}  // namespace pcl_msgs_commsgs

using Vertices = pcl_msgs_commsgs::Vertices;
using ModelCoefficients = pcl_msgs_commsgs::ModelCoefficients;
using PointIndices = pcl_msgs_commsgs::PointIndices;
using PolygonMesh = pcl_msgs_commsgs::PolygonMesh;

/**
 * @brief Bidirectional conversion between ROS pcl_msgs::msg::Vertices and commsgs Vertices.
 *
 * @par fromRos
 * @param from Input ROS message (pcl_msgs::msg::Vertices). Fields are copied without coordinate transforms.
 * @return commsgs Vertices for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Vertices from planners, bridges, or drivers.
 * @return ROS pcl_msgs::msg::Vertices ready for rclcpp publish() or subscribe() adapters.
 */
Vertices fromRos(const pcl_msgs::msg::Vertices & from);
pcl_msgs::msg::Vertices toRos(const Vertices & from);

/**
 * @brief Bidirectional conversion between ROS pcl_msgs::msg::ModelCoefficients and commsgs ModelCoefficients.
 *
 * @par fromRos
 * @param from Input ROS message (pcl_msgs::msg::ModelCoefficients). Fields are copied without coordinate transforms.
 * @return commsgs ModelCoefficients for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs ModelCoefficients from planners, bridges, or drivers.
 * @return ROS pcl_msgs::msg::ModelCoefficients ready for rclcpp publish() or subscribe() adapters.
 */
ModelCoefficients fromRos(const pcl_msgs::msg::ModelCoefficients & from);
pcl_msgs::msg::ModelCoefficients toRos(const ModelCoefficients & from);

/**
 * @brief Bidirectional conversion between ROS pcl_msgs::msg::PointIndices and commsgs PointIndices.
 *
 * @par fromRos
 * @param from Input ROS message (pcl_msgs::msg::PointIndices). Fields are copied without coordinate transforms.
 * @return commsgs PointIndices for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PointIndices from planners, bridges, or drivers.
 * @return ROS pcl_msgs::msg::PointIndices ready for rclcpp publish() or subscribe() adapters.
 */
PointIndices fromRos(const pcl_msgs::msg::PointIndices & from);
pcl_msgs::msg::PointIndices toRos(const PointIndices & from);

/**
 * @brief Bidirectional conversion between ROS pcl_msgs::msg::PolygonMesh and commsgs PolygonMesh.
 *
 * @par fromRos
 * @param from Input ROS message (pcl_msgs::msg::PolygonMesh). Fields are copied without coordinate transforms.
 * @return commsgs PolygonMesh for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs PolygonMesh from planners, bridges, or drivers.
 * @return ROS pcl_msgs::msg::PolygonMesh ready for rclcpp publish() or subscribe() adapters.
 */
PolygonMesh fromRos(const pcl_msgs::msg::PolygonMesh & from);
pcl_msgs::msg::PolygonMesh toRos(const PolygonMesh & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__PCL_MSGS_HPP_
