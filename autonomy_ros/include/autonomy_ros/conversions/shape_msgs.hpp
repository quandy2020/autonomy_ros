// Copyright 2026 autonomy_ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTONOMY_ROS__CONVERSIONS__SHAPE_MSGS_HPP_
#define AUTONOMY_ROS__CONVERSIONS__SHAPE_MSGS_HPP_

/// @file shape_msgs.hpp
/// @brief Converts ROS 2 shape_msgs to commsgs shape types in builtin_interfaces.
///
/// Proto schema: autonomy/commsgs/proto/shape_msgs.proto
///
/// @par Overview
/// Commsgs shape structs (Plane, SolidPrimitive, Mesh, MeshTriangle) are defined
/// under autonomy::commsgs::builtin_interfaces; this module maps ROS shape_msgs.
///
/// @par Usage
/// Include this header (or conversions/conversions.hpp) for RViz collision shapes
/// and visualization primitives. Use fromRos() / toRos() like other packages.
///
/// @par Example
/// @code
/// #include "autonomy_ros/conversions/shape_msgs.hpp"
/// auto mesh = autonomy_ros::conversions::fromRos(ros_mesh);
/// @endcode

#include "autonomy/commsgs/shape_msgs.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/mesh_triangle.hpp"
#include "shape_msgs/msg/plane.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace autonomy_ros::conversions
{

// commsgs shape types live in builtin_interfaces (see shape_msgs.hpp).
using Plane = ::autonomy::commsgs::builtin_interfaces::Plane;
using SolidPrimitive = ::autonomy::commsgs::builtin_interfaces::SolidPrimitive;
using MeshTriangle = ::autonomy::commsgs::builtin_interfaces::MeshTriangle;
using Mesh = ::autonomy::commsgs::builtin_interfaces::Mesh;

/**
 * @brief Bidirectional conversion between ROS shape_msgs::msg::Plane and commsgs Plane.
 *
 * @par fromRos
 * @param from Input ROS message (shape_msgs::msg::Plane). Fields are copied without coordinate transforms.
 * @return commsgs Plane for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Plane from planners, bridges, or drivers.
 * @return ROS shape_msgs::msg::Plane ready for rclcpp publish() or subscribe() adapters.
 */
Plane fromRos(const shape_msgs::msg::Plane & from);
shape_msgs::msg::Plane toRos(const Plane & from);

/**
 * @brief Bidirectional conversion between ROS shape_msgs::msg::SolidPrimitive and commsgs SolidPrimitive.
 *
 * @par fromRos
 * @param from Input ROS message (shape_msgs::msg::SolidPrimitive). Fields are copied without coordinate transforms.
 * @return commsgs SolidPrimitive for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs SolidPrimitive from planners, bridges, or drivers.
 * @return ROS shape_msgs::msg::SolidPrimitive ready for rclcpp publish() or subscribe() adapters.
 */
SolidPrimitive fromRos(const shape_msgs::msg::SolidPrimitive & from);
shape_msgs::msg::SolidPrimitive toRos(const SolidPrimitive & from);

/**
 * @brief Bidirectional conversion between ROS shape_msgs::msg::MeshTriangle and commsgs MeshTriangle.
 *
 * @par fromRos
 * @param from Input ROS message (shape_msgs::msg::MeshTriangle). Fields are copied without coordinate transforms.
 * @return commsgs MeshTriangle for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs MeshTriangle from planners, bridges, or drivers.
 * @return ROS shape_msgs::msg::MeshTriangle ready for rclcpp publish() or subscribe() adapters.
 */
MeshTriangle fromRos(const shape_msgs::msg::MeshTriangle & from);
shape_msgs::msg::MeshTriangle toRos(const MeshTriangle & from);

/**
 * @brief Bidirectional conversion between ROS shape_msgs::msg::Mesh and commsgs Mesh.
 *
 * @par fromRos
 * @param from Input ROS message (shape_msgs::msg::Mesh). Fields are copied without coordinate transforms.
 * @return commsgs Mesh for autonomy core APIs or protobuf via ToProto().
 *
 * @par toRos
 * @param from Input commsgs Mesh from planners, bridges, or drivers.
 * @return ROS shape_msgs::msg::Mesh ready for rclcpp publish() or subscribe() adapters.
 */
Mesh fromRos(const shape_msgs::msg::Mesh & from);
shape_msgs::msg::Mesh toRos(const Mesh & from);

}  // namespace autonomy_ros::conversions

#endif  // AUTONOMY_ROS__CONVERSIONS__SHAPE_MSGS_HPP_
