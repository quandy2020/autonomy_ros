/*
 * Copyright 2026 autonomy_ros contributors
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

/**
 * @file
 * @brief Test fixtures for autonomy_controller unit tests.
 */

#ifndef AUTONOMY_CONTROLLER_TEST_HELPERS_HPP_
#define AUTONOMY_CONTROLLER_TEST_HELPERS_HPP_

#include <cmath>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

/**
 * @namespace autonomy_controller::test
 * @brief Inline helpers for controller plugin unit tests.
 */
namespace autonomy_controller
{
namespace test
{

/**
 * @brief Build a planar pose with yaw about +z.
 * @param x Position x [m].
 * @param y Position y [m].
 * @param yaw Heading [rad].
 */
inline automsgs::msgs::geometry_msgs::Pose MakePose(
  double x, double y, double yaw = 0.0)
{
  automsgs::msgs::geometry_msgs::Pose pose;
  pose.mutable_position()->set_x(x);
  pose.mutable_position()->set_y(y);
  pose.mutable_position()->set_z(0.0);
  pose.mutable_orientation()->set_x(0.0);
  pose.mutable_orientation()->set_y(0.0);
  pose.mutable_orientation()->set_z(std::sin(yaw * 0.5));
  pose.mutable_orientation()->set_w(std::cos(yaw * 0.5));
  return pose;
}

/**
 * @brief Build a stamped pose in the given frame.
 */
inline automsgs::msgs::geometry_msgs::PoseStamped MakePoseStamped(
  double x, double y, double yaw = 0.0, const std::string & frame = "odom")
{
  automsgs::msgs::geometry_msgs::PoseStamped stamped;
  stamped.mutable_header()->set_frame_id(frame);
  *stamped.mutable_pose() = MakePose(x, y, yaw);
  return stamped;
}

/**
 * @brief Build a body-frame twist (planar motion).
 */
inline automsgs::msgs::geometry_msgs::Twist MakeTwist(
  double vx, double vy, double wz)
{
  automsgs::msgs::geometry_msgs::Twist twist;
  twist.mutable_linear()->set_x(vx);
  twist.mutable_linear()->set_y(vy);
  twist.mutable_angular()->set_z(wz);
  return twist;
}

/**
 * @brief Build a 2-D point with z = 0.
 */
inline automsgs::msgs::geometry_msgs::Point MakePoint(double x, double y)
{
  automsgs::msgs::geometry_msgs::Point p;
  p.set_x(x);
  p.set_y(y);
  p.set_z(0.0);
  return p;
}

/**
 * @brief Build a straight polyline path with uniform spacing in pose count.
 */
inline automsgs::msgs::nav_msgs::Path MakeStraightPath(
  double x0, double y0, double x1, double y1, int n = 10)
{
  automsgs::msgs::nav_msgs::Path path;
  path.mutable_header()->set_frame_id("odom");
  for (int i = 0; i < n; ++i) {
    const double t = (n == 1) ? 0.0 : static_cast<double>(i) / (n - 1);
    *path.add_poses() =
      MakePoseStamped(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
  }
  return path;
}

}  // namespace test
}  // namespace autonomy_controller

#endif  // AUTONOMY_CONTROLLER_TEST_HELPERS_HPP_
