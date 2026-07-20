/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include <cmath>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy_controller {
namespace test {

inline autonomy::commsgs::geometry_msgs::Pose MakePose(
  double x, double y, double yaw = 0.0)
{
  autonomy::commsgs::geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

inline autonomy::commsgs::geometry_msgs::PoseStamped MakePoseStamped(
  double x, double y, double yaw = 0.0, const std::string & frame = "odom")
{
  autonomy::commsgs::geometry_msgs::PoseStamped stamped;
  stamped.header.frame_id = frame;
  stamped.pose = MakePose(x, y, yaw);
  return stamped;
}

inline autonomy::commsgs::geometry_msgs::Twist MakeTwist(
  double vx, double vy, double wz)
{
  autonomy::commsgs::geometry_msgs::Twist twist;
  twist.linear.x = vx;
  twist.linear.y = vy;
  twist.angular.z = wz;
  return twist;
}

inline autonomy::commsgs::geometry_msgs::Point MakePoint(double x, double y)
{
  autonomy::commsgs::geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

inline autonomy::commsgs::planning_msgs::Path MakeStraightPath(
  double x0, double y0, double x1, double y1, int n = 10)
{
  autonomy::commsgs::planning_msgs::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i < n; ++i) {
    const double t = (n == 1) ? 0.0 : static_cast<double>(i) / (n - 1);
    path.poses.push_back(
      MakePoseStamped(x0 + t * (x1 - x0), y0 + t * (y1 - y0)));
  }
  return path;
}

}  // namespace test
}  // namespace autonomy_controller
