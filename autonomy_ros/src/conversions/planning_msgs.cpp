/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 * email: quandy2020@126.com
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

#include "autonomy_ros/conversions/planning_msgs.hpp"

#include "autonomy_ros/conversions/detail.hpp"
#include "autonomy_ros/conversions/geometry_msgs.hpp"

namespace autonomy_ros
{

Odometry fromRos(const nav_msgs::msg::Odometry & from)
{
  Odometry to;
  copyHeader(from.header, *to.mutable_header());
  to.set_child_frame_id(from.child_frame_id);
  copyPose(from.pose.pose, *to.mutable_pose()->mutable_pose()->mutable_pose());
  copyCovariance6(from.pose.covariance, to.mutable_pose()->mutable_covariance());
  copyTwist(from.twist.twist, *to.mutable_twist()->mutable_twist());
  copyCovariance6(from.twist.covariance, to.mutable_twist()->mutable_covariance());
  return to;
}

nav_msgs::msg::Odometry toRos(const Odometry & from)
{
  nav_msgs::msg::Odometry to;
  copyHeader(from.header(), to.header);
  to.child_frame_id = from.child_frame_id();
  copyPose(from.pose().pose().pose(), to.pose.pose);
  copyCovariance6(from.pose().covariance(), to.pose.covariance);
  copyTwist(from.twist().twist(), to.twist.twist);
  copyCovariance6(from.twist().covariance(), to.twist.covariance);
  return to;
}

nav_msgs::msg::Path toRos(const Path & from)
{
  nav_msgs::msg::Path to;
  copyHeader(from.header(), to.header);
  to.poses.reserve(static_cast<size_t>(from.poses_size()));
  for (const auto & pose : from.poses()) {
    to.poses.push_back(toRos(pose));
  }
  return to;
}

Path fromRos(const nav_msgs::msg::Path & from)
{
  Path to;
  copyHeader(from.header, *to.mutable_header());
  to.clear_poses();
  for (const auto & pose : from.poses) {
    *to.add_poses() = fromRos(pose);
  }
  return to;
}

}  // namespace autonomy_ros
