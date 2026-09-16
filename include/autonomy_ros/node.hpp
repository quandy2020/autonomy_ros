/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_ROS__NODE_HPP_
#define AUTONOMY_ROS__NODE_HPP_

#include <memory>

#include "autolink/node/node.hpp"
#include "autonomy_ros/options.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autonomy_ros
{
class AutolinkBridge;
class Visualizer;
class RvizTools;

/**
 * @brief Process-isolated ROS front-end for autonomy (autolink message bridge).
 *
 * Does not link or embed libautonomy.
 */
class RosAutonomySystem
{
public:
  RosAutonomySystem(rclcpp::Node & node, Options options);

  ~RosAutonomySystem();

  bool IsRunning() const { return running_; }

private:
  rclcpp::Node & node_;
  bool running_{false};
  Options options_;

  std::shared_ptr<autolink::Node> al_node_;
  std::unique_ptr<Visualizer> visualizer_;
  std::unique_ptr<AutolinkBridge> bridge_;
  std::unique_ptr<RvizTools> rviz_tools_;
};

}  // namespace autonomy_ros

#endif  // AUTONOMY_ROS__NODE_HPP_
