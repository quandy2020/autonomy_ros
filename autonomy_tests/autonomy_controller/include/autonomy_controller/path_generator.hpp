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
 * @brief Reference path generation for controller simulation.
 */

#ifndef AUTONOMY_CONTROLLER_PATH_GENERATOR_HPP_
#define AUTONOMY_CONTROLLER_PATH_GENERATOR_HPP_

#include <string>

#include "autonomy/commsgs/planning_msgs.hpp"

/**
 * @namespace autonomy_ros
 * @brief Shared helpers used by autonomy_ros test nodes.
 */
namespace autonomy_ros
{

/**
 * @enum PathShape
 * @brief Closed-loop reference path geometry.
 */
enum class PathShape
{
  Circle,
  Rectangle,
  FigureEight,
  Line,
};

/**
 * @struct PathGeneratorParams
 * @brief Numeric parameters for reference path generation.
 */
struct PathGeneratorParams
{
  /** TF frame stamped on every pose. */
  std::string frame_id{"map"};
  double center_x{0.0};
  double center_y{0.0};
  /** Circle radius [m]. */
  double radius{2.0};
  /** Rectangle full width / height [m]. */
  double width{4.0};
  double height{3.0};
  /** Gerono lemniscate semi-axis a [m]: x = a·sin(t), y = a·sin(t)cos(t). */
  double figure_eight_scale{2.0};
  /** Max distance between consecutive poses [m]. */
  double pose_spacing{0.05};
};

/**
 * @brief Parse shape name from ROS parameter string.
 * @param name One of circle, rectangle, rect, figure_eight, figure8, eight.
 */
PathShape ParsePathShape(const std::string & name);

/**
 * @brief Generate a closed reference path in the given frame.
 * @param shape Closed-loop geometry.
 * @param params Center, size, and pose spacing.
 */
autonomy::commsgs::planning_msgs::Path GeneratePath(
  PathShape shape,
  const PathGeneratorParams & params);

/**
 * @brief Generate an open path from start to goal (RViz /goal_pose mode).
 * @param start Current robot pose in the planning frame.
 * @param goal Target pose in the planning frame.
 * @param pose_spacing Max distance between consecutive poses [m].
 */
autonomy::commsgs::planning_msgs::Path GenerateLinePath(
  const autonomy::commsgs::geometry_msgs::PoseStamped & start,
  const autonomy::commsgs::geometry_msgs::PoseStamped & goal,
  double pose_spacing);

}  // namespace autonomy_ros

#endif  // AUTONOMY_CONTROLLER_PATH_GENERATOR_HPP_
