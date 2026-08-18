/*
 * Copyright 2026 autonomy_ros contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <string>

#include "autonomy_ros/conversions/planning_msgs.hpp"

namespace autonomy_ros
{

enum class PathShape
{
  Circle,
  Rectangle,
  FigureEight,
};

struct PathGeneratorParams
{
  std::string frame_id{"map"};
  double center_x{0.0};
  double center_y{0.0};
  /** Circle radius [m]. */
  double radius{2.0};
  /** Rectangle full width / height [m]. */
  double width{4.0};
  double height{3.0};
  /** Figure-eight scale [m] (Gerono lemniscate). */
  double figure_eight_scale{2.0};
  /** Max distance between consecutive poses [m]. */
  double pose_spacing{0.05};
};

/** Parse shape name: circle, rectangle, rect, figure_eight, figure8, eight. */
PathShape ParsePathShape(const std::string & name);

/** Generate a closed reference path in the given frame. */
automsgs::msgs::nav_msgs::Path GeneratePath(
  PathShape shape,
  const PathGeneratorParams & params);

}  // namespace autonomy_ros
