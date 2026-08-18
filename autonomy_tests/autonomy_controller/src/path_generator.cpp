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
 * @brief Implements reference path generation for controller simulation.
 */

#include "autonomy_controller/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <vector>

namespace autonomy_ros
{
namespace
{

using automsgs::msgs::geometry_msgs::PoseStamped;
using automsgs::msgs::nav_msgs::Path;

PoseStamped MakePose(double x, double y, double yaw, const std::string & frame_id)
{
  PoseStamped pose;
  pose.mutable_header()->set_frame_id(frame_id);
  pose.mutable_pose()->mutable_position()->set_x(x);
  pose.mutable_pose()->mutable_position()->set_y(y);
  pose.mutable_pose()->mutable_position()->set_z(0.0);
  const double half_yaw = yaw * 0.5;
  pose.mutable_pose()->mutable_orientation()->set_z(std::sin(half_yaw));
  pose.mutable_pose()->mutable_orientation()->set_w(std::cos(half_yaw));
  return pose;
}

void AppendSegment(
  std::vector<PoseStamped> & poses,
  double x0, double y0,
  double x1, double y1,
  double spacing,
  const std::string & frame_id)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-9) {
    return;
  }
  const int segments = std::max(1, static_cast<int>(std::ceil(dist / spacing)));
  const double yaw = std::atan2(dy, dx);
  for (int s = 1; s <= segments; ++s) {
    const double t = static_cast<double>(s) / segments;
    poses.push_back(MakePose(x0 + t * dx, y0 + t * dy, yaw, frame_id));
  }
}

Path BuildFromPolyline(
  const std::vector<std::pair<double, double>> & points,
  double spacing,
  const std::string & frame_id,
  bool closed)
{
  if (points.size() < 2) {
    throw std::runtime_error("path polyline needs at least 2 points");
  }

  std::vector<PoseStamped> poses;
  poses.push_back(MakePose(points.front().first, points.front().second, 0.0, frame_id));

  const size_t end = closed ? points.size() : points.size() - 1;
  for (size_t i = 0; i < end; ++i) {
    const auto & p0 = points[i];
    const auto & p1 = points[(i + 1) % points.size()];
    AppendSegment(poses, p0.first, p0.second, p1.first, p1.second, spacing, frame_id);
  }

  if (!closed && poses.size() >= 2) {
    const auto & last = points.back();
    const auto & prev = poses[poses.size() - 2];
    const double yaw = std::atan2(
      last.second - prev.pose().position().y(),
      last.first - prev.pose().position().x());
    poses.back() = MakePose(last.first, last.second, yaw, frame_id);
  }

  if (closed && poses.size() >= 2) {
    const auto & p0 = poses[0].pose().position();
    const auto & p1 = poses[1].pose().position();
    const double yaw = std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
    poses[0] = MakePose(p0.x(), p0.y(), yaw, frame_id);
  }

  Path path;
  path.mutable_header()->set_frame_id(frame_id);
  for (const auto & pose : poses) {
    *path.add_poses() = pose;
  }
  return path;
}

Path BuildFromSamples(
  const std::vector<std::pair<double, double>> & samples,
  double spacing,
  const std::string & frame_id,
  bool closed)
{
  if (samples.size() < 2) {
    throw std::runtime_error("path samples need at least 2 points");
  }
  return BuildFromPolyline(samples, spacing, frame_id, closed);
}

std::string ToLower(std::string s)
{
  for (char & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

}  // namespace

PathShape ParsePathShape(const std::string & name)
{
  const std::string key = ToLower(name);
  if (key == "circle" || key == "circ") {
    return PathShape::Circle;
  }
  if (key == "rectangle" || key == "rect") {
    return PathShape::Rectangle;
  }
  if (key == "figure_eight" || key == "figure8" || key == "eight" || key == "8") {
    return PathShape::FigureEight;
  }
  if (key == "line" || key == "straight") {
    return PathShape::Line;
  }
  throw std::runtime_error("unknown path_shape: " + name);
}

Path GeneratePath(PathShape shape, const PathGeneratorParams & params)
{
  if (params.pose_spacing <= 0.0) {
    throw std::runtime_error("pose_spacing must be > 0");
  }

  const std::string & frame = params.frame_id;
  const double cx = params.center_x;
  const double cy = params.center_y;

  switch (shape) {
    case PathShape::Circle: {
        if (params.radius <= 0.0) {
          throw std::runtime_error("circle radius must be > 0");
        }
        const double circumference = 2.0 * M_PI * params.radius;
        const int samples = std::max(
          8, static_cast<int>(std::ceil(circumference / params.pose_spacing)));
        std::vector<std::pair<double, double>> pts;
        pts.reserve(static_cast<size_t>(samples));
        for (int i = 0; i < samples; ++i) {
          const double t = 2.0 * M_PI * static_cast<double>(i) / samples;
          pts.emplace_back(
            cx + params.radius * std::cos(t),
            cy + params.radius * std::sin(t));
        }
        return BuildFromSamples(pts, params.pose_spacing, frame, true);
      }

    case PathShape::Rectangle: {
        if (params.width <= 0.0 || params.height <= 0.0) {
          throw std::runtime_error("rectangle width/height must be > 0");
        }
        const double hx = params.width * 0.5;
        const double hy = params.height * 0.5;
        const std::vector<std::pair<double, double>> corners = {
          {cx - hx, cy - hy},
          {cx + hx, cy - hy},
          {cx + hx, cy + hy},
          {cx - hx, cy + hy},
        };
        return BuildFromPolyline(corners, params.pose_spacing, frame, true);
      }

    case PathShape::FigureEight: {
        if (params.figure_eight_scale <= 0.0) {
          throw std::runtime_error("figure_eight_scale must be > 0");
        }
        // Gerono lemniscate: x = a·sin(t), y = a·sin(t)cos(t), t ∈ [0, 2π).
        // Single smooth closed curve crossing once at the center (figure ∞).
        // Start from the outer lobe instead of the self-intersection to avoid
        // immediate branch ambiguity for closed-loop tracking controllers.
        const double a = params.figure_eight_scale;
        const double phase = 0.5 * M_PI;
        constexpr int kArcEstimateSteps = 128;
        double perimeter = 0.0;
        for (int i = 0; i < kArcEstimateSteps; ++i) {
          const double t0 =
            phase + 2.0 * M_PI * static_cast<double>(i) / kArcEstimateSteps;
          const double t1 =
            phase + 2.0 * M_PI * static_cast<double>(i + 1) / kArcEstimateSteps;
          const double speed0 = std::hypot(a * std::cos(t0), a * std::cos(2.0 * t0));
          const double speed1 = std::hypot(a * std::cos(t1), a * std::cos(2.0 * t1));
          perimeter += 0.5 * (speed0 + speed1) * (t1 - t0);
        }
        const int samples = std::max(
          32, static_cast<int>(std::ceil(perimeter / params.pose_spacing)));
        std::vector<std::pair<double, double>> pts;
        pts.reserve(static_cast<size_t>(samples));
        for (int i = 0; i < samples; ++i) {
          const double t = phase + 2.0 * M_PI * static_cast<double>(i) / samples;
          const double s = std::sin(t);
          pts.emplace_back(
            cx + a * s,
            cy + a * s * std::cos(t));
        }
        return BuildFromSamples(pts, params.pose_spacing, frame, true);
      }

    case PathShape::Line: {
        const double x0 = params.center_x - params.width * 0.5;
        const double x1 = params.center_x + params.width * 0.5;
        const double y = params.center_y;
        const double yaw = std::atan2(0.0, x1 - x0);
        Path path;
        path.mutable_header()->set_frame_id(frame);
        const double dist = std::hypot(x1 - x0, y - y);
        const int segments =
            std::max(1, static_cast<int>(std::ceil(dist / params.pose_spacing)));
        for (int s = 0; s <= segments; ++s) {
            const double t = static_cast<double>(s) / segments;
            *path.add_poses() = MakePose(x0 + t * (x1 - x0), y, yaw, frame);
        }
        return path;
      }
  }

  throw std::runtime_error("unsupported path shape");
}

Path GenerateLinePath(
  const PoseStamped & start,
  const PoseStamped & goal,
  double spacing)
{
  if (spacing <= 0.0) {
    throw std::runtime_error("pose_spacing must be > 0");
  }
  const std::string & frame = start.header().frame_id().empty() ?
    goal.header().frame_id() : start.header().frame_id();
  const std::vector<std::pair<double, double>> segment = {
    {start.pose().position().x(), start.pose().position().y()},
    {goal.pose().position().x(), goal.pose().position().y()},
  };
  Path path = BuildFromPolyline(segment, spacing, frame, false);
  if (path.poses_size() >= 2) {
    *path.mutable_poses(0) = start;
    *path.mutable_poses(path.poses_size() - 1) = goal;
    path.mutable_poses(path.poses_size() - 1)->mutable_header()->set_frame_id(frame);
  } else if (path.poses_size() == 1) {
    *path.mutable_poses(0) = goal;
  }
  path.mutable_header()->set_frame_id(frame);
  return path;
}

}  // namespace autonomy_ros
