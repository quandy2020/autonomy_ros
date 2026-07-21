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
 * @brief Test fixtures for autonomy_planner unit tests.
 */

#ifndef AUTONOMY_PLANNER_TEST_HELPERS_HPP_
#define AUTONOMY_PLANNER_TEST_HELPERS_HPP_

#include <memory>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

/**
 * @namespace autonomy_planner::test
 * @brief Inline helpers for planner plugin unit tests.
 */
namespace autonomy_planner {
namespace test {

/**
 * @brief Create a small fixed-size costmap wrapper for planner tests.
 * @param width_m Map width in meters.
 * @param height_m Map height in meters.
 * @param resolution Cell size in meters.
 */
inline autonomy::map::costmap_2d::Costmap2DWrapper::SharedPtr CreateTestCostmapWrapper(
  double width_m = 10.0,
  double height_m = 10.0,
  double resolution = 0.05)
{
  autonomy::map::proto::Costmap2DOptions options;
  options.set_enabled(true);
  options.set_frame_id("map");
  options.set_name("test_costmap");
  options.set_resolution(resolution);
  options.set_width(width_m);
  options.set_height(height_m);
  options.add_plugins("none");
  return std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(
    options, "test_costmap");
}

/** @brief Reset all costmap cells to FREE_SPACE. */
inline void ClearToFree(
  autonomy::map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper)
{
  auto * costmap = costmap_wrapper->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    autonomy::map::costmap_2d::FREE_SPACE);
}

/** @brief Build a PoseStamped at (x, y) in the given frame. */
inline autonomy::commsgs::geometry_msgs::PoseStamped MakePose(
  double x, double y, const std::string & frame_id = "map")
{
  autonomy::commsgs::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

/** @brief Default NavFn planner options for unit tests. */
inline autonomy::planning::proto::PlannerOptions MakeNavfnOptions()
{
  autonomy::planning::proto::PlannerOptions options;
  options.mutable_navfn()->set_tolerance(0.1);
  options.mutable_navfn()->set_use_astar(false);
  options.mutable_navfn()->set_allow_unknown(true);
  options.mutable_navfn()->set_use_final_approach_orientation(false);
  return options;
}

/** @brief Default Dijkstra planner options for unit tests. */
inline autonomy::planning::proto::PlannerOptions MakeDijkstraOptions()
{
  autonomy::planning::proto::PlannerOptions options;
  options.mutable_dijkstra()->set_tolerance(0.15);
  options.mutable_dijkstra()->set_allow_unknown(true);
  options.mutable_dijkstra()->set_use_final_approach_orientation(true);
  return options;
}

/** @brief Default Theta* planner options for unit tests. */
inline autonomy::planning::proto::PlannerOptions MakeThetaStarOptions()
{
  autonomy::planning::proto::PlannerOptions options;
  options.mutable_theta_star()->set_how_many_corners(8);
  options.mutable_theta_star()->set_allow_unknown(true);
  options.mutable_theta_star()->set_w_euc_cost(2.0);
  options.mutable_theta_star()->set_w_traversal_cost(1.0);
  options.mutable_theta_star()->set_w_heuristic_cost(1.0);
  return options;
}

}  // namespace test
}  // namespace autonomy_planner

#endif  // AUTONOMY_PLANNER_TEST_HELPERS_HPP_
