/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/planning/planner/dijkstra/dijkstra_planner.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy_planner/test_helpers.hpp"

namespace
{

using autonomy_planner::test::ClearToFree;
using autonomy_planner::test::CreateTestCostmapWrapper;
using autonomy_planner::test::MakeDijkstraOptions;
using autonomy_planner::test::MakePose;

}  // namespace

TEST(DijkstraPlanner, FindsPathOnFreeMap)
{
  auto costmap_wrapper = CreateTestCostmapWrapper();
  ClearToFree(costmap_wrapper);

  autonomy::planning::planner::dijkstra::DijkstraPlanner planner(
    MakeDijkstraOptions(), "dijkstra_planner", costmap_wrapper);

  automsgs::msgs::nav_msgs::Path path;
  const auto start = MakePose(0.25, 0.25);
  const auto goal = MakePose(0.40, 0.40);
  const uint32_t code =
    planner.CreatePlan(start, goal, path, []() {return false;});

  EXPECT_EQ(
    static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS),
    code);
  EXPECT_GT(path.poses_size(), 0);
}

TEST(DijkstraPlanner, FindsLongerPathAroundBlock)
{
  auto costmap_wrapper = CreateTestCostmapWrapper(10.0, 10.0, 0.05);
  auto * costmap = costmap_wrapper->getCostmap();
  ClearToFree(costmap_wrapper);

  // Vertical wall with a gap near the top.
  for (unsigned int my = 20; my < 160; ++my) {
    costmap->setCost(100, my, autonomy::map::costmap_2d::LETHAL_OBSTACLE);
  }

  autonomy::planning::planner::dijkstra::DijkstraPlanner planner(
    MakeDijkstraOptions(), "dijkstra_planner", costmap_wrapper);

  automsgs::msgs::nav_msgs::Path path;
  const auto start = MakePose(2.0, 5.0);
  const auto goal = MakePose(8.0, 5.0);
  const uint32_t code =
    planner.CreatePlan(start, goal, path, []() {return false;});

  EXPECT_EQ(
    static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS),
    code);
  EXPECT_GE(path.poses_size(), 2);
}
