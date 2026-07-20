/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/planning/planner/theta_star/theta_star_planner.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy_planner/test_helpers.hpp"

namespace
{

using autonomy_planner::test::ClearToFree;
using autonomy_planner::test::CreateTestCostmapWrapper;
using autonomy_planner::test::MakePose;
using autonomy_planner::test::MakeThetaStarOptions;

}  // namespace

TEST(ThetaStarPlanner, FindsPathOnFreeMap)
{
  auto costmap_wrapper = CreateTestCostmapWrapper();
  ClearToFree(costmap_wrapper);

  autonomy::planning::planner::theta_star::ThetaStarPlanner planner(
    MakeThetaStarOptions(), "theta_star_planner", costmap_wrapper);

  autonomy::commsgs::planning_msgs::Path path;
  const auto start = MakePose(0.25, 0.25);
  const auto goal = MakePose(0.40, 0.40);
  const uint32_t code =
    planner.CreatePlan(start, goal, path, []() {return false;});

  EXPECT_EQ(
    static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS),
    code);
  EXPECT_FALSE(path.poses.empty());
}

TEST(ThetaStarPlanner, FindsPathAroundObstacle)
{
  auto costmap_wrapper = CreateTestCostmapWrapper(10.0, 10.0, 0.05);
  auto * costmap = costmap_wrapper->getCostmap();
  ClearToFree(costmap_wrapper);

  for (unsigned int mx = 80; mx < 120; ++mx) {
    for (unsigned int my = 60; my < 140; ++my) {
      costmap->setCost(mx, my, autonomy::map::costmap_2d::LETHAL_OBSTACLE);
    }
  }

  autonomy::planning::planner::theta_star::ThetaStarPlanner planner(
    MakeThetaStarOptions(), "theta_star_planner", costmap_wrapper);

  autonomy::commsgs::planning_msgs::Path path;
  const auto start = MakePose(2.0, 5.0);
  const auto goal = MakePose(8.0, 5.0);
  const uint32_t code =
    planner.CreatePlan(start, goal, path, []() {return false;});

  EXPECT_EQ(
    static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS),
    code);
  EXPECT_GE(path.poses.size(), 2U);
}
