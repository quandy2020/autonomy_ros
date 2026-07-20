/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/planning/planner/navfn/navfn_planner.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy_planner/test_helpers.hpp"

namespace
{

using autonomy_planner::test::ClearToFree;
using autonomy_planner::test::CreateTestCostmapWrapper;
using autonomy_planner::test::MakeNavfnOptions;
using autonomy_planner::test::MakePose;

}  // namespace

TEST(NavfnPlanner, FindsPathOnFreeMap)
{
  auto costmap_wrapper = CreateTestCostmapWrapper();
  ASSERT_NE(costmap_wrapper->getCostmap(), nullptr);
  ClearToFree(costmap_wrapper);

  autonomy::planning::planner::navfn::NavfnPlanner planner(
    MakeNavfnOptions(), "navfn_planner", costmap_wrapper);

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

TEST(NavfnPlanner, ReturnsCanceledWhenCancelled)
{
  auto costmap_wrapper = CreateTestCostmapWrapper();
  ClearToFree(costmap_wrapper);

  autonomy::planning::planner::navfn::NavfnPlanner planner(
    MakeNavfnOptions(), "navfn_planner", costmap_wrapper);

  autonomy::commsgs::planning_msgs::Path path;
  const auto start = MakePose(0.25, 0.25);
  const auto goal = MakePose(0.40, 0.40);
  const uint32_t code =
    planner.CreatePlan(start, goal, path, []() {return true;});

  EXPECT_EQ(
    static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_CANCELED),
    code);
}

TEST(NavfnPlanner, DoesNotMutateLethalStartCell)
{
  auto costmap_wrapper = CreateTestCostmapWrapper();
  auto * costmap = costmap_wrapper->getCostmap();
  ClearToFree(costmap_wrapper);

  const unsigned int start_mx = 5;
  const unsigned int start_my = 5;
  costmap->setCost(start_mx, start_my, autonomy::map::costmap_2d::LETHAL_OBSTACLE);
  const unsigned char cost_before = costmap->getCost(start_mx, start_my);

  autonomy::planning::planner::navfn::NavfnPlanner planner(
    MakeNavfnOptions(), "navfn_planner", costmap_wrapper);

  autonomy::commsgs::planning_msgs::Path path;
  const auto start = MakePose(start_mx * 0.05, start_my * 0.05);
  const auto goal = MakePose(0.40, 0.40);
  const uint32_t code =
    planner.CreatePlan(start, goal, path, []() {return false;});

  EXPECT_EQ(cost_before, costmap->getCost(start_mx, start_my));
  EXPECT_EQ(
    static_cast<uint32_t>(
      autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS),
    code);
  EXPECT_FALSE(path.poses.empty());
}
