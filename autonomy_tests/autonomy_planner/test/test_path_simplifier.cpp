/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/planning/utils/path_simplifier.hpp"
#include "autonomy_planner/test_helpers.hpp"

namespace
{

using autonomy_planner::test::MakePose;

}  // namespace

TEST(PathSimplifier, ReturnsUnchangedWhenEpsilonNonPositive)
{
  autonomy::commsgs::planning_msgs::Path path;
  path.poses = {MakePose(0.0, 0.0), MakePose(1.0, 0.0), MakePose(2.0, 0.0)};
  const auto simplified = autonomy::planning::utils::SimplifyPath(path, 0.0);
  EXPECT_EQ(path.poses.size(), simplified.poses.size());
}

TEST(PathSimplifier, ReducesCollinearPoints)
{
  autonomy::commsgs::planning_msgs::Path path;
  path.poses = {
    MakePose(0.0, 0.0), MakePose(1.0, 0.0), MakePose(2.0, 0.0),
    MakePose(3.0, 0.0)};
  const auto simplified = autonomy::planning::utils::SimplifyPath(path, 0.05);
  ASSERT_GE(simplified.poses.size(), 2U);
  EXPECT_LE(simplified.poses.size(), path.poses.size());
  EXPECT_DOUBLE_EQ(simplified.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(simplified.poses.back().pose.position.x, 3.0);
}

TEST(PathSimplifier, KeepsCornerPoint)
{
  autonomy::commsgs::planning_msgs::Path path;
  path.poses = {
    MakePose(0.0, 0.0), MakePose(1.0, 0.0), MakePose(1.0, 1.0),
    MakePose(1.0, 2.0)};
  const auto simplified = autonomy::planning::utils::SimplifyPath(path, 0.05);
  ASSERT_EQ(simplified.poses.size(), 3U);
  EXPECT_DOUBLE_EQ(simplified.poses[1].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(simplified.poses[1].pose.position.y, 0.0);
}
