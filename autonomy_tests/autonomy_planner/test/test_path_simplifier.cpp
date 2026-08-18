/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/planning/utils/path_simplifier.hpp"
#include "autonomy_planner/test_helpers.hpp"

namespace
{

using autonomy_planner::test::MakePose;

automsgs::msgs::nav_msgs::Path MakePath(
  std::initializer_list<std::pair<double, double>> points)
{
  automsgs::msgs::nav_msgs::Path path;
  for (const auto & point : points) {
    *path.add_poses() = MakePose(point.first, point.second);
  }
  return path;
}

}  // namespace

TEST(PathSimplifier, ReturnsUnchangedWhenEpsilonNonPositive)
{
  const auto path = MakePath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  const auto simplified = autonomy::planning::utils::SimplifyPath(path, 0.0);
  EXPECT_EQ(path.poses_size(), simplified.poses_size());
}

TEST(PathSimplifier, ReducesCollinearPoints)
{
  const auto path = MakePath(
    {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
  const auto simplified = autonomy::planning::utils::SimplifyPath(path, 0.05);
  ASSERT_GE(simplified.poses_size(), 2);
  EXPECT_LE(simplified.poses_size(), path.poses_size());
  EXPECT_DOUBLE_EQ(simplified.poses(0).pose().position().x(), 0.0);
  EXPECT_DOUBLE_EQ(
    simplified.poses(simplified.poses_size() - 1).pose().position().x(), 3.0);
}

TEST(PathSimplifier, KeepsCornerPoint)
{
  const auto path = MakePath(
    {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {1.0, 2.0}});
  const auto simplified = autonomy::planning::utils::SimplifyPath(path, 0.05);
  ASSERT_EQ(simplified.poses_size(), 3);
  EXPECT_DOUBLE_EQ(simplified.poses(1).pose().position().x(), 1.0);
  EXPECT_DOUBLE_EQ(simplified.poses(1).pose().position().y(), 0.0);
}
