/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <cmath>
#include <gtest/gtest.h>

#include "autonomy/control/utils/controller_utils.hpp"
#include "autonomy_controller/test_helpers.hpp"

using autonomy::control::utils::CircleSegmentIntersection;
using autonomy::control::utils::GetLookAheadPoint;
using autonomy::control::utils::LinearInterpolation;
using autonomy_controller::test::MakePoint;
using autonomy_controller::test::MakeStraightPath;

TEST(ControllerUtils, LinearInterpolationMidpoint)
{
  const auto p1 = MakePoint(0.0, 0.0);
  const auto p2 = MakePoint(4.0, 0.0);
  const auto mid = LinearInterpolation(p1, p2, 2.0);
  EXPECT_NEAR(mid.x, 2.0, 1e-9);
  EXPECT_NEAR(mid.y, 0.0, 1e-9);
}

TEST(ControllerUtils, CircleSegmentIntersectionOnXAxis)
{
  const auto p1 = MakePoint(0.0, 0.0);
  const auto p2 = MakePoint(2.0, 0.0);
  const auto hit = CircleSegmentIntersection(p1, p2, 1.0);
  EXPECT_NEAR(hit.x, 1.0, 1e-6);
  EXPECT_NEAR(hit.y, 0.0, 1e-6);
}

TEST(ControllerUtils, LookAheadAlongStraightPath)
{
  const auto path = MakeStraightPath(0.0, 0.0, 5.0, 0.0, 11);
  double lookahead = 2.0;
  const auto pose = GetLookAheadPoint(lookahead, path, false);
  EXPECT_NEAR(pose.pose.position.x, 2.0, 0.15);
  EXPECT_NEAR(pose.pose.position.y, 0.0, 1e-6);
}

TEST(ControllerUtils, LookAheadBeyondPathUsesGoal)
{
  const auto path = MakeStraightPath(0.0, 0.0, 1.0, 0.0, 5);
  double lookahead = 10.0;
  const auto pose = GetLookAheadPoint(lookahead, path, false);
  EXPECT_NEAR(pose.pose.position.x, 1.0, 0.15);
}
