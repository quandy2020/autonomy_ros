/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>
#include <memory>

#include "autonomy/control/checker/position_goal_checker.hpp"
#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/checker/stopped_goal_checker.hpp"
#include "autonomy_controller/test_helpers.hpp"

using autonomy::control::checker::PositionGoalChecker;
using autonomy::control::checker::SimpleGoalChecker;
using autonomy::control::checker::StoppedGoalChecker;
using autonomy_controller::test::MakePose;
using autonomy_controller::test::MakeTwist;

TEST(GoalCheckers, SimpleGoalReachedWithinTolerance)
{
  SimpleGoalChecker checker;
  checker.Initialize("simple", nullptr);
  checker.SetTolerances(0.25, 0.25, false);

  const auto goal = MakePose(1.0, 2.0, 0.0);
  const auto near = MakePose(1.1, 2.05, 0.05);
  const auto far = MakePose(2.0, 2.0, 0.0);
  const auto stop = MakeTwist(0.0, 0.0, 0.0);

  EXPECT_TRUE(checker.IsGoalReached(near, goal, stop));
  EXPECT_FALSE(checker.IsGoalReached(far, goal, stop));
}

TEST(GoalCheckers, SimpleGoalStatefulSkipsXyAfterFirstHit)
{
  SimpleGoalChecker checker;
  checker.Initialize("simple", nullptr);
  checker.SetTolerances(0.25, 0.25, true);

  const auto goal = MakePose(0.0, 0.0, 0.0);
  const auto inside = MakePose(0.1, 0.0, 0.0);
  const auto outside = MakePose(1.0, 0.0, 0.0);
  const auto stop = MakeTwist(0.0, 0.0, 0.0);

  ASSERT_TRUE(checker.IsGoalReached(inside, goal, stop));
  // After XY locked, only yaw is checked — far XY still passes if yaw ok.
  EXPECT_TRUE(checker.IsGoalReached(outside, goal, stop));

  checker.Reset();
  EXPECT_FALSE(checker.IsGoalReached(outside, goal, stop));
}

TEST(GoalCheckers, PositionGoalIgnoresYaw)
{
  PositionGoalChecker checker;
  checker.Initialize("position", nullptr);
  checker.SetXYGoalTolerance(0.3);

  const auto goal = MakePose(0.0, 0.0, 0.0);
  const auto near_bad_yaw = MakePose(0.1, 0.0, 1.5);
  const auto stop = MakeTwist(0.0, 0.0, 0.0);

  EXPECT_TRUE(checker.IsGoalReached(near_bad_yaw, goal, stop));
}

TEST(GoalCheckers, StoppedGoalRequiresZeroVelocity)
{
  StoppedGoalChecker checker;
  checker.Initialize("stopped", nullptr);

  const auto goal = MakePose(0.0, 0.0, 0.0);
  const auto near = MakePose(0.05, 0.0, 0.0);
  const auto moving = MakeTwist(0.5, 0.0, 0.0);
  const auto stop = MakeTwist(0.0, 0.0, 0.0);

  EXPECT_FALSE(checker.IsGoalReached(near, goal, moving));
  EXPECT_TRUE(checker.IsGoalReached(near, goal, stop));
}
