/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/control/checker/pose_progress_checker.hpp"
#include "autonomy/control/checker/simple_progress_checker.hpp"
#include "autonomy_controller/test_helpers.hpp"

using autonomy::control::checker::PoseProgressChecker;
using autonomy::control::checker::SimpleProgressChecker;
using autonomy_controller::test::MakePoseStamped;

TEST(ProgressCheckers, SimpleProgressFirstPoseSetsBaseline)
{
  SimpleProgressChecker checker;
  checker.Initialize("progress");

  auto pose = MakePoseStamped(0.0, 0.0);
  EXPECT_TRUE(checker.Check(pose));

  // Still within default radius 0.5 → fail (no progress).
  pose = MakePoseStamped(0.1, 0.0);
  EXPECT_FALSE(checker.Check(pose));

  // Moved beyond radius → pass and reset baseline.
  pose = MakePoseStamped(1.0, 0.0);
  EXPECT_TRUE(checker.Check(pose));
}

TEST(ProgressCheckers, SimpleProgressResetClearsBaseline)
{
  SimpleProgressChecker checker;
  checker.Initialize("progress");

  auto pose = MakePoseStamped(0.0, 0.0);
  ASSERT_TRUE(checker.Check(pose));
  pose = MakePoseStamped(0.1, 0.0);
  ASSERT_FALSE(checker.Check(pose));

  checker.Reset();
  pose = MakePoseStamped(0.1, 0.0);
  EXPECT_TRUE(checker.Check(pose));
}

TEST(ProgressCheckers, PoseProgressCheckerConstructs)
{
  PoseProgressChecker checker;
  checker.Initialize("pose_progress");
  auto pose = MakePoseStamped(0.0, 0.0, 0.0);
  EXPECT_TRUE(checker.Check(pose));
}
