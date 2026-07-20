/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <cmath>
#include <gtest/gtest.h>

#include "autonomy/control/controller/graceful_controller/smooth_control_law.hpp"
#include "autonomy_controller/test_helpers.hpp"

using autonomy::control::controller::SmoothControlLaw;
using autonomy_controller::test::MakePose;

TEST(SmoothControlLaw, ForwardTowardTargetProducesPositiveVx)
{
  SmoothControlLaw law(
    /*k_phi=*/1.0, /*k_delta=*/2.0, /*beta=*/0.4, /*lambda=*/2.0,
    /*slowdown_radius=*/0.5, /*deceleration_max=*/2.5,
    /*v_linear_min=*/0.1, /*v_linear_max=*/0.5, /*v_angular_max=*/1.0);

  const auto current = MakePose(0.0, 0.0, 0.0);
  const auto target = MakePose(2.0, 0.0, 0.0);
  const auto cmd = law.CalculateRegularVelocity(target, current, false);

  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_LE(std::fabs(cmd.linear.x), 0.5 + 1e-6);
  EXPECT_LE(std::fabs(cmd.angular.z), 1.0 + 1e-6);
}

TEST(SmoothControlLaw, NextPoseAdvancesTowardTarget)
{
  SmoothControlLaw law(1.0, 2.0, 0.4, 2.0, 0.5, 2.5, 0.1, 0.5, 1.0);
  const auto current = MakePose(0.0, 0.0, 0.0);
  const auto target = MakePose(3.0, 0.0, 0.0);
  const auto next = law.CalculateNextPose(0.1, target, current, false);

  EXPECT_GT(next.position.x, current.position.x);
}

TEST(SmoothControlLaw, SpeedLimitUpdateIsRespected)
{
  SmoothControlLaw law(1.0, 2.0, 0.4, 2.0, 0.5, 2.5, 0.05, 1.0, 2.0);
  law.SetSpeedLimit(0.05, 0.2, 0.5);

  const auto current = MakePose(0.0, 0.0, 0.0);
  const auto target = MakePose(5.0, 0.0, 0.0);
  const auto cmd = law.CalculateRegularVelocity(target, current, false);
  EXPECT_LE(std::fabs(cmd.linear.x), 0.2 + 1e-6);
  EXPECT_LE(std::fabs(cmd.angular.z), 0.5 + 1e-6);
}
