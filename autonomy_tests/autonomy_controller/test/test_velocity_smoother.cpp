/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/control/proto/smoother_options.pb.h"
#include "autonomy/control/utils/velocity_smoother.hpp"

using autonomy::control::proto::VelocitySmootherOptions;
using autonomy::control::utils::VelocitySmoother;

namespace {

VelocitySmoother MakeSmoother(double freq = 20.0)
{
  VelocitySmootherOptions opts;
  opts.set_smoothing_frequency(freq);
  opts.add_max_velocity(0.5);
  opts.add_max_velocity(0.0);
  opts.add_max_velocity(2.5);
  opts.add_min_velocity(-0.5);
  opts.add_min_velocity(0.0);
  opts.add_min_velocity(-2.5);
  opts.add_max_accel(2.5);
  opts.add_max_accel(0.0);
  opts.add_max_accel(3.2);
  opts.add_max_decel(-2.5);
  opts.add_max_decel(0.0);
  opts.add_max_decel(-3.2);
  return VelocitySmoother(opts);
}

}  // namespace

TEST(VelocitySmoother, FindEtaNoConstraintWhenWithinAccel)
{
  auto smoother = MakeSmoother(20.0);
  // dv = 0.05, accel/freq = 2.5/20 = 0.125 → within limits → -1
  EXPECT_DOUBLE_EQ(smoother.findEtaConstraint(0.0, 0.05, 2.5, -2.5), -1.0);
}

TEST(VelocitySmoother, FindEtaScalesWhenAccelExceeded)
{
  auto smoother = MakeSmoother(20.0);
  // dv = 0.5, max step = 0.125 → eta = 0.125/0.5 = 0.25
  EXPECT_NEAR(smoother.findEtaConstraint(0.0, 0.5, 2.5, -2.5), 0.25, 1e-9);
}

TEST(VelocitySmoother, ApplyConstraintsClampsDelta)
{
  auto smoother = MakeSmoother(20.0);
  const double out = smoother.applyConstraints(0.0, 0.5, 2.5, -2.5, 1.0);
  EXPECT_NEAR(out, 0.125, 1e-9);
}
