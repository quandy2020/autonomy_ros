/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
#include "autonomy/control/controller/mppi_controller/motion_models.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"

using autonomy::control::controller::mppi_controller::AckermannMotionModel;
using autonomy::control::controller::mppi_controller::DiffDriveMotionModel;
using autonomy::control::controller::mppi_controller::OmniMotionModel;
using autonomy::control::controller::mppi_controller::models::ControlSequence;

TEST(MotionModels, HolonomicFlags)
{
  DiffDriveMotionModel diff;
  OmniMotionModel omni;
  EXPECT_FALSE(diff.isHolonomic());
  EXPECT_TRUE(omni.isHolonomic());
}

TEST(MotionModels, AckermannDefaultTurningRadius)
{
  AckermannMotionModel ackermann(nullptr, "ackermann");
  EXPECT_NEAR(ackermann.getMinTurningRadius(), 0.2f, 1e-6f);
  EXPECT_FALSE(ackermann.isHolonomic());
}

TEST(MotionModels, AckermannConstraintsClampWz)
{
  autonomy::control::proto::MPPIControllerOptions opts;
  opts.mutable_ackermann_constraints()->set_min_turning_r(1.0f);
  AckermannMotionModel ackermann(&opts, "ackermann");

  ControlSequence seq;
  seq.vx = Eigen::ArrayXf::Constant(4, 1.0f);
  seq.wz = Eigen::ArrayXf::Constant(4, 5.0f);
  ackermann.applyConstraints(seq);

  for (int i = 0; i < seq.wz.size(); ++i) {
    EXPECT_NEAR(seq.wz(i), 1.0f, 1e-5f);
  }
}
