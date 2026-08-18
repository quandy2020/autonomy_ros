/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/control/utils/conversions.hpp"
#include "autonomy_controller/test_helpers.hpp"

using autonomy::control::utils::twist2Dto3D;
using autonomy::control::utils::twist3Dto2D;
using autonomy_controller::test::MakeTwist;

TEST(ControlConversions, Twist2Dto3DRoundTrip)
{
  automsgs::msgs::geometry_msgs::Twist2D in;
  in.set_x(0.4f);
  in.set_y(-0.1f);
  in.set_theta(1.2f);

  const auto twist3 = twist2Dto3D(in);
  EXPECT_NEAR(twist3.linear().x(), 0.4, 1e-6);
  EXPECT_NEAR(twist3.linear().y(), -0.1, 1e-6);
  EXPECT_NEAR(twist3.angular().z(), 1.2, 1e-6);

  const auto back = twist3Dto2D(twist3);
  EXPECT_NEAR(back.x(), in.x(), 1e-6);
  EXPECT_NEAR(back.y(), in.y(), 1e-6);
  EXPECT_NEAR(back.theta(), in.theta(), 1e-6);
}

TEST(ControlConversions, Twist3Dto2DDropsUnusedAxes)
{
  auto twist = MakeTwist(0.5, 0.2, -0.3);
  twist.mutable_linear()->set_z(9.0);
  twist.mutable_angular()->set_x(1.0);
  twist.mutable_angular()->set_y(2.0);

  const auto t2 = twist3Dto2D(twist);
  EXPECT_NEAR(t2.x(), 0.5, 1e-6);
  EXPECT_NEAR(t2.y(), 0.2, 1e-6);
  EXPECT_NEAR(t2.theta(), -0.3, 1e-6);
}
