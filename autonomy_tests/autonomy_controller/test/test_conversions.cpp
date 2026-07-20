/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/utils/conversions.hpp"
#include "autonomy_controller/test_helpers.hpp"

using autonomy::control::utils::twist2Dto3D;
using autonomy::control::utils::twist3Dto2D;
using autonomy_controller::test::MakeTwist;

TEST(ControlConversions, Twist2Dto3DRoundTrip)
{
  autonomy::commsgs::geometry_msgs::Twist2D in;
  in.x = 0.4;
  in.y = -0.1;
  in.theta = 1.2;

  const auto twist3 = twist2Dto3D(in);
  EXPECT_DOUBLE_EQ(twist3.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(twist3.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(twist3.angular.z, 1.2);

  const auto back = twist3Dto2D(twist3);
  EXPECT_DOUBLE_EQ(back.x, in.x);
  EXPECT_DOUBLE_EQ(back.y, in.y);
  EXPECT_DOUBLE_EQ(back.theta, in.theta);
}

TEST(ControlConversions, Twist3Dto2DDropsUnusedAxes)
{
  auto twist = MakeTwist(0.5, 0.2, -0.3);
  twist.linear.z = 9.0;
  twist.angular.x = 1.0;
  twist.angular.y = 2.0;

  const auto t2 = twist3Dto2D(twist);
  EXPECT_DOUBLE_EQ(t2.x, 0.5);
  EXPECT_DOUBLE_EQ(t2.y, 0.2);
  EXPECT_DOUBLE_EQ(t2.theta, -0.3);
}
