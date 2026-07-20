/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>
#include <string>

#include "autonomy/control/constants.hpp"

TEST(ControlConstants, ActionAndNodeNames)
{
  EXPECT_STREQ(autonomy::control::kControllerServerNodeName, "controller_server");
  EXPECT_STREQ(autonomy::control::kFollowPathActionName, "follow_path");
  EXPECT_STREQ(autonomy::control::kSpinActionName, "spin");
  EXPECT_STREQ(autonomy::control::kBackUpActionName, "backup");
  EXPECT_STREQ(autonomy::control::kWaitActionName, "wait");
  EXPECT_FALSE(std::string(autonomy::control::kFollowPathActionName).empty());
}
