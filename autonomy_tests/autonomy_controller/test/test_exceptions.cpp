/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>
#include <stdexcept>
#include <string>

#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/proto/controller_options.pb.h"

using autonomy::control::common::ControllerException;
using autonomy::control::common::ControllerTFError;
using autonomy::control::common::ControllerTimedOut;
using autonomy::control::common::FailedToMakeProgress;
using autonomy::control::common::InvalidController;
using autonomy::control::common::InvalidPath;
using autonomy::control::common::NoValidControl;
using autonomy::control::common::PatienceExceeded;

TEST(ControlExceptions, HierarchyAndMessage)
{
  try {
    throw InvalidPath("empty path");
  } catch (const ControllerException & ex) {
    EXPECT_STREQ(ex.what(), "empty path");
  }

  EXPECT_THROW(throw InvalidController("bad"), InvalidController);
  EXPECT_THROW(throw ControllerTFError("tf"), ControllerTFError);
  EXPECT_THROW(throw FailedToMakeProgress("stuck"), FailedToMakeProgress);
  EXPECT_THROW(throw PatienceExceeded("timeout"), PatienceExceeded);
  EXPECT_THROW(throw NoValidControl("none"), NoValidControl);
  EXPECT_THROW(throw ControllerTimedOut("dt"), ControllerTimedOut);
}

TEST(ControlProto, ResultCodeValues)
{
  using autonomy::control::proto::CONTROLLER_RESULT_FAILURE;
  using autonomy::control::proto::CONTROLLER_RESULT_SUCCESS;
  EXPECT_EQ(CONTROLLER_RESULT_SUCCESS, 0);
  EXPECT_EQ(CONTROLLER_RESULT_FAILURE, 100);
}
