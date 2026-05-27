// Copyright 2025 autonomy_ros contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "autonomy_msgs/msg/task_type.hpp"
#include "autonomy_ros/navigation/task_muxer.hpp"

using autonomy_msgs::msg::TaskType;
using autonomy_ros::navigation::TaskMuxer;

TEST(TaskMuxer, AcquireAndRelease)
{
  TaskMuxer muxer;
  EXPECT_EQ(
    muxer.acquire("a", TaskType::NAVIGATION),
    TaskMuxer::AcquireResult::Acquired);
  EXPECT_TRUE(muxer.ownsTask("a"));
  muxer.release("a");
  EXPECT_FALSE(muxer.hasActiveTask());
}

TEST(TaskMuxer, RejectLowerPriority)
{
  TaskMuxer muxer;
  muxer.acquire("through", TaskType::WAYPOINTS);
  EXPECT_EQ(
    muxer.acquire("nav", TaskType::NAVIGATION),
    TaskMuxer::AcquireResult::RejectedBusy);
}

TEST(TaskMuxer, PreemptWithForceFlag)
{
  TaskMuxer muxer;
  muxer.acquire("through", TaskType::WAYPOINTS);
  EXPECT_EQ(
    muxer.acquire("nav", TaskType::NAVIGATION, true),
    TaskMuxer::AcquireResult::PreemptedPrevious);
  EXPECT_TRUE(muxer.ownsTask("nav"));
}

TEST(TaskMuxer, CanAcquireWithoutMutation)
{
  TaskMuxer muxer;
  muxer.acquire("through", TaskType::WAYPOINTS);
  EXPECT_FALSE(muxer.canAcquire("nav", TaskType::NAVIGATION));
  EXPECT_TRUE(muxer.ownsTask("through"));
}
