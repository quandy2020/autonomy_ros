// Copyright 2025 autonomy_ros contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "autonomy_msgs/msg/task_type.hpp"
#include "autonomy_ros/task/task_muxer.hpp"

using autonomy_msgs::msg::TaskType;
using autonomy_ros::task::TaskMuxer;

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
  muxer.acquire("tour", TaskType::GUIDED_TOUR);
  EXPECT_EQ(
    muxer.acquire("nav", TaskType::NAVIGATION),
    TaskMuxer::AcquireResult::RejectedBusy);
}

TEST(TaskMuxer, PreemptWithTeleop)
{
  TaskMuxer muxer;
  muxer.acquire("tour", TaskType::GUIDED_TOUR);
  EXPECT_EQ(
    muxer.acquire("teleop", TaskType::TELEOP, true),
    TaskMuxer::AcquireResult::PreemptedPrevious);
  EXPECT_TRUE(muxer.ownsTask("teleop"));
  EXPECT_FALSE(muxer.ownsTask("tour"));
}

TEST(TaskMuxer, CanAcquireWithoutMutation)
{
  TaskMuxer muxer;
  muxer.acquire("dock", TaskType::DOCK);
  EXPECT_FALSE(muxer.canAcquire("nav", TaskType::NAVIGATION));
  EXPECT_TRUE(muxer.ownsTask("dock"));
}
