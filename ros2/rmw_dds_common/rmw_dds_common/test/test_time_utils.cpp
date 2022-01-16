// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <climits>

#include "rmw_dds_common/time_utils.hpp"

static bool
operator==(rmw_time_t t1, rmw_time_t t2)
{
  return t1.sec == t2.sec && t1.nsec == t2.nsec;
}

TEST(test_time_utils, test_unmodified_zeros)
{
  const rmw_time_t zeros {0, 0};
  auto t1 = rmw_dds_common::clamp_rmw_time_to_dds_time(zeros);
  EXPECT_TRUE(t1 == zeros);
}

TEST(test_time_utils, test_unmodified_max)
{
  const rmw_time_t max_dds_time {0x7FFFFFFF, 0xFFFFFFFF};
  auto t2 = rmw_dds_common::clamp_rmw_time_to_dds_time(max_dds_time);
  EXPECT_TRUE(t2 == max_dds_time);
}

TEST(test_time_utils, test_seconds_overflow)
{
  const rmw_time_t slightly_too_large {2147483651, 0};
  auto t3 = rmw_dds_common::clamp_rmw_time_to_dds_time(slightly_too_large);
  const rmw_time_t result3 {0x7FFFFFFF, 4000000000};
  EXPECT_TRUE(t3 == result3);
}

TEST(test_time_utils, test_saturation)
{
  const rmw_time_t max_64 {LLONG_MAX, ULLONG_MAX};
  const rmw_time_t max_dds_time {0x7FFFFFFF, 0xFFFFFFFF};
  auto t4 = rmw_dds_common::clamp_rmw_time_to_dds_time(max_64);
  EXPECT_TRUE(t4 == max_dds_time);
}
