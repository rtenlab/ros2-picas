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

#include <rcpputils/get_env.hpp>

#include <string>

/* Tests get_env_var.
 *
 * Expected environment variables must be set by the calling code:
 *
 *   - EMPTY_TEST=
 *   - NORMAL_TEST=foo
 *
 * These are set in the call to `ament_add_gtest()` in the `CMakeLists.txt`.
 */

TEST(TestGetEnv, test_get_env) {
  std::string env;
  env = rcpputils::get_env_var("NORMAL_TEST");
  EXPECT_STREQ("foo", env.c_str());
  env = rcpputils::get_env_var("SHOULD_NOT_EXIST_TEST");
  EXPECT_STREQ("", env.c_str());
  env = rcpputils::get_env_var("EMPTY_TEST");
  EXPECT_STREQ("", env.c_str());
}
