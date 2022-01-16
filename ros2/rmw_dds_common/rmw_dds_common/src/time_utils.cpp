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

#include <climits>

#include "rmw/types.h"
#include "rcutils/logging_macros.h"

#include "rmw_dds_common/time_utils.hpp"

rmw_time_t
rmw_dds_common::clamp_rmw_time_to_dds_time(const rmw_time_t & time)
{
  rmw_time_t t = time;

  // Let's try to keep the seconds value representable with 32-bits
  // by moving the excess seconds over to the nanoseconds field.
  if (t.sec > INT_MAX) {
    uint64_t diff = t.sec - INT_MAX;
    t.sec -= diff;
    t.nsec += diff * (1000LL * 1000LL * 1000LL);
  }

  // In case the nanoseconds value is too large for a 32-bit unsigned,
  // we can at least saturate and emit a warning
  if (t.nsec > UINT_MAX) {
    RCUTILS_LOG_WARN_NAMED(
      "rmw_dds_common",
      "nanoseconds value too large for 32-bits, saturated at UINT_MAX");
    t.nsec = UINT_MAX;
  }

  return t;
}
