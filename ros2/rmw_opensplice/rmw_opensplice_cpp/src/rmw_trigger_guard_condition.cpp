// Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

#ifdef __clang__
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wmismatched-tags"
#endif
#if defined(_MSC_VER)
# pragma warning(push)
# pragma warning(disable: 4099)
#endif
#include <ccpp_dds_dcps.h>
#if defined(_MSC_VER)
# pragma warning(pop)
#endif
#ifdef __clang__
# pragma GCC diagnostic pop
#endif
#include <dds_dcps.h>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"

#include "identifier.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition)
{
  if (!guard_condition) {
    RMW_SET_ERROR_MSG("guard condition handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard condition handle,
    guard_condition->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  DDS::GuardCondition * dds_guard_condition =
    static_cast<DDS::GuardCondition *>(guard_condition->data);
  if (!dds_guard_condition) {
    RMW_SET_ERROR_MSG("guard condition is null");
    return RMW_RET_ERROR;
  }
  if (dds_guard_condition->set_trigger_value(true) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to set trigger value to true");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}
}  // extern "C"
