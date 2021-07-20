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
rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init context,
    context->implementation_identifier,
    opensplice_cpp_identifier,
    // TODO(wjwwood): replace this with RMW_RET_INCORRECT_RMW_IMPLEMENTATION when refactored
    return NULL);
  rmw_guard_condition_t * guard_condition = rmw_guard_condition_allocate();
  if (!guard_condition) {
    RMW_SET_ERROR_MSG("failed to allocate guard condition");
    goto fail;
  }
  guard_condition->implementation_identifier = opensplice_cpp_identifier;
  guard_condition->data = rmw_allocate(sizeof(DDS::GuardCondition));
  if (!guard_condition->data) {
    RMW_SET_ERROR_MSG("failed to allocate dds guard condition");
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(
    guard_condition->data, guard_condition->data, goto fail, DDS::GuardCondition, )
  return guard_condition;
fail:
  if (guard_condition->data) {
    // The allocation succeeded but the constructor threw, so deallocate.
    rmw_free(guard_condition->data);
  }
  if (guard_condition) {
    rmw_guard_condition_free(guard_condition);
  }
  return nullptr;
}

rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition)
{
  if (!guard_condition) {
    RMW_SET_ERROR_MSG("guard condition handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard condition handle,
    guard_condition->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  auto result = RMW_RET_OK;
  DDS::GuardCondition * dds_guard_condition =
    static_cast<DDS::GuardCondition *>(guard_condition->data);
  // Explicitly call destructor since the "placement new" was used
  RMW_TRY_DESTRUCTOR(
    dds_guard_condition->~GuardCondition(), GuardCondition, result = RMW_RET_ERROR)
  rmw_free(guard_condition->data);
  rmw_guard_condition_free(guard_condition);
  return result;
}
}  // extern "C"
