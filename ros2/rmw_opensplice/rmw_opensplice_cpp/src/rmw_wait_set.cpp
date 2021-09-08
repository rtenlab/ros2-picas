// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include "types.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init context,
    context->implementation_identifier,
    opensplice_cpp_identifier,
    // TODO(wjwwood): replace this with RMW_RET_INCORRECT_RMW_IMPLEMENTATION when refactored
    return nullptr);

  rmw_wait_set_t * wait_set = rmw_wait_set_allocate();
  OpenSpliceWaitSetInfo * wait_set_info = nullptr;

  // From here onward, error results in unrolling in the goto fail block.
  if (!wait_set) {
    RMW_SET_ERROR_MSG("failed to allocate wait set");
    goto fail;
  }
  wait_set->implementation_identifier = opensplice_cpp_identifier;
  wait_set->data = rmw_allocate(sizeof(OpenSpliceWaitSetInfo));

  wait_set_info = static_cast<OpenSpliceWaitSetInfo *>(wait_set->data);

  if (!wait_set_info) {
    RMW_SET_ERROR_MSG("failed to allocate wait set implementation struct");
    goto fail;
  }

  wait_set_info->wait_set = static_cast<DDS::WaitSet *>(rmw_allocate(sizeof(DDS::WaitSet)));
  if (!wait_set_info->wait_set) {
    RMW_SET_ERROR_MSG("failed to allocate wait set");
    goto fail;
  }

  RMW_TRY_PLACEMENT_NEW(
    wait_set_info->wait_set, wait_set_info->wait_set, goto fail, DDS::WaitSet, )

  // Now allocate storage for the ConditionSeq objects
  wait_set_info->active_conditions =
    static_cast<DDS::ConditionSeq *>(rmw_allocate(sizeof(DDS::ConditionSeq)));
  if (!wait_set_info->active_conditions) {
    RMW_SET_ERROR_MSG("failed to allocate active_conditions sequence");
    goto fail;
  }

  wait_set_info->attached_conditions =
    static_cast<DDS::ConditionSeq *>(rmw_allocate(sizeof(DDS::ConditionSeq)));
  if (!wait_set_info->attached_conditions) {
    RMW_SET_ERROR_MSG("failed to allocate attached_conditions sequence");
    goto fail;
  }

  // If max_conditions is greater than zero, re-allocate both ConditionSeqs to max_conditions
  if (max_conditions > 0) {
    RMW_TRY_PLACEMENT_NEW(
      wait_set_info->active_conditions, wait_set_info->active_conditions, goto fail,
      DDS::ConditionSeq, static_cast<DDS::ULong>(max_conditions))

    RMW_TRY_PLACEMENT_NEW(
      wait_set_info->attached_conditions, wait_set_info->attached_conditions, goto fail,
      DDS::ConditionSeq,
      static_cast<DDS::ULong>(max_conditions))
  } else {
    // Else, don't preallocate: the vectors will size dynamically when rmw_wait is called.
    // Default-construct the ConditionSeqs.
    RMW_TRY_PLACEMENT_NEW(
      wait_set_info->active_conditions, wait_set_info->active_conditions, goto fail,
      DDS::ConditionSeq, )

    RMW_TRY_PLACEMENT_NEW(
      wait_set_info->attached_conditions, wait_set_info->attached_conditions, goto fail,
      DDS::ConditionSeq, )
  }

  return wait_set;

fail:
  if (wait_set_info) {
    if (wait_set_info->active_conditions) {
      // How to know which constructor threw?
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        wait_set_info->active_conditions->~DDS_DCPSUObjSeq(), ConditionSeq)
      rmw_free(wait_set_info->active_conditions);
    }
    if (wait_set_info->attached_conditions) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        wait_set_info->attached_conditions->~DDS_DCPSUObjSeq(), ConditionSeq)
      rmw_free(wait_set_info->attached_conditions);
    }
    if (wait_set_info->wait_set) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(wait_set_info->wait_set->~WaitSet(), WaitSet)
      rmw_free(wait_set_info->wait_set);
    }
    wait_set_info = nullptr;
  }
  if (wait_set) {
    if (wait_set->data) {
      rmw_free(wait_set->data);
    }
    rmw_wait_set_free(wait_set);
  }
  return nullptr;
}

rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  if (!wait_set) {
    RMW_SET_ERROR_MSG("wait set handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set handle,
    wait_set->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  auto result = RMW_RET_OK;
  OpenSpliceWaitSetInfo * wait_set_info = static_cast<OpenSpliceWaitSetInfo *>(wait_set->data);

  // Explicitly call destructor since the "placement new" was used

  using DDS::ConditionSeq;
  if (wait_set_info->active_conditions) {
    RMW_TRY_DESTRUCTOR(
      wait_set_info->active_conditions->~ConditionSeq(), ConditionSeq, result = RMW_RET_ERROR)
    rmw_free(wait_set_info->active_conditions);
  }
  if (wait_set_info->attached_conditions) {
    RMW_TRY_DESTRUCTOR(
      wait_set_info->attached_conditions->~ConditionSeq(), ConditionSeq, result = RMW_RET_ERROR)
    rmw_free(wait_set_info->attached_conditions);
  }
  if (wait_set_info->wait_set) {
    RMW_TRY_DESTRUCTOR(wait_set_info->wait_set->~WaitSet(), WaitSet, result = RMW_RET_ERROR)
    rmw_free(wait_set_info->wait_set);
    wait_set_info = nullptr;
  }
  if (wait_set->data) {
    rmw_free(wait_set->data);
  }
  if (wait_set) {
    rmw_wait_set_free(wait_set);
  }
  return result;
}
}  // extern "C"
