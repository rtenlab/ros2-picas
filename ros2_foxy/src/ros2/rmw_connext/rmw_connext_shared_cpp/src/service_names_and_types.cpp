// Copyright 2015-2017 Open Source Robotics Foundation, Inc.
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

#include <map>
#include <set>
#include <string>

#include "rcutils/allocator.h"
#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"
#include "rcutils/types/string_array.h"

#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"

#include "rmw_connext_shared_cpp/demangle.hpp"
#include "rmw_connext_shared_cpp/names_and_types_helpers.hpp"
#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw_connext_shared_cpp/service_names_and_types.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

rmw_ret_t
get_service_names_and_types(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_names_and_types_check_zero(service_names_and_types)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);

  // combine publisher and subscriber information
  std::map<std::string, std::set<std::string>> services;
  node_info->publisher_listener->fill_service_names_and_types(services);
  node_info->subscriber_listener->fill_service_names_and_types(services);

  // Fill out service_names_and_types
  if (!services.empty()) {
    rmw_ret_t rmw_ret =
      copy_services_to_names_and_types(services, allocator, service_names_and_types);
    if (rmw_ret != RMW_RET_OK) {
      return rmw_ret;
    }
  }

  return RMW_RET_OK;
}
