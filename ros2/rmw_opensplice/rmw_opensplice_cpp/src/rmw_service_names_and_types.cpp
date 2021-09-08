// Copyright 2014-2017 Open Source Robotics Foundation, Inc.
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

#include <set>
#include <map>
#include <string>

#include "rcutils/allocator.h"
#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/names_and_types.h"
#include "rmw/types.h"

#include "identifier.hpp"
#include "types.hpp"
#include "demangle.hpp"
#include "names_and_types_helpers.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_ret_t
rmw_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types)
{
  rmw_ret_t ret = validate_node(node, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = rmw_names_and_types_check_zero(service_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  // combine publisher and subscriber information
  std::map<std::string, std::set<std::string>> services;
  node_info->publisher_listener->fill_service_names_and_types(services);
  node_info->subscriber_listener->fill_service_names_and_types(services);

  rmw_ret_t rmw_ret;
  rmw_ret = copy_services_to_names_and_types(services, allocator, service_names_and_types);
  if (rmw_ret != RMW_RET_OK) {
    return rmw_ret;
  }
  return RMW_RET_OK;
}
}  // extern "C"
