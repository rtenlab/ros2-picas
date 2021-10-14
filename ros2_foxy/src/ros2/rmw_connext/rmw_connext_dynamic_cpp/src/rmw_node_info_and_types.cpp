// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
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
#include "rmw/allocators.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"

#include "rmw_connext_shared_cpp/node_info_and_types.hpp"
#include "rmw_connext_cpp/identifier.hpp"

extern "C"
{
rmw_ret_t
get_subscriber_names_and_types_by_node(
        const rmw_node_t * node,
        rcutils_allocator_t * allocator,
        const char * node_name,
        const char * node_namespace,
        bool no_demangle,
        rmw_names_and_types_t * topic_names_and_types)
{
  return rmw_connext_shared_cpp::get_subscriber_names_and_types_by_node(
          rti_connext_identifier, node, allocator, node_name, node_namespace, no_demangle, topic_names_and_types);
}

rmw_ret_t
get_publisher_names_and_types_by_node(
        const rmw_node_t * node,
        rcutils_allocator_t * allocator,
        const char * node_name,
        const char * node_namespace,
        bool no_demangle,
        rmw_names_and_types_t * topic_names_and_types)
{
  return rmw_connext_shared_cpp::get_publisher_names_and_types_by_node(
          rti_connext_identifier, node, allocator, node_name, node_namespace, no_demangle, topic_names_and_types);
}

rmw_ret_t
get_service_names_and_types_by_node(
        const rmw_node_t * node,
        rcutils_allocator_t * allocator,
        const char * node_name,
        const char * node_namespace,
        rmw_names_and_types_t * service_names_and_types)
{
  return rmw_connext_shared_cpp::get_service_names_and_types_by_node(
          rti_connext_identifier, node, allocator, node_name, node_namespace, service_names_and_types);
}

rmw_ret_t
get_client_names_and_types_by_node(
        const rmw_node_t * node,
        rcutils_allocator_t * allocator,
        const char * node_name,
        const char * node_namespace,
        rmw_names_and_types_t * service_names_and_types)
{
  return rmw_connext_shared_cpp::get_client_names_and_types_by_node(
          rti_connext_identifier, node, allocator, node_name, node_namespace, service_names_and_types);
}
}  // extern "C"
