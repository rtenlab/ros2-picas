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

#ifndef RMW_CONNEXT_SHARED_CPP__NODE_NAMES_HPP_
#define RMW_CONNEXT_SHARED_CPP__NODE_NAMES_HPP_

#include "rmw/names_and_types.h"
#include "rmw/types.h"

#include "rmw_connext_shared_cpp/visibility_control.h"

RMW_CONNEXT_SHARED_CPP_PUBLIC
rmw_ret_t
get_node_names(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces);

RMW_CONNEXT_SHARED_CPP_PUBLIC
rmw_ret_t
count_publishers(
  const char * implementation_identifier,
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count);

RMW_CONNEXT_SHARED_CPP_PUBLIC
rmw_ret_t
count_subscribers(
  const char * implementation_identifier,
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count);

RMW_CONNEXT_SHARED_CPP_PUBLIC
const rmw_guard_condition_t *
node_get_graph_guard_condition(const rmw_node_t * node);

#endif  // RMW_CONNEXT_SHARED_CPP__NODE_NAMES_HPP_
