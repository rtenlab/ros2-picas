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

#include "rcutils/format_string.h"
#include "rcutils/types.h"
#include "rcutils/split.h"

#include "rmw/rmw.h"
#include "rmw/allocators.h"
#include "rmw/error_handling.h"

#include "rmw_connext_shared_cpp/namespace_prefix.hpp"
#include "rmw_connext_shared_cpp/ndds_include.hpp"

bool
_process_topic_name(
  const char * topic_name,
  bool avoid_ros_namespace_conventions,
  char ** topic_str)
{
  bool success = true;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  const char * topic_prefix = "";
  char * concat_str = nullptr;

  if (!avoid_ros_namespace_conventions) {
    topic_prefix = ros_topic_prefix;
  }

  concat_str = rcutils_format_string(allocator, "%s%s", topic_prefix, topic_name);
  if (!concat_str) {
    RMW_SET_ERROR_MSG("could not allocate memory for topic string");
    success = false;
    goto end;
  }
  *topic_str = DDS::String_dup(concat_str);

end:
  if (concat_str) {
    allocator.deallocate(concat_str, allocator.state);
  }
  return success;
}

bool
_process_service_name(
  const char * service_name,
  bool avoid_ros_namespace_conventions,
  char ** request_topic_str,
  char ** response_topic_str)
{
  bool success = true;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  const char * requester_prefix = "";
  const char * response_prefix = "";
  char * request_concat_str = nullptr;
  char * response_concat_str = nullptr;

  if (!avoid_ros_namespace_conventions) {
    requester_prefix = ros_service_requester_prefix;
    response_prefix = ros_service_response_prefix;
  }

  // concat the ros_service_*_prefix and Request/Reply suffixes with the service_name
  request_concat_str = rcutils_format_string(
    allocator,
    "%s%s%s", requester_prefix, service_name, "Request");
  if (!request_concat_str) {
    RMW_SET_ERROR_MSG("could not allocate memory for request topic string");
    success = false;
    goto end;
  }
  response_concat_str = rcutils_format_string(
    allocator,
    "%s%s%s", response_prefix, service_name, "Reply");
  if (!response_concat_str) {
    RMW_SET_ERROR_MSG("could not allocate memory for response topic string");
    success = false;
    goto end;
  }
  *request_topic_str = DDS::String_dup(request_concat_str);
  *response_topic_str = DDS::String_dup(response_concat_str);

end:
  if (request_concat_str) {
    allocator.deallocate(request_concat_str, allocator.state);
  }
  if (response_concat_str) {
    allocator.deallocate(response_concat_str, allocator.state);
  }
  return success;
}
