// Copyright 2014-2018 Open Source Robotics Foundation, Inc.
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

#include <string>

#include <CdrTypeSupport.h>

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rosidl_typesupport_opensplice_c/identifier.h"
#include "rosidl_typesupport_opensplice_cpp/identifier.hpp"
#include "rosidl_typesupport_opensplice_cpp/message_type_support.h"

#include "typesupport_macros.hpp"

RMW_LOCAL
const rosidl_message_type_support_t *
get_typesupport(
  const rosidl_message_type_support_t * type_support)
{
  RMW_OPENSPLICE_EXTRACT_MESSAGE_TYPESUPPORT(type_support, ts);

  return ts;
}

extern "C"
{
rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  rmw_ret_t ret = RMW_RET_OK;

  if (!ros_message) {
    RMW_SET_ERROR_MSG("ros_message handle is null");
    return RMW_RET_ERROR;
  }

  auto ts = get_typesupport(type_support);
  if (!ts) {
    return RMW_RET_ERROR;
  }

  if (!serialized_message) {
    RMW_SET_ERROR_MSG("serialized_message handle is null");
    return RMW_RET_ERROR;
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);

  const char * error_string = callbacks->serialize(ros_message, serialized_message);
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to serialize message:") + error_string).c_str());
    return RMW_RET_ERROR;
  }

  return ret;
}

rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  if (!serialized_message) {
    RMW_SET_ERROR_MSG("serialized_message handle is null");
    return RMW_RET_ERROR;
  }

  if (!ros_message) {
    RMW_SET_ERROR_MSG("ros_message handle is null");
    return RMW_RET_ERROR;
  }

  auto ts = get_typesupport(type_support);
  if (!ts) {
    return RMW_RET_ERROR;
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);

  const char * error_string = callbacks->deserialize(
    serialized_message->buffer, (unsigned int)serialized_message->buffer_length, ros_message);
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to deserialize message:") + error_string).c_str());
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * /*type_support*/,
  const rosidl_message_bounds_t * /*message_bounds*/,
  size_t * /*size*/)
{
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_ERROR;
}
}  // extern "C"
