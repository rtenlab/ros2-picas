// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef TYPESUPPORT_MACROS_HPP_
#define TYPESUPPORT_MACROS_HPP_

#define RMW_OPENSPLICE_EXTRACT_MESSAGE_TYPESUPPORT(TYPE_SUPPORTS, TYPE_SUPPORT) \
  if (!TYPE_SUPPORTS) { \
    RMW_SET_ERROR_MSG("type supports handle is null"); \
    return nullptr; \
  } \
  const rosidl_message_type_support_t * TYPE_SUPPORT = \
    get_message_typesupport_handle( \
    TYPE_SUPPORTS, rosidl_typesupport_opensplice_c__identifier); \
  if (!TYPE_SUPPORT) { \
    TYPE_SUPPORT = get_message_typesupport_handle( \
      TYPE_SUPPORTS, rosidl_typesupport_opensplice_cpp::typesupport_identifier); \
    if (!TYPE_SUPPORT) { \
      char __msg[1024]; \
      snprintf( \
        __msg, 1024, \
        "type support handle implementation '%s' (%p) does not match valid type supports " \
        "('%s' (%p), '%s' (%p))", \
        TYPE_SUPPORTS->typesupport_identifier, \
        static_cast<const void *>(TYPE_SUPPORTS->typesupport_identifier), \
        rosidl_typesupport_opensplice_cpp::typesupport_identifier, \
        static_cast<const void *>(rosidl_typesupport_opensplice_cpp::typesupport_identifier), \
        rosidl_typesupport_opensplice_c__identifier, \
        static_cast<const void *>(rosidl_typesupport_opensplice_c__identifier)); \
      RMW_SET_ERROR_MSG(__msg); \
      return nullptr; \
    } \
  }

#define RMW_OPENSPLICE_EXTRACT_SERVICE_TYPESUPPORT(TYPE_SUPPORTS, TYPE_SUPPORT) \
  if (!TYPE_SUPPORTS) { \
    RMW_SET_ERROR_MSG("type supports handle is null"); \
    return nullptr; \
  } \
  const rosidl_service_type_support_t * TYPE_SUPPORT = \
    get_service_typesupport_handle( \
    TYPE_SUPPORTS, rosidl_typesupport_opensplice_c__identifier); \
  if (!TYPE_SUPPORT) { \
    TYPE_SUPPORT = get_service_typesupport_handle( \
      TYPE_SUPPORTS, rosidl_typesupport_opensplice_cpp::typesupport_identifier); \
    if (!TYPE_SUPPORT) { \
      char __msg[1024]; \
      snprintf( \
        __msg, 1024, \
        "type support handle implementation '%s' (%p) does not match valid type supports " \
        "('%s' (%p), '%s' (%p))", \
        TYPE_SUPPORTS->typesupport_identifier, \
        static_cast<const void *>(TYPE_SUPPORTS->typesupport_identifier), \
        rosidl_typesupport_opensplice_cpp::typesupport_identifier, \
        static_cast<const void *>(rosidl_typesupport_opensplice_cpp::typesupport_identifier), \
        rosidl_typesupport_opensplice_c__identifier, \
        static_cast<const void *>(rosidl_typesupport_opensplice_c__identifier)); \
      RMW_SET_ERROR_MSG(__msg); \
      return nullptr; \
    } \
  }

#endif  // TYPESUPPORT_MACROS_HPP_
