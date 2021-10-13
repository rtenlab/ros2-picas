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

#ifndef ROSIDL_TYPESUPPORT_CONNEXT_CPP__MESSAGE_TYPE_SUPPORT_H_
#define ROSIDL_TYPESUPPORT_CONNEXT_CPP__MESSAGE_TYPE_SUPPORT_H_

#include "rcutils/types/uint8_array.h"

#include "rosidl_runtime_c/message_type_support_struct.h"

// forward declare DDS_TypeCode
struct DDS_TypeCode;

typedef struct message_type_support_callbacks_t
{
  const char * message_namespace;
  const char * message_name;
  // Function to register type with given dds_participant
  DDS_TypeCode * (*get_type_code)(void);
  bool (* convert_ros_to_dds)(
    const void * untyped_ros_message,
    void * untyped_data_message);
  bool (* convert_dds_to_ros)(
    const void * untyped_data_message,
    void * untyped_ros_message);
  // Function to serialize a ROS message to a CDR stream
  bool (* to_cdr_stream)(
    const void * untyped_ros_message,
    rcutils_uint8_array_t * cdr_stream);
  // Function to deserialize a CDR message to a ROS message
  bool (* to_message)(
    const rcutils_uint8_array_t * cdr_stream,
    void * untyped_ros_message);
} message_type_support_callbacks_t;

#endif  // ROSIDL_TYPESUPPORT_CONNEXT_CPP__MESSAGE_TYPE_SUPPORT_H_
