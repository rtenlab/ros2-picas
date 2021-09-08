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

#ifndef ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__MESSAGE_TYPE_SUPPORT_H_
#define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__MESSAGE_TYPE_SUPPORT_H_

#include <stdbool.h>
#include <ccpp.h>

#include "rosidl_generator_c/message_type_support_struct.h"

typedef struct message_type_support_callbacks_t
{
  const char * message_namespace;
  const char * message_name;
  // Function to register type with given dds_participant.
  // Returns NULL if the registration was successful, otherwise an error string.
  const char *
  (*register_type)(void * untyped_participant, const char * type_name);
  // Function to publish a ROS message with a given DDS data_writer
  // Returns NULL if the publish was successful, otherwise an error string.
  const char *
  (*publish)(void * dds_data_writer, const void * ros_message);
  // Function to take a ROS message from a dds data reader
  // Returns NULL if the take was successful, otherwise an error string.
  // If no data is available to be taken, NULL is returned but taken will be set to false.
  const char *
  (*take)(
    void * dds_data_reader,
    bool ignore_local_publications,
    void * ros_message,
    bool * taken,
    void * sending_publication_handle);
  const char *
  (*serialize)(
    const void * ros_message,
    void * untyped_serialized_message);
  const char *
  (*deserialize)(
    const uint8_t * buffer,
    unsigned length,
    void * ros_message);
  // Functions for converting between DDS and ROS messages of this type.
  // Returns NULL if successful, otherwise an error string.
  const char *
  (*convert_ros_to_dds)(const void * ros_message, void * dds_message);
  const char *
  (*convert_dds_to_ros)(const void * dds_message, void * ros_message);
} message_type_support_callbacks_t;

#endif  // ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__MESSAGE_TYPE_SUPPORT_H_
