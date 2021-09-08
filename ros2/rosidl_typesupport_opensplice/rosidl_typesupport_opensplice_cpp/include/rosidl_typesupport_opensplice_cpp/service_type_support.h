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

#ifndef ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__SERVICE_TYPE_SUPPORT_H_
#define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__SERVICE_TYPE_SUPPORT_H_

#include <stddef.h>
#include <stdint.h>

#include "rmw/rmw.h"

#include "rosidl_generator_c/service_type_support_struct.h"

typedef struct service_type_support_callbacks_t
{
  const char * service_namespace;
  const char * service_name;
  // Function to create a requester
  // Returns NULL if the requester was successfully created, otherwise an error string.
  // Passing NULL to the allocator will result in malloc being used.
  const char * (*create_requester)(
    void * participant, const char * service_name, void ** requester, void ** reader,
    const void * datareader_qos, const void * datawriter_qos, bool avoid_ros_namespace_conventions,
    void * (*allocator)(size_t));
  // De-allocate a requester
  // The deallocator should match the allocator passed to create_requester.
  // Returns NULL if the requester was successfully destroyed, otherwise an error string.
  // Passing NULL for the deallocator will result in free being used.
  const char * (*destroy_requester)(void * untyped_requester, void (* deallocator)(void *));
  // Function to create a responder
  // Returns NULL if the responder was successfully created, otherwise an error string.
  // Passing NULL to the allocator will result in malloc being used.
  const char * (*create_responder)(
    void * participant, const char * service_name, void ** responder, void ** reader,
    const void * datareader_qos, const void * datawriter_qos, bool avoid_ros_namespace_conventions,
    void * (*allocator)(size_t));
  // De-allocate a responder
  // The deallocator should match the allocator passed to create_respnder.
  // Returns NULL if the responder was successfully destroyed, otherwise an error string.
  // Passing NULL for the deallocator will result in free being used.
  const char * (*destroy_responder)(void * untyped_requester, void (* deallocator)(void *));
  // Function to send ROS requests
  // Returns NULL if the request was successfully sent, otherwise an error string.
  const char * (*send_request)(
    void * requester, const void * ros_request, int64_t * sequence_number);
  // Function to read a ROS request from the wire
  // Returns NULL if the request was successfully taken, otherwise an error string.
  // If no data is available to be taken, NULL is returned but taken will be set to false.
  const char * (*take_request)(
    void * responder, rmw_request_id_t * request_header, void * ros_request, bool * taken);
  // Function to send ROS responses
  // Returns NULL if the response was successfully sent, otherwise an error string.
  const char * (*send_response)(
    void * responder, const rmw_request_id_t * request_header, const void * ros_response);
  // Function to read a ROS response from the wire
  // Returns NULL if the response was successfully taken, otherwise an error string.
  // If no data is available to be taken, NULL is returned but taken will be set to false.
  const char * (*take_response)(
    void * requester, rmw_request_id_t * request_header, void * ros_response, bool * taken);
  // Function to check if a service server is available for a given client
  // Returns NULL if the check was successfully, otherwise an error string.
  // If no server is available, NULL is returned but is_available will be set to false.
  const char * (*server_is_available)(
    void * requester, const rmw_node_t * node, bool * is_available);
} service_type_support_callbacks_t;

#endif  // ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__SERVICE_TYPE_SUPPORT_H_
