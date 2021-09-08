// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__MISC_HPP_
#define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__MISC_HPP_

#include <rosidl_typesupport_opensplice_cpp/visibility_control.h>
#include <string>
#include <vector>

namespace rosidl_typesupport_opensplice_cpp
{

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC
const std::vector<std::string> & get_ros_prefixes();

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC
std::string get_ros_topic_prefix();

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC
std::string get_ros_service_request_prefix();

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC
std::string get_ros_service_response_prefix();

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC
bool process_topic_name(
  const char * topic_name,
  bool avoid_ros_namespace_conventions,
  std::string & topic_str);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC
bool process_service_name(
  const char * service_name,
  bool avoid_ros_namespace_conventions,
  std::string & service_str,
  std::string & request_str,
  std::string & response_str);

}  // namespace rosidl_typesupport_opensplice_cpp

#endif  // ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__MISC_HPP_
