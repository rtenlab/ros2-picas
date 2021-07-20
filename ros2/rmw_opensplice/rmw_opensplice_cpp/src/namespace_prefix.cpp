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

#include <string>

#include "rosidl_typesupport_opensplice_cpp/misc.hpp"

#include "namespace_prefix.hpp"

using rosidl_typesupport_opensplice_cpp::get_ros_prefixes;
using rosidl_typesupport_opensplice_cpp::get_ros_service_request_prefix;
using rosidl_typesupport_opensplice_cpp::get_ros_service_response_prefix;

std::string
_get_ros_service_request_prefix()
{
  return get_ros_service_request_prefix();
}

std::string
_get_ros_service_response_prefix()
{
  return get_ros_service_response_prefix();
}

/// Return the ROS specific prefix if it exists, otherwise "".
std::string
_get_ros_prefix_if_exists(const std::string & topic_name)
{
  for (auto prefix : get_ros_prefixes()) {
    if (topic_name.rfind(std::string(prefix) + "/", 0) == 0) {
      return prefix;
    }
  }
  return "";
}
