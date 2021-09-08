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

#include <string>
#include <vector>
#include "rosidl_typesupport_opensplice_cpp/misc.hpp"

namespace rosidl_typesupport_opensplice_cpp
{
static const char * const ros_topic_prefix = "rt";
static const char * const ros_service_request_prefix = "rq";
static const char * const ros_service_response_prefix = "rr";

const std::vector<std::string> &
get_ros_prefixes()
{
  static const std::vector<std::string> ros_prefixes =
  {ros_topic_prefix, ros_service_request_prefix, ros_service_response_prefix};

  return ros_prefixes;
}

std::string
get_ros_topic_prefix()
{
  return ros_topic_prefix;
}

std::string
get_ros_service_request_prefix()
{
  return ros_service_request_prefix;
}

std::string
get_ros_service_response_prefix()
{
  return ros_service_response_prefix;
}

bool
process_topic_name(
  const char * topic_name,
  bool avoid_ros_namespace_conventions,
  std::string & topic_str)
{
  const std::string topic_name_ = topic_name;
  topic_str.clear();
  if (!avoid_ros_namespace_conventions) {
    topic_str = std::string(ros_topic_prefix) + topic_name_;
  } else {
    topic_str = topic_name_;
  }
  return true;
}

bool
process_service_name(
  const char * service_name,
  bool avoid_ros_namespace_conventions,
  std::string & service_str,
  std::string & request_str,
  std::string & response_str)
{
  const std::string service_name_ = service_name;
  request_str.clear();
  response_str.clear();
  service_str.clear();
  if (!avoid_ros_namespace_conventions) {
    request_str = std::string(ros_service_request_prefix) + service_name_ + "Request";
    response_str = std::string(ros_service_response_prefix) + service_name_ + "Reply";
  } else {
    request_str = service_name_ + "Request";
    response_str = service_name_ + "Reply";
  }
  service_str = service_name_;
  return true;
}

}  // namespace rosidl_typesupport_opensplice_cpp
