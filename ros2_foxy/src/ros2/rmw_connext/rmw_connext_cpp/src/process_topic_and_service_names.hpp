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

#ifndef PROCESS_TOPIC_AND_SERVICE_NAMES_HPP_
#define PROCESS_TOPIC_AND_SERVICE_NAMES_HPP_

bool
_process_topic_name(
  const char * topic_name,
  bool avoid_ros_namespace_conventions,
  char ** topic_str);

bool
_process_service_name(
  const char * service_name,
  bool avoid_ros_namespace_conventions,
  char ** request_topic_str,
  char ** response_topic_str);

#endif  // PROCESS_TOPIC_AND_SERVICE_NAMES_HPP_
