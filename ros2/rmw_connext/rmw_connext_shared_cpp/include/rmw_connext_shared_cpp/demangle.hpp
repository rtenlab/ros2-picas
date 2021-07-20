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

#ifndef RMW_CONNEXT_SHARED_CPP__DEMANGLE_HPP_
#define RMW_CONNEXT_SHARED_CPP__DEMANGLE_HPP_

#include <string>

std::string
_demangle_if_ros_topic(const std::string & topic_name);

std::string
_demangle_if_ros_type(const std::string & dds_type_string);

std::string
_demangle_service_from_topic(const std::string & topic_name);

std::string
_demangle_service_type_only(const std::string & dds_type_name);

#endif  // RMW_CONNEXT_SHARED_CPP__DEMANGLE_HPP_
