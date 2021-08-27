// Copyright 2014-2016 Open Source Robotics Foundation, Inc.
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

#ifndef PUBLISH_TAKE_HPP_
#define PUBLISH_TAKE_HPP_

#ifndef _WIN32
# pragma GCC diagnostic push
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#include <ndds/ndds_cpp.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

bool using_introspection_c_typesupport(const char * typesupport_identifier);

bool using_introspection_cpp_typesupport(const char * typesupport_identifier);

bool _publish(DDS_DynamicData * dynamic_data, const void * ros_message,
  const void * untyped_members, const char * typesupport);

bool _take(DDS_DynamicData * dynamic_data, void * ros_message,
  const void * untyped_members, const char * typesupport);

#endif  // PUBLISH_TAKE_HPP_
