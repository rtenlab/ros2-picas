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

#include "./publish_take.hpp"
#include "./templates.hpp"

bool using_introspection_c_typesupport(const char * typesupport_identifier)
{
  return typesupport_identifier == rosidl_typesupport_introspection_c__identifier;
}

bool using_introspection_cpp_typesupport(const char * typesupport_identifier)
{
  return typesupport_identifier ==
         rosidl_typesupport_introspection_cpp::typesupport_identifier;
}

bool _publish(DDS_DynamicData * dynamic_data, const void * ros_message,
  const void * untyped_members, const char * typesupport)
{
  if (using_introspection_c_typesupport(typesupport)) {
    return publish<rosidl_typesupport_introspection_c__MessageMembers>(
      dynamic_data, ros_message, untyped_members);
  } else if (using_introspection_cpp_typesupport(typesupport)) {
    return publish<rosidl_typesupport_introspection_cpp::MessageMembers>(
      dynamic_data, ros_message, untyped_members);
  }

  RMW_SET_ERROR_MSG("Unknown typesupport identifier")
  return false;
}

bool _take(DDS_DynamicData * dynamic_data, void * ros_message,
  const void * untyped_members, const char * typesupport)
{
  if (using_introspection_c_typesupport(typesupport)) {
    return take<rosidl_typesupport_introspection_c__MessageMembers>(
      dynamic_data, ros_message, untyped_members);
  } else if (using_introspection_cpp_typesupport(typesupport)) {
    return take<rosidl_typesupport_introspection_cpp::MessageMembers>(
      dynamic_data, ros_message, untyped_members);
  }
  RMW_SET_ERROR_MSG("Unknown typesupport identifier")
  return false;
}
