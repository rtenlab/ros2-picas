// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_TYPESUPPORT_CONNEXT_CPP__WSTRING_CONVERSION_HPP_
#define ROSIDL_TYPESUPPORT_CONNEXT_CPP__WSTRING_CONVERSION_HPP_

#include <string>

#include "ndds/ndds_version.h"

#include "rosidl_typesupport_connext_cpp/visibility_control.h"

// forward declare DDS_Wchar
#if RTI_DDS_VERSION_MAJOR < 6
typedef unsigned int DDS_Wchar;
#else
typedef unsigned short DDS_Wchar;  // NOLINT
#endif

namespace rosidl_typesupport_connext_cpp
{

ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC
DDS_Wchar * create_wstring_from_u16string(
  const std::u16string & u16str);

ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC
bool wstring_to_u16string(
  const DDS_Wchar * wstr, std::u16string & u16str);

}  // namespace rosidl_typesupport_connext_cpp

#endif  // ROSIDL_TYPESUPPORT_CONNEXT_CPP__WSTRING_CONVERSION_HPP_
