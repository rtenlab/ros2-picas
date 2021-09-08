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

#include <rosidl_typesupport_connext_c/wstring_conversion.hpp>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wpedantic"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#include "ndds/ndds_c.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_generator_c/u16string_functions.h"

namespace rosidl_typesupport_connext_c
{

DDS_Wchar * create_wstring_from_u16string(const rosidl_generator_c__U16String & u16str)
{
  DDS_Wchar * wstr = DDS_Wstring_alloc(static_cast<DDS_Long>(u16str.size));
  if (NULL == wstr) {
    return wstr;
  }
  for (size_t i = 0; i < u16str.size; ++i) {
    wstr[i] = static_cast<DDS_Wchar>(u16str.data[i]);
  }
  wstr[u16str.size] = static_cast<DDS_Wchar>(u'\0');
  return wstr;
}

bool wstring_to_u16string(const DDS_Wchar * wstr, rosidl_generator_c__U16String & u16str)
{
  size_t size = static_cast<size_t>(DDS_Wstring_length(wstr));
  bool succeeded = rosidl_generator_c__U16String__resize(&u16str, size);
  if (!succeeded) {
    return false;
  }
  for (size_t i = 0; i < size; ++i) {
    u16str.data[i] = static_cast<char16_t>(wstr[i]);
  }
  return true;
}

}  // namespace rosidl_typesupport_connext_c
