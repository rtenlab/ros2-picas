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

#include <rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp>

namespace rosidl_typesupport_fastrtps_cpp
{

void u16string_to_wstring(const std::u16string & u16str, std::wstring & wstr)
{
  wstr.resize(u16str.size());
  for (size_t i = 0; i < u16str.size(); ++i) {
    wstr[i] = static_cast<wchar_t>(u16str[i]);
  }
}

bool wstring_to_u16string(const std::wstring & wstr, std::u16string & u16str)
{
  try {
    u16str.resize(wstr.size());
  } catch (...) {
    return false;
  }
  for (size_t i = 0; i < wstr.size(); ++i) {
    u16str[i] = static_cast<char16_t>(wstr[i]);
  }
  return true;
}

}  // namespace rosidl_typesupport_fastrtps_cpp
