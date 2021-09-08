// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_CONNEXT_SHARED_CPP__GUID_HELPER_HPP_
#define RMW_CONNEXT_SHARED_CPP__GUID_HELPER_HPP_

#include <cstring>
#include <iostream>
#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "ndds/ndds_namespace_cpp.h"


inline bool operator==(const DDS_GUID_t & lhs, const DDS_GUID_t & rhs)
{
  ///  http://community.rti.com/rti-doc/500/ndds.5.0.0/doc/html/api_cpp/group__DDSGUIDSupportModule.html#
  return DDS_BOOLEAN_TRUE == DDS_GUID_equals(&lhs, &rhs);
}

inline bool operator!=(const DDS_GUID_t & lhs, const DDS_GUID_t & rhs)
{
  return !operator==(lhs, rhs);
}

inline bool operator<(const DDS_GUID_t & lhs, const DDS_GUID_t & rhs)
{
  ///  http://community.rti.com/rti-doc/500/ndds.5.0.0/doc/html/api_cpp/group__DDSGUIDSupportModule.html#
  return DDS_GUID_compare(&lhs, &rhs) < 0;
}

inline bool operator>(const DDS_GUID_t & lhs, const DDS_GUID_t & rhs) {return operator<(rhs, lhs);}
inline bool operator<=(const DDS_GUID_t & lhs, const DDS_GUID_t & rhs)
{
  return !operator>(lhs, rhs);
}
inline bool operator>=(const DDS_GUID_t & lhs, const DDS_GUID_t & rhs)
{
  return !operator<(lhs, rhs);
}


inline std::ostream & operator<<(std::ostream & output, const DDS_GUID_t & guiP)
{
  output << std::hex;
  for (uint8_t i = 0; i < 11; ++i) {
    output << static_cast<int>(guiP.value[i]) << ".";
  }
  output << static_cast<int>(guiP.value[11]);
  return output << std::dec;
}


inline void DDS_InstanceHandle_to_GUID(DDS_GUID_t * guid, DDS_InstanceHandle_t instanceHandle)
{
  memcpy(guid->value, reinterpret_cast<DDS_Octet const *>(&instanceHandle), 16);
}

//  Taken from http://community.rti.com/comment/689#comment-689
inline void DDS_BuiltinTopicKey_to_GUID(DDS_GUID_t * guid, DDS_BuiltinTopicKey_t builtinTopicKey)
{
#if BIG_ENDIAN
  memcpy(guid->value, reinterpret_cast<DDS_Octet const *>(&builtinTopicKey), 16);
#else
  /* little endian */
  DDS_Octet const * keyBuffer = reinterpret_cast<DDS_Octet *>(&builtinTopicKey);
  for (uint8_t i = 0; i < 4; ++i) {
    DDS_Octet * guidElement = &(guid->value[i * 4]);
    DDS_Octet const * keyBufferElement = keyBuffer + (i * 4);
    guidElement[0] = keyBufferElement[3];
    guidElement[1] = keyBufferElement[2];
    guidElement[2] = keyBufferElement[1];
    guidElement[3] = keyBufferElement[0];
  }
#endif
}

#endif  // RMW_CONNEXT_SHARED_CPP__GUID_HELPER_HPP_
