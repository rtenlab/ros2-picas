// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef TEMPLATES_HPP_
#define TEMPLATES_HPP_

#include <limits>
#include <string>
#include <vector>

#include "rosidl_generator_c/primitives_sequence_functions.h"
#include "rosidl_generator_c/string.h"
#include "rosidl_generator_c/string_functions.h"

#include "rmw/allocators.h"
#include "rmw/impl/cpp/macros.hpp"

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_c/visibility_control.h"

#include "./macros.hpp"

/********** Utility structs **********/

// This struct is used to associate MessageMember the introspection packages
// with the corresponding MessageMembers struct
template<typename T>
struct GenericMembersT;

template<>
struct GenericMembersT<rosidl_typesupport_introspection_cpp::MessageMember>
{
  using type = rosidl_typesupport_introspection_cpp::MessageMembers;
};

template<>
struct GenericMembersT<rosidl_typesupport_introspection_c__MessageMember>
{
  using type = rosidl_typesupport_introspection_c__MessageMembers;
};

// This struct is used to associate a C array with a struct that we can metaprogram with
template<uint8_t Id>
struct GenericCSequence;

template<uint8_t Id>
struct IdTypeMap;

// multiple definitions of ambiguous primitive types
SPECIALIZE_GENERIC_C_SEQUENCE(STRING, String, rosidl_generator_c__String)
SPECIALIZE_GENERIC_C_SEQUENCE(BOOL, bool, bool)
SPECIALIZE_GENERIC_C_SEQUENCE(BYTE, byte, uint8_t)
SPECIALIZE_GENERIC_C_SEQUENCE(CHAR, char, signed char)
SPECIALIZE_GENERIC_C_SEQUENCE(FLOAT32, float32, float)
SPECIALIZE_GENERIC_C_SEQUENCE(FLOAT64, float64, double)
SPECIALIZE_GENERIC_C_SEQUENCE(INT8, int8, int8_t)
SPECIALIZE_GENERIC_C_SEQUENCE(UINT8, uint8, uint8_t)
SPECIALIZE_GENERIC_C_SEQUENCE(INT16, int16, int16_t)
SPECIALIZE_GENERIC_C_SEQUENCE(UINT16, uint16, uint16_t)
SPECIALIZE_GENERIC_C_SEQUENCE(INT32, int32, int32_t)
SPECIALIZE_GENERIC_C_SEQUENCE(UINT32, uint32, uint32_t)
SPECIALIZE_GENERIC_C_SEQUENCE(INT64, int64, int64_t)
SPECIALIZE_GENERIC_C_SEQUENCE(UINT64, uint64, uint64_t)

/********** end utility structs **********/

/********** Map dynamic data function names to types **********/
template<typename T, typename DDSType>
DDS_ReturnCode_t set_dynamic_data(DDS_DynamicData * dynamic_data, size_t index,
  const DDSType value);

template<typename T, typename DDSType>
DDS_ReturnCode_t set_dynamic_data_array(
  DDS_DynamicData * dynamic_data, size_t index, size_t array_size, const DDSType * values);

template<typename T, typename DDSType>
DDS_ReturnCode_t get_dynamic_data(DDS_DynamicData * dynamic_data, DDSType & value, size_t index);

template<typename T, typename DDSType>
DDS_ReturnCode_t get_dynamic_data_array(
  DDS_DynamicData * dynamic_data, DDSType * & values, size_t array_size, size_t index);

DEFINE_DYNAMIC_DATA_METHODS(bool, DDS_Boolean, boolean)
DEFINE_DYNAMIC_DATA_METHODS(signed char, char, char)
DEFINE_DYNAMIC_DATA_METHODS(float, float, float)
DEFINE_DYNAMIC_DATA_METHODS(double, double, double)
DEFINE_DYNAMIC_DATA_METHODS(int8_t, DDS_Octet, octet)
DEFINE_DYNAMIC_DATA_METHODS(uint8_t, uint8_t, octet)
DEFINE_DYNAMIC_DATA_METHODS(int16_t, int16_t, short)  // NOLINT
DEFINE_DYNAMIC_DATA_METHODS(uint16_t, uint16_t, ushort)
DEFINE_DYNAMIC_DATA_METHODS(int32_t, int32_t, long)  // NOLINT
DEFINE_DYNAMIC_DATA_METHODS(uint32_t, uint32_t, ulong)
DEFINE_DYNAMIC_DATA_METHODS(int64_t, DDS_LongLong, longlong)
DEFINE_DYNAMIC_DATA_METHODS(uint64_t, DDS_UnsignedLongLong, ulonglong)

template<>
DDS_ReturnCode_t set_dynamic_data<char *, char *>(
  DDS_DynamicData * dynamic_data, size_t index, char * const value)
{
  auto member_id = static_cast<DDS_DynamicDataMemberId>(index);
  return dynamic_data->set_string(NULL, member_id, value);
}

template<>
DDS_ReturnCode_t set_dynamic_data<char *, const char *>(
  DDS_DynamicData * dynamic_data, size_t index, const char * value)
{
  auto member_id = static_cast<DDS_DynamicDataMemberId>(index);
  return dynamic_data->set_string(NULL, member_id, value);
}

DDS_ReturnCode_t get_dynamic_data_string(
  DDS_DynamicData * dynamic_data, char * & values, DDS_UnsignedLong * array_size, size_t index)
{
  auto member_id = static_cast<DDS_DynamicDataMemberId>(index);
  return dynamic_data->get_string(values, array_size, NULL, member_id);
}

/********** end dynamic data functions **********/

/********** set_*values **********/

template<typename TrueType, typename T, typename MessageMemberT>
bool set_primitive_value(
  const void * ros_message,
  const MessageMemberT * member,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  const T * value =
    reinterpret_cast<const T *>(static_cast<const char *>(ros_message) + member->offset_);
  DDS_ReturnCode_t status = set_dynamic_data<TrueType>(
    dynamic_data,
    i + 1,
    *value);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to set primitive value");
    return false;
  }
  return true;
}

template<uint8_t Id, typename T, typename MessageMemberT>
size_t set_array_size_and_values(
  const MessageMemberT * member,
  const void * ros_message,
  T * & ros_values);

template<uint8_t Id, typename T = typename IdTypeMap<Id>::type,
typename std::enable_if<!std::is_same<T, bool>::value>::type * = nullptr>
size_t set_array_size_and_values(
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  const void * ros_message,
  T * & ros_values)
{
  if (member->array_size_ && !member->is_upper_bound_) {
    ros_values =
      const_cast<T *>(reinterpret_cast<const T *>(static_cast<const char *>(ros_message) +
      member->offset_));
    return member->array_size_;
  }
  const void * untyped_vector = static_cast<const char *>(ros_message) + member->offset_;
  auto output = static_cast<const std::vector<T> *>(untyped_vector);
  ros_values = const_cast<T *>(output->data());
  return output->size();
}

template<>
size_t set_array_size_and_values<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL>(
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  const void * ros_message,
  bool * & ros_values)
{
  if (member->array_size_ && !member->is_upper_bound_) {
    ros_values =
      const_cast<bool *>(reinterpret_cast<const bool *>(static_cast<const char *>(ros_message) +
      member->offset_));
    return member->array_size_;
  }
  fprintf(stderr, "WARNING: specialization of set_array_size_and_values is invalid!\n");
  return 0;
}

template<uint8_t Id, typename T = typename IdTypeMap<Id>::type>
size_t set_array_size_and_values(
  const rosidl_typesupport_introspection_c__MessageMember * member,
  const void * ros_message,
  T * & ros_values)
{
  if (member->array_size_ && !member->is_upper_bound_) {
    ros_values =
      const_cast<T *>(reinterpret_cast<const T *>(static_cast<const char *>(ros_message) +
      member->offset_));
    return member->array_size_;
  }
  const void * untyped_array = static_cast<const char *>(ros_message) + member->offset_;
  auto output = static_cast<const typename GenericCSequence<Id>::type *>(untyped_array);
  ros_values = output->data;
  return output->size;
}

template<uint8_t Id, typename MessageMemberT, typename T = typename IdTypeMap<Id>::type>
bool set_value(
  const MessageMemberT * member,
  const void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  T * ros_values = nullptr;
  if (member->is_array_) {
    size_t array_size = set_array_size_and_values<Id>(member, ros_message, ros_values);
    DDS_ReturnCode_t status = set_dynamic_data_array<T>(
      dynamic_data,
      i + 1,
      static_cast<DDS_UnsignedLong>(array_size),
      ros_values);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to set array value");
      return false;
    }
    return true;
  }
  return set_primitive_value<T, T>(ros_message, member, dynamic_data, i);
}

// TODO(jacquelinekay): Consider that set_value is a generalization of
// set_value_with_different_types and the two functions could be phrased more elegantly as such
template<uint8_t Id, typename DDSType, typename MessageMemberT,
typename T = typename IdTypeMap<Id>::type>
bool set_value_with_different_types(
  const MessageMemberT * member,
  const void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  T * ros_values = nullptr;
  if (member->is_array_) {
    size_t array_size = set_array_size_and_values<Id>(member, ros_message, ros_values);
    DDSType * values = nullptr;
    if (array_size > 0) {
      if (!ros_values) {
        RMW_SET_ERROR_MSG("failed to cast ros_values from message array");
        return false;
      }
      values = static_cast<DDSType *>(rmw_allocate(sizeof(DDSType) * array_size));
      if (!values) {
        RMW_SET_ERROR_MSG("failed to allocate memory");
        return false;
      }
      for (size_t j = 0; j < array_size; ++j) {
        values[j] = ros_values[j];
      }
    }
    DDS_ReturnCode_t status = set_dynamic_data_array<T>(
      dynamic_data,
      i + 1,
      static_cast<DDS_UnsignedLong>(array_size),
      values);
    rmw_free(values);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to set array value");
      return false;
    }
  } else {
    set_primitive_value<T, DDSType>(ros_message, member, dynamic_data, i);
  }
  return true;
}

template<>
bool
set_value_with_different_types<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL, DDS_Boolean>(
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  const void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  const uint8_t Id = rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL;
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  if (member->is_array_) {
    DDS_Boolean * values = nullptr;
    size_t array_size;
    if (member->array_size_ && !member->is_upper_bound_) {
      bool * ros_values = nullptr;
      array_size = set_array_size_and_values<Id>(member, ros_message, ros_values);
      if (array_size > 0) {
        if (!ros_values) {
          RMW_SET_ERROR_MSG("failed to cast ros_values from message array");
          return false;
        }
        values = static_cast<DDS_Boolean *>(rmw_allocate(sizeof(DDS_Boolean) * array_size));
        if (!values) {
          RMW_SET_ERROR_MSG("failed to allocate memory");
          return false;
        }
        for (size_t j = 0; j < array_size; ++j) {
          values[j] = ros_values[j];
        }
      }
    } else {
      const void * untyped_vector = static_cast<const char *>(ros_message) + member->offset_;
      auto output = static_cast<const std::vector<bool> *>(untyped_vector);
      array_size = output->size();
      values = static_cast<DDS_Boolean *>(rmw_allocate(sizeof(DDS_Boolean) * array_size));
      for (size_t j = 0; j < output->size(); ++j) {
        values[j] = (*output)[j];
      }
    }
    if (array_size > 0) {
      DDS_ReturnCode_t status = set_dynamic_data_array<bool>(
        dynamic_data,
        i + 1,
        static_cast<DDS_UnsignedLong>(array_size),
        values);
      rmw_free(values);
      if (status != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to set array value");
        return false;
      }
    }
  } else {
    set_primitive_value<bool, DDS_Boolean>(ros_message, member, dynamic_data, i);
  }
  return true;
}


template<>
bool set_value<rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING>(
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  const void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  std::string * ros_values = nullptr;
  if (member->is_array_) {
    DDS_DynamicData dynamic_data_member(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
      dynamic_data_member,
      NULL,
      static_cast<DDS_DynamicDataMemberId>(i + 1));
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to bind complex member");
      return false;
    }
    size_t array_size =
      set_array_size_and_values<rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING>(
      member, ros_message, ros_values);
    if (!ros_values && array_size > 0) {
      RMW_SET_ERROR_MSG("failed to cast ros_values from message array");
      return false;
    }
    if (array_size > (std::numeric_limits<DDS_DynamicDataMemberId>::max)()) {
      RMW_SET_ERROR_MSG(
        "failed to set string since the requested string length exceeds the DDS type");
      return false;
    }
    for (size_t j = 0; j < array_size; ++j) {
      status = set_dynamic_data<char *>(
        &dynamic_data_member,
        j + 1,
        ros_values[j].c_str());
      if (status != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to set array value");
        return false;
      }
    }
    status = dynamic_data->unbind_complex_member(dynamic_data_member);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to unbind complex member");
      return false;
    }
  } else {
    const std::string * value =
      reinterpret_cast<const std::string *>(static_cast<const char *>(ros_message) +
      member->offset_);
    if (!value) {
      RMW_SET_ERROR_MSG("failed to cast string");
      return false;
    }
    if (!value->c_str()) {
      return false;
    }
    DDS_ReturnCode_t status = set_dynamic_data<char *>(
      dynamic_data,
      i + 1,
      value->c_str());
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to set value");
      return false;
    }
  }
  return true;
}

template<>
bool set_value<rosidl_typesupport_introspection_c__ROS_TYPE_STRING>(
  const rosidl_typesupport_introspection_c__MessageMember * member,
  const void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  const uint8_t Id = rosidl_typesupport_introspection_c__ROS_TYPE_STRING;
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  rosidl_generator_c__String * ros_values = nullptr;
  if (member->is_array_) {
    DDS_DynamicData dynamic_data_member(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
      dynamic_data_member,
      NULL,
      static_cast<DDS_DynamicDataMemberId>(i + 1));
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to bind complex member");
      return false;
    }
    size_t array_size = set_array_size_and_values<Id>(member, ros_message, ros_values);
    if (!ros_values && array_size > 0) {
      RMW_SET_ERROR_MSG("failed to cast ros_values from message array");
      return false;
    }
    if (array_size > (std::numeric_limits<DDS_DynamicDataMemberId>::max)()) {
      RMW_SET_ERROR_MSG(
        "failed to set string since the requested string length exceeds the DDS type");
      return false;
    }
    for (size_t j = 0; j < array_size; ++j) {
      status = set_dynamic_data<char *>(
        &dynamic_data_member,
        static_cast<DDS_DynamicDataMemberId>(j + 1),
        ros_values[j].data);
      if (status != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to set array value");
        return false;
      }
    }
    status = dynamic_data->unbind_complex_member(dynamic_data_member);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to unbind complex member");
      return false;
    }
  } else {
    const rosidl_generator_c__String * value =
      reinterpret_cast<const rosidl_generator_c__String *>(
      static_cast<const char *>(ros_message) + member->offset_);
    if (!value) {
      RMW_SET_ERROR_MSG("failed to cast string");
      return false;
    }
    if (!value->data) {
      RMW_SET_ERROR_MSG("String data was null");
      return false;
    }

    DDS_ReturnCode_t status = set_dynamic_data<char *>(
      dynamic_data,
      static_cast<DDS_DynamicDataMemberId>(i + 1),
      value->data);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to set value");
      return false;
    }
  }
  return true;
}

template<
  typename MembersType,
  typename StringType = typename std::conditional<
    std::is_same<MembersType, rosidl_typesupport_introspection_c__MessageMembers>::value,
    rosidl_generator_c__String, std::string
  >::type>
bool publish(
  DDS_DynamicData * dynamic_data, const void * ros_message,
  const void * untyped_members)
{
  auto members = static_cast<const MembersType *>(untyped_members);
  if (!members) {
    RMW_SET_ERROR_MSG("members handle is null");
    return false;
  }
  USING_INTROSPECTION_TYPEIDS()
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    auto * member = members->members_ + i;
    switch (member->type_id_) {
      case ROS_TYPE_BOOL:
        if (!set_value_with_different_types<ROS_TYPE_BOOL, DDS_Boolean>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_BYTE:
        if (!set_value<ROS_TYPE_BYTE>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_CHAR:
        if (!set_value_with_different_types<ROS_TYPE_CHAR, DDS_Char>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_FLOAT32:
        if (!set_value<ROS_TYPE_FLOAT32>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_FLOAT64:
        if (!set_value<ROS_TYPE_FLOAT64>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT8:
        if (!set_value_with_different_types<ROS_TYPE_INT8, DDS_Octet>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_UINT8:
        if (!set_value<ROS_TYPE_UINT8>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT16:
        if (!set_value<ROS_TYPE_INT16>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_UINT16:
        if (!set_value<ROS_TYPE_UINT16>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT32:
        if (!set_value<ROS_TYPE_INT32>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_UINT32:
        if (!set_value<ROS_TYPE_UINT32>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT64:
        if (!set_value_with_different_types<ROS_TYPE_INT64, DDS_LongLong>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_UINT64:
        if (!set_value_with_different_types<ROS_TYPE_UINT64, DDS_UnsignedLongLong>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_STRING:
        if (!set_value<ROS_TYPE_STRING>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_MESSAGE:
        {
          if (member->is_array_) {
            const void * untyped_member = static_cast<const char *>(ros_message) + member->offset_;
            if (!member->size_function) {
              RMW_SET_ERROR_MSG("size function handle is null");
              return false;
            }
            if (!member->get_const_function) {
              RMW_SET_ERROR_MSG("get const function handle is null");
              return false;
            }

            DDS_DynamicData array_data(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
            DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
              array_data,
              NULL,
              i + 1);
            if (status != DDS_RETCODE_OK) {
              RMW_SET_ERROR_MSG("failed to bind complex member");
              return false;
            }
            size_t array_size = member->size_function(untyped_member);
            for (size_t j = 0; j < array_size; ++j) {
              const void * ros_message;
              {
                const void * sub_ros_message = member->get_const_function(untyped_member, j);
                // offset message pointer since the macro adds the member offset to it
                ros_message = static_cast<const char *>(sub_ros_message) - member->offset_;
              }
              DDS_DynamicData * array_data_ptr = &array_data;
              if (!set_submessage_value(member, ros_message, array_data_ptr, j)) {
                return false;
              }
            }
            status = dynamic_data->unbind_complex_member(array_data);
            if (status != DDS_RETCODE_OK) {
              RMW_SET_ERROR_MSG("failed to unbind complex member");
              return false;
            }
          } else {
            if (!set_submessage_value(member, ros_message, dynamic_data, i)) {
              return false;
            }
          }
        }
        break;
      default:
        RMW_SET_ERROR_MSG(
          (std::string("unknown type id ") + std::to_string(member->type_id_)).c_str());
        return false;
    }
  }
  return true;
}

template<typename MessageMemberT>
bool set_submessage_value(
  const MessageMemberT * member,
  const void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  using MembersT = typename GenericMembersT<MessageMemberT>::type;
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  DDS_DynamicData sub_dynamic_data(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
  DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
    sub_dynamic_data,
    NULL,
    static_cast<DDS_DynamicDataMemberId>(i + 1));
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to bind complex member");
    return false;
  }
  const void * sub_ros_message = static_cast<const char *>(ros_message) + member->offset_;
  if (!member->members_) {
    RMW_SET_ERROR_MSG("members handle is null");
    return false;
  }
  bool published = publish<MembersT>(
    &sub_dynamic_data, sub_ros_message, member->members_->data);
  if (!published) {
    DDS_UnsignedLong count = sub_dynamic_data.get_member_count();
    for (DDS_UnsignedLong k = 0; k < count; ++k) {
      DDS_DynamicDataMemberInfo info;
      status = sub_dynamic_data.get_member_info_by_index(info, k);
      if (status != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to get member info");
        return false;
      }
    }
  }
  status = dynamic_data->unbind_complex_member(sub_dynamic_data);
  if (!published) {
    RMW_SET_ERROR_MSG("failed to publish sub message");
    return false;
  }
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to unbind complex member");
    return false;
  }
  return true;
}

/********** end set_*values functions **********/

/********** get_*values functions **********/

template<typename MessageMemberT>
bool get_array_size(const MessageMemberT * member, size_t & array_size,
  DDS_DynamicData * dynamic_data, size_t i)
{
  if (member->array_size_ && !member->is_upper_bound_) {
    array_size = member->array_size_;
  } else {
    if (!dynamic_data) {
      RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
      return false;
    }
    DDS_DynamicData dynamic_data_member(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
      dynamic_data_member,
      NULL,
      static_cast<DDS_DynamicDataMemberId>(i + 1));
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to bind complex member");
      return false;
    }
    array_size = dynamic_data_member.get_member_count();
    status = dynamic_data->unbind_complex_member(dynamic_data_member);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to unbind complex member");
      return false;
    }
  }
  return true;
}

template<uint8_t Id, typename T, typename MessageMemberT>
bool resize_array_and_get_values(
  T * & ros_values,
  void * ros_message,
  const MessageMemberT * member,
  size_t array_size);

template<uint8_t Id, typename T = typename IdTypeMap<Id>::type,
typename std::enable_if<!std::is_same<T, bool>::value>::type * = nullptr>
bool resize_array_and_get_values(
  T * & ros_values,
  void * ros_message,
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  size_t array_size)
{
  if (member->array_size_ && !member->is_upper_bound_) {
    ros_values = reinterpret_cast<T *>(static_cast<char *>(ros_message) + member->offset_);
  } else {
    void * untyped_vector = static_cast<char *>(ros_message) + member->offset_;
    auto vector = static_cast<std::vector<T> *>(untyped_vector);
    if (!vector) {
      RMW_SET_ERROR_MSG("Failed to cast vector from ROS message");
      return false;
    }
    vector->resize(array_size);
    ros_values = vector->data();
  }
  return true;
}

template<>
bool resize_array_and_get_values<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL>(
  bool * & ros_values,
  void * ros_message,
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  size_t array_size)
{
  if (member->array_size_ && !member->is_upper_bound_) {
    ros_values = reinterpret_cast<bool *>(static_cast<char *>(ros_message) + member->offset_);
    return true;
  } else {
    void * untyped_vector = static_cast<char *>(ros_message) + member->offset_;
    auto vector = static_cast<std::vector<bool> *>(untyped_vector);
    if (!vector) {
      RMW_SET_ERROR_MSG("Failed to cast vector from ROS message");
      return false;
    }
    vector->resize(array_size);
    for (size_t i = 0; i < vector->size(); ++i) {
      ros_values[i] = (*vector)[i];
    }
  }
  return false;
}

template<uint8_t Id, typename T = typename IdTypeMap<Id>::type>
bool resize_array_and_get_values(
  T * & ros_values,
  void * ros_message,
  const rosidl_typesupport_introspection_c__MessageMember * member,
  size_t array_size)
{
  if (member->array_size_ && !member->is_upper_bound_) {
    ros_values = reinterpret_cast<T *>(static_cast<char *>(ros_message) + member->offset_);
  } else {
    void * untyped_output = static_cast<char *>(ros_message) + member->offset_;
    auto output = static_cast<typename GenericCSequence<Id>::type *>(untyped_output);
    if (!output) {
      RMW_SET_ERROR_MSG("Failed to cast C array from ROS message");
      return false;
    }

    if (output->size < array_size) {
      GenericCSequence<Id>::fini(output);
      if (!GenericCSequence<Id>::init(output, array_size)) {
        RMW_SET_ERROR_MSG("Could not resize array");
        return false;
      }
    }
    ros_values = output->data;
  }
  return true;
}

template<uint8_t Id, typename MessageMemberT, typename T = typename IdTypeMap<Id>::type>
bool get_value(
  const MessageMemberT * member,
  void * & ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  T * ros_values = nullptr;
  if (member->is_array_) {
    size_t array_size;
    if (!get_array_size(member, array_size, dynamic_data, i)) {
      return false;
    }

    resize_array_and_get_values<Id>(ros_values, ros_message, member, array_size);

    DDS_ReturnCode_t status = get_dynamic_data_array<T, T>(
      dynamic_data,
      ros_values,
      array_size,
      i + 1);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to get array value");
      return false;
    }
  } else {
    T * value = reinterpret_cast<T *>(static_cast<char *>(ros_message) + member->offset_);
    DDS_ReturnCode_t status = get_dynamic_data<T, T>(
      dynamic_data,
      *value,
      i + 1);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to get primitive value");
      return false;
    }
  }
  return true;
}

template<typename T, typename DDSType>
T primitive_convert_from_dds(DDSType value)
{
  return value;
}

template<>
bool primitive_convert_from_dds(DDS_Boolean value)
{
  return value == DDS_BOOLEAN_TRUE;
}

template<uint8_t Id, typename DDSType, typename MessageMemberT,
typename T = typename IdTypeMap<Id>::type>
bool get_value_with_different_types(
  const MessageMemberT * member,
  void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  T * ros_values = nullptr;
  if (member->is_array_) {
    size_t array_size;
    if (!get_array_size(member, array_size, dynamic_data, i)) {
      return false;
    }
    resize_array_and_get_values<Id>(ros_values, ros_message, member, array_size);

    if (array_size > 0) {
      if (!ros_values) {
        RMW_SET_ERROR_MSG("failed to cast ros_values from message array");
        return false;
      }
      DDSType * values =
        static_cast<DDSType *>(rmw_allocate(sizeof(DDSType) * array_size));
      if (!values) {
        RMW_SET_ERROR_MSG("failed to allocate memory");
        return false;
      }
      DDS_ReturnCode_t status = get_dynamic_data_array<T, DDSType>(
        dynamic_data,
        values,
        array_size,
        i + 1);
      if (status != DDS_RETCODE_OK) {
        rmw_free(values);
        RMW_SET_ERROR_MSG("failed to get array value");
        return false;
      }
      for (size_t i = 0; i < array_size; ++i) {
        ros_values[i] = primitive_convert_from_dds<T, DDSType>(values[i]);
      }
      rmw_free(values);
    }
  } else {
    DDSType value = 0;
    DDS_ReturnCode_t status = get_dynamic_data<T, DDSType>(
      dynamic_data,
      value,
      i + 1);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to get primitive value");
      return false;
    }
    auto ros_value =
      reinterpret_cast<T *>(static_cast<char *>(ros_message) + member->offset_);
    *ros_value = primitive_convert_from_dds<T, DDSType>(value);
  }
  return true;
}

template<>
bool
get_value_with_different_types<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL, DDS_Boolean>(
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  const uint8_t Id = rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL;
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  if (member->is_array_) {
    size_t array_size;
    if (!get_array_size(member, array_size, dynamic_data, i)) {
      return false;
    }

    if (array_size > 0) {
      DDS_Boolean * values =
        static_cast<DDS_Boolean *>(rmw_allocate(sizeof(DDS_Boolean) * array_size));
      if (!values) {
        RMW_SET_ERROR_MSG("failed to allocate memory");
        return false;
      }
      DDS_ReturnCode_t status = get_dynamic_data_array<bool, DDS_Boolean>(
        dynamic_data,
        values,
        array_size,
        i + 1);
      if (status != DDS_RETCODE_OK) {
        rmw_free(values);
        RMW_SET_ERROR_MSG("failed to get array value");
        return false;
      }
      if (member->array_size_ && !member->is_upper_bound_) {
        bool * ros_values = nullptr;
        resize_array_and_get_values<Id>(ros_values, ros_message, member, array_size);
        if (!ros_values && array_size > 0) {
          RMW_SET_ERROR_MSG("failed to cast ros_values from message array");
          return false;
        }

        for (size_t i = 0; i < array_size; ++i) {
          ros_values[i] = primitive_convert_from_dds<bool, DDS_Boolean>(values[i]);
        }
      } else {
        void * untyped_vector = static_cast<char *>(ros_message) + member->offset_;
        auto vector = static_cast<std::vector<bool> *>(untyped_vector);
        if (!vector) {
          RMW_SET_ERROR_MSG("Failed to cast vector from ROS message");
          return false;
        }
        vector->resize(array_size);
        for (size_t i = 0; i < array_size; ++i) {
          (*vector)[i] = primitive_convert_from_dds<bool, DDS_Boolean>(values[i]);
        }
      }
      rmw_free(values);
    }
  } else {
    DDS_Boolean value = 0;
    DDS_ReturnCode_t status = get_dynamic_data<bool, DDS_Boolean>(
      dynamic_data,
      value,
      i + 1);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to get primitive value");
      return false;
    }
    auto ros_value =
      reinterpret_cast<bool *>(static_cast<char *>(ros_message) + member->offset_);
    *ros_value = primitive_convert_from_dds<bool, DDS_Boolean>(value);
  }
  return true;
}

template<typename T, typename U>
bool string_assign(T dst, U src);

template<>
bool string_assign(rosidl_generator_c__String * dst, char * src)
{
  return rosidl_generator_c__String__assign(dst, src);
}

template<>
bool string_assign(std::string * dst, char * src)
{
  if (!dst || !src) {
    return false;
  }
  *dst = src;
  return true;
}

template<typename T, typename MessageMemberT>
bool get_string_value(
  const MessageMemberT * member,
  void * & ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  if (member->is_array_) {
    T * ros_values = nullptr;
    size_t array_size;
    if (!get_array_size(member, array_size, dynamic_data, i)) {
      return false;
    }
    if (array_size > (std::numeric_limits<DDS_DynamicDataMemberId>::max)()) {
      RMW_SET_ERROR_MSG(
        "failed to get string since the requested string length exceeds the DDS type");
      return false;
    }
    resize_array_and_get_values<rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING>(
      ros_values, ros_message, member, array_size);

    DDS_DynamicData dynamic_data_member(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
      dynamic_data_member,
      NULL,
      static_cast<DDS_DynamicDataMemberId>(i + 1));
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to bind complex member");
      return false;
    }
    for (size_t j = 0; j < array_size; ++j) {
      char * value = nullptr;
      DDS_UnsignedLong size;
      /* TODO(wjwwood): Figure out how value is allocated. Why are we freeing it? */

      status = get_dynamic_data_string(
        &dynamic_data_member,
        value,
        &size,
        j + 1);
      if (status != DDS_RETCODE_OK) {
        if (value) {
          delete[] value;
        }
        RMW_SET_ERROR_MSG("failed to get array value");
        return false;
      }
      if (!string_assign(&ros_values[j], value)) {
        RMW_SET_ERROR_MSG("failed to assign string");
        return false;
      }
      if (value) {
        delete[] value;
      }
    }
    status = dynamic_data->unbind_complex_member(dynamic_data_member);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to unbind complex member");
      return false;
    }
  } else {
    char * value = 0;
    DDS_UnsignedLong size;
    DDS_ReturnCode_t status = get_dynamic_data_string(
      dynamic_data,
      value,
      &size,
      i + 1);
    if (status != DDS_RETCODE_OK) {
      if (value) {
        delete[] value;
      }
      RMW_SET_ERROR_MSG("failed to get primitive value");
      return false;
    }
    auto ros_value =
      reinterpret_cast<T *>(static_cast<char *>(ros_message) + member->offset_);
    if (!string_assign(ros_value, value)) {
      RMW_SET_ERROR_MSG("failed to assign string");
      return false;
    }
    if (value) {
      delete[] value;
    }
  }
  return true;
}

template<
  typename MembersType,
  typename StringType = typename std::conditional<
    std::is_same<MembersType, rosidl_typesupport_introspection_c__MessageMembers>::value,
    rosidl_generator_c__String, std::string
  >::type>
bool take(DDS_DynamicData * dynamic_data, void * ros_message,
  const void * untyped_members)
{
  auto members = static_cast<const MembersType *>(untyped_members);
  if (!members) {
    RMW_SET_ERROR_MSG("members handle is null");
    return false;
  }
  USING_INTROSPECTION_TYPEIDS()
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    auto member = members->members_ + i;
    switch (member->type_id_) {
      case ROS_TYPE_BOOL:
        if (!get_value_with_different_types<ROS_TYPE_BOOL, DDS_Boolean>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_BYTE:
        if (!get_value<ROS_TYPE_BYTE>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_CHAR:
        if (!get_value_with_different_types<ROS_TYPE_CHAR, DDS_Char>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_FLOAT32:
        if (!get_value<ROS_TYPE_FLOAT32>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_FLOAT64:
        if (!get_value<ROS_TYPE_FLOAT64>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT8:
        if (!get_value_with_different_types<ROS_TYPE_INT8, DDS_Octet>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_UINT8:
        if (!get_value<ROS_TYPE_UINT8>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT16:
        if (!get_value<ROS_TYPE_INT16>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_UINT16:
        if (!get_value<ROS_TYPE_UINT16>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT32:
        if (!get_value<ROS_TYPE_INT32>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_UINT32:
        if (!get_value<ROS_TYPE_UINT32>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case ROS_TYPE_INT64:
        if (!get_value_with_different_types<ROS_TYPE_INT64, DDS_LongLong>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_UINT64:
        if (!get_value_with_different_types<ROS_TYPE_UINT64, DDS_UnsignedLongLong>(
            member, ros_message, dynamic_data, i))
        {
          return false;
        }
        break;
      case ROS_TYPE_STRING:
        if (!get_string_value<StringType>(member, ros_message, dynamic_data, i)) {
          return false;
        }
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        {
          if (member->is_array_) {
            void * untyped_member = static_cast<char *>(ros_message) + member->offset_;
            if (!member->array_size_ || member->is_upper_bound_) {
              if (!member->resize_function) {
                RMW_SET_ERROR_MSG("resize function handle is null");
                return false;
              }
            }
            if (!member->get_function) {
              RMW_SET_ERROR_MSG("get function handle is null");
              return false;
            }
            size_t array_size;
            if (!get_array_size(member, array_size, dynamic_data, i)) {
              return false;
            }
            DDS_DynamicData array_data(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
            DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
              array_data,
              NULL,
              i + 1);
            if (status != DDS_RETCODE_OK) {
              RMW_SET_ERROR_MSG("failed to bind complex member");
              return false;
            }
            if (!member->array_size_ || member->is_upper_bound_) {
              member->resize_function(untyped_member, array_size);
            }
            bool errored = false;
            for (size_t j = 0; j < array_size; ++j) {
              void * ros_message;
              {
                void * sub_ros_message = member->get_function(untyped_member, j);
                // offset message pointer since the macro adds the member offset to it
                ros_message = static_cast<char *>(sub_ros_message) - member->offset_;
              }
              DDS_DynamicData * array_data_ptr = &array_data;
              if (!get_submessage_value(member, ros_message, array_data_ptr, j)) {
                break;
              }
            }
            status = dynamic_data->unbind_complex_member(array_data);
            if (status != DDS_RETCODE_OK) {
              RMW_SET_ERROR_MSG("failed to unbind complex member");
              return false;
            }
            if (errored) {
              RMW_SET_ERROR_MSG("get_submessage_value failed");
              return false;
            }
          } else {
            if (!get_submessage_value(member, ros_message, dynamic_data, i)) {
              return false;
            }
          }
        }
        break;
      default:
        RMW_SET_ERROR_MSG(
          (std::string("unknown type id ") + std::to_string(member->type_id_)).c_str());
        return false;
    }
  }
  return true;
}

template<typename MessageMemberT>
bool get_submessage_value(
  const MessageMemberT * member,
  void * ros_message,
  DDS_DynamicData * dynamic_data,
  size_t i)
{
  using MembersT = typename GenericMembersT<MessageMemberT>::type;
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("DDS_DynamicData pointer was NULL!");
    return false;
  }
  DDS_DynamicData sub_dynamic_data(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
  DDS_ReturnCode_t status = dynamic_data->bind_complex_member(
    sub_dynamic_data,
    NULL,
    static_cast<DDS_DynamicDataMemberId>(i + 1));
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to bind complex member");
    return false;
  }
  void * sub_ros_message = static_cast<char *>(ros_message) + member->offset_;
  if (!member->members_) {
    RMW_SET_ERROR_MSG("members handle is null");
    return false;
  }
  bool success = take<MembersT>(
    &sub_dynamic_data, sub_ros_message, member->members_->data);
  status = dynamic_data->unbind_complex_member(sub_dynamic_data);
  if (!success) {
    RMW_SET_ERROR_MSG("take failed for submessage");
    return false;
  }
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to unbind complex member");
    return false;
  }
  return true;
}

/********** end get_*values functions **********/

#endif  // TEMPLATES_HPP_
