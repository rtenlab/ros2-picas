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

#ifndef UTILITY_TEMPLATES_HPP_
#define UTILITY_TEMPLATES_HPP_

#include <iostream>
#include <limits>
#include <sstream>
#include <string>

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

#include "rmw/error_handling.h"

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

template<typename MembersType>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_LOCAL
inline std::string
_create_type_name(const void * untyped_members)
{
  auto members = static_cast<const MembersType *>(untyped_members);
  if (!members) {
    RMW_SET_ERROR_MSG("members handle is null");
    return "";
  }
  std::ostringstream ss;
  std::string message_namespace(members->message_namespace_);
  if (!message_namespace.empty()) {
    ss << message_namespace << "::";
  }
  ss << "dds_::" << members->message_name_ << "_";
  return ss.str();
}

template<typename ServiceType>
const void * get_request_ptr(const void * untyped_service_members)
{
  auto service_members = static_cast<const ServiceType *>(untyped_service_members);
  if (!service_members) {
    RMW_SET_ERROR_MSG("service members handle is null");
    return NULL;
  }
  return service_members->request_members_;
}

template<typename ServiceType>
const void * get_response_ptr(const void * untyped_service_members)
{
  auto service_members = static_cast<const ServiceType *>(untyped_service_members);
  if (!service_members) {
    RMW_SET_ERROR_MSG("service members handle is null");
    return NULL;
  }
  return service_members->response_members_;
}

template<typename MembersType>
DDS_TypeCode * create_type_code(
  std::string type_name, const void * untyped_members, const char * introspection_identifier)
{
  auto members = static_cast<const MembersType *>(untyped_members);
  if (!members) {
    RMW_SET_ERROR_MSG("members handle is null");
    return NULL;
  }

  DDS_TypeCodeFactory * factory = DDS_TypeCodeFactory::get_instance();
  if (!factory) {
    RMW_SET_ERROR_MSG("failed to get typecode factory");
    return NULL;
  }
  // Past this point, a failure results in unrolling code in the goto fail block.
  DDS_StructMemberSeq struct_members;
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  // Start initializing elements.
  DDS_TypeCode * type_code = factory->create_struct_tc(type_name.c_str(), struct_members, ex);
  if (!type_code || ex != DDS_NO_EXCEPTION_CODE) {
    RMW_SET_ERROR_MSG("failed to create struct typecode");
    goto fail;
  }

  USING_INTROSPECTION_TYPEIDS()
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    auto * member = members->members_ + i;
    const DDS_TypeCode * member_type_code = nullptr;
    DDS_TypeCode * member_type_code_non_const = nullptr;
    switch (member->type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        member_type_code = factory->get_primitive_tc(DDS_TK_BOOLEAN);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
        member_type_code = factory->get_primitive_tc(DDS_TK_OCTET);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        member_type_code = factory->get_primitive_tc(DDS_TK_CHAR);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
        member_type_code = factory->get_primitive_tc(DDS_TK_FLOAT);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        member_type_code = factory->get_primitive_tc(DDS_TK_DOUBLE);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        member_type_code = factory->get_primitive_tc(DDS_TK_OCTET);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        member_type_code = factory->get_primitive_tc(DDS_TK_OCTET);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        member_type_code = factory->get_primitive_tc(DDS_TK_SHORT);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        member_type_code = factory->get_primitive_tc(DDS_TK_USHORT);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        member_type_code = factory->get_primitive_tc(DDS_TK_LONG);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        member_type_code = factory->get_primitive_tc(DDS_TK_ULONG);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        member_type_code = factory->get_primitive_tc(DDS_TK_LONGLONG);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        member_type_code = factory->get_primitive_tc(DDS_TK_ULONGLONG);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        {
          DDS_UnsignedLong max_string_size;
          if (member->string_upper_bound_) {
            if (member->string_upper_bound_ > (std::numeric_limits<DDS_UnsignedLong>::max)()) {
              RMW_SET_ERROR_MSG(
                "failed to create string typecode since the upper bound exceeds the DDS type");
              goto fail;
            }
            max_string_size = static_cast<DDS_UnsignedLong>(member->string_upper_bound_);
          } else {
            max_string_size = RTI_INT32_MAX;
          }
          member_type_code_non_const = factory->create_string_tc(max_string_size, ex);
          member_type_code = member_type_code_non_const;
        }
        if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) {
          RMW_SET_ERROR_MSG("failed to create string typecode");
          goto fail;
        }
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        {
          if (!member->members_) {
            RMW_SET_ERROR_MSG("members handle is null");
            return NULL;
          }
          auto sub_members = static_cast<const MembersType *>(member->members_->data);
          if (!sub_members) {
            RMW_SET_ERROR_MSG("sub members handle is null");
            return NULL;
          }
          std::string field_type_name = _create_type_name<MembersType>(sub_members);
          member_type_code = create_type_code<MembersType>(field_type_name, sub_members,
              introspection_identifier);
          if (!member_type_code) {
            // error string was set within the function
            goto fail;
          }
        }
        break;
      default:
        RMW_SET_ERROR_MSG(
          (std::string("unknown type id ") + std::to_string(member->type_id_)).c_str());
        goto fail;
    }
    if (!member_type_code) {
      RMW_SET_ERROR_MSG("failed to create typecode");
      goto fail;
    }
    if (member->is_array_) {
      if (member->array_size_) {
        if (member->array_size_ > (std::numeric_limits<DDS_UnsignedLong>::max)()) {
          RMW_SET_ERROR_MSG("failed to create array typecode since the size exceeds the DDS type");
          goto fail;
        }
        DDS_UnsignedLong array_size = static_cast<DDS_UnsignedLong>(member->array_size_);
        if (!member->is_upper_bound_) {
          member_type_code_non_const = factory->create_array_tc(array_size, member_type_code, ex);
          member_type_code = member_type_code_non_const;
          if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) {
            RMW_SET_ERROR_MSG("failed to create array typecode");
            goto fail;
          }
        } else {
          member_type_code_non_const =
            factory->create_sequence_tc(array_size, member_type_code, ex);
          member_type_code = member_type_code_non_const;
          if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) {
            RMW_SET_ERROR_MSG("failed to create sequence typecode");
            goto fail;
          }
        }
      } else {
        member_type_code_non_const = factory->create_sequence_tc(
          RTI_INT32_MAX, member_type_code, ex);
        member_type_code = member_type_code_non_const;
        if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) {
          RMW_SET_ERROR_MSG("failed to create sequence typecode");
          goto fail;
        }
      }
    }
    auto zero_based_index = type_code->add_member(
      (std::string(member->name_) + "_").c_str(),
      i,
      member_type_code,
      DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    if (ex != DDS_NO_EXCEPTION_CODE) {
      RMW_SET_ERROR_MSG("failed to add member");
      goto fail;
    }
    if (zero_based_index != i) {
      RMW_SET_ERROR_MSG("unexpected member index");
      return NULL;
    }
  }
  // since empty message definitions are not supported
  // we have to add the same dummy field as in rosidl_generator_dds_idl
  if (members->member_count_ == 0) {
    const DDS_TypeCode * member_type_code;
    member_type_code = factory->get_primitive_tc(DDS_TK_BOOLEAN);
    if (!member_type_code) {
      RMW_SET_ERROR_MSG("failed to get primitive typecode");
      goto fail;
    }
    type_code->add_member("_dummy", DDS_TYPECODE_MEMBER_ID_INVALID,
      member_type_code,
      DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
    if (ex != DDS_NO_EXCEPTION_CODE) {
      RMW_SET_ERROR_MSG("failed to add member");
      goto fail;
    }
  }
  DDS_StructMemberSeq_finalize(&struct_members);
  return type_code;
fail:
  if (type_code) {
    if (factory) {
      DDS_ExceptionCode_t exc = DDS_NO_EXCEPTION_CODE;
      factory->delete_tc(type_code, exc);
      if (exc != DDS_NO_EXCEPTION_CODE) {
        std::stringstream ss;
        ss << "failed to delete type code during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    } else {
      std::stringstream ss;
      ss << "leaking type code during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  return nullptr;
}

#endif  // UTILITY_TEMPLATES_HPP_
