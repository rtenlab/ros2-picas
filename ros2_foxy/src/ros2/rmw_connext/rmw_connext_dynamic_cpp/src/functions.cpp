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

#ifdef Connext_GLIBCXX_USE_CXX11_ABI_ZERO
#define _GLIBCXX_USE_CXX11_ABI 0
#endif

#include <cassert>
#include <exception>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#include <ndds/connext_cpp/connext_cpp_replier_details.h>
#include <ndds/connext_cpp/connext_cpp_requester_details.h>
#include <ndds/ndds_cpp.h>
#include <ndds/ndds_requestreply_cpp.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// TODO(karsten1987): Introduce c_rcutils.h
#include "rcutils/types.h"
#include "rcutils/split.h"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/init.h"
#include "rmw/rmw.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/types.h"

#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

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

#include "rmw_connext_shared_cpp/shared_functions.hpp"
#include "rmw_connext_shared_cpp/topic_endpoint_info.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

#include "./macros.hpp"
#include "./publish_take.hpp"
#include "./utility_templates.hpp"

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_LOCAL
inline std::string
_create_type_name(
  const void * untyped_members,
  const char * typesupport)
{
  if (using_introspection_c_typesupport(typesupport)) {
    return _create_type_name<rosidl_typesupport_introspection_c__MessageMembers>(untyped_members);
  } else if (using_introspection_cpp_typesupport(typesupport)) {
    return _create_type_name<rosidl_typesupport_introspection_cpp::MessageMembers>(untyped_members);
  }
  RMW_SET_ERROR_MSG("Unknown typesupport identifier");
  return "";
}

const void * get_request_ptr(const void * untyped_service_members, const char * typesupport)
{
  if (using_introspection_c_typesupport(typesupport)) {
    return get_request_ptr<rosidl_typesupport_introspection_c__ServiceMembers>(
      untyped_service_members);
  } else if (using_introspection_cpp_typesupport(typesupport)) {
    return get_request_ptr<rosidl_typesupport_introspection_cpp::ServiceMembers>(
      untyped_service_members);
  }
  RMW_SET_ERROR_MSG("Unknown typesupport identifier");
  return NULL;
}

const void * get_response_ptr(const void * untyped_service_members, const char * typesupport)
{
  if (using_introspection_c_typesupport(typesupport)) {
    return get_response_ptr<rosidl_typesupport_introspection_c__ServiceMembers>(
      untyped_service_members);
  } else if (using_introspection_cpp_typesupport(typesupport)) {
    return get_response_ptr<rosidl_typesupport_introspection_cpp::ServiceMembers>(
      untyped_service_members);
  }
  RMW_SET_ERROR_MSG("Unknown typesupport identifier");
  return NULL;
}

// This extern "C" prevents accidental overloading of functions. With this in
// place, overloading produces an error rather than a new C++ symbol.
extern "C"
{
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_EXPORT
const char * rti_connext_dynamic_identifier = "rmw_connext_dynamic_cpp";
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_EXPORT
const char * rti_connext_dynamic_serialization_format = "cdr";

struct CustomPublisherInfo
{
  const char * typesupport_identifier;
  DDSDynamicDataTypeSupport * dynamic_data_type_support_;
  DDSPublisher * dds_publisher_;
  DDSDataWriter * data_writer_;
  DDSDynamicDataWriter * dynamic_writer_;
  DDS_TypeCode * type_code_;
  const void * untyped_members_;
  DDS_DynamicData * dynamic_data;
  rmw_gid_t publisher_gid;
};

struct CustomSubscriberInfo
{
  const char * typesupport_identifier;
  DDSDynamicDataTypeSupport * dynamic_data_type_support_;
  DDSDynamicDataReader * dynamic_reader_;
  DDSDataReader * data_reader_;
  DDSReadCondition * read_condition_;
  DDSSubscriber * dds_subscriber_;
  bool ignore_local_publications;
  DDS_TypeCode * type_code_;
  const void * untyped_members_;
  DDS_DynamicData * dynamic_data;
};

struct ConnextDynamicServiceInfo
{
  const char * typesupport_identifier;
  connext::Replier<DDS_DynamicData, DDS_DynamicData> * replier_;
  DDSDataReader * request_datareader_;
  DDSReadCondition * read_condition_;
  DDS::DynamicDataTypeSupport * request_type_support_;
  DDS::DynamicDataTypeSupport * response_type_support_;
  DDS_TypeCode * response_type_code_;
  DDS_TypeCode * request_type_code_;
  const void * untyped_request_members_;
  const void * untyped_response_members_;
};

struct ConnextDynamicClientInfo
{
  const char * typesupport_identifier;
  connext::Requester<DDS_DynamicData, DDS_DynamicData> * requester_;
  DDSDataReader * response_datareader_;
  DDSReadCondition * read_condition_;
  DDS::DynamicDataTypeSupport * request_type_support_;
  DDS::DynamicDataTypeSupport * response_type_support_;
  DDS_TypeCode * response_type_code_;
  DDS_TypeCode * request_type_code_;
  const void * untyped_request_members_;
  const void * untyped_response_members_;
};


const char *
rmw_get_implementation_identifier()
{
  return rti_connext_dynamic_identifier;
}

const char *
rmw_get_serialization_format()
{
  return rti_connext_dynamic_serialization_format;
}

rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  if (NULL != init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  init_options->instance_id = 0;
  init_options->implementation_identifier = rti_connext_dynamic_identifier;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src,
    src->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (NULL != dst->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *dst = *src;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&(init_options->allocator), return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options,
    init_options->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  *init_options = rmw_get_zero_initialized_init_options();
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  context->instance_id = options->instance_id;
  context->implementation_identifier = rti_connext_dynamic_identifier;
  context->impl = nullptr;
  return init();
}

rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // Nothing to do here for now.
  // This is just the middleware's notification that shutdown was called.
  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // context impl is explicitly supposed to be nullptr for now, see rmw_init's code
  // RCUTILS_CHECK_ARGUMENT_FOR_NULL(context->impl, RMW_RET_INVALID_ARGUMENT);
  *context = rmw_get_zero_initialized_context();
  return RMW_RET_OK;
}

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  const rmw_security_options_t * security_options,
  bool localhost_only)
{
  return create_node(
    rti_connext_dynamic_identifier, context, name, namespace_, domain_id, security_options,
    localhost_only);
}

rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  return destroy_node(rti_connext_dynamic_identifier, node);
}

rmw_ret_t
destroy_type_code(DDS_TypeCode * type_code)
{
  DDS_TypeCodeFactory * factory = NULL;
  factory = DDS_TypeCodeFactory::get_instance();
  if (!factory) {
    RMW_SET_ERROR_MSG("failed to get typecode factory");
    return RMW_RET_ERROR;
  }

  DDS_ExceptionCode_t ex;
  factory->delete_tc(type_code, ex);
  if (ex != DDS_NO_EXCEPTION_CODE) {
    RMW_SET_ERROR_MSG("failed to delete type code struct");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

DDS_TypeCode * _create_type_code(
  std::string type_name, const void * untyped_members, const char * typesupport)
{
  if (using_introspection_c_typesupport(typesupport)) {
    return create_type_code<rosidl_typesupport_introspection_c__MessageMembers>(
      type_name, untyped_members, typesupport);
  } else if (using_introspection_cpp_typesupport(typesupport)) {
    return create_type_code<rosidl_typesupport_introspection_cpp::MessageMembers>(
      type_name, untyped_members, typesupport);
  }

  RMW_SET_ERROR_MSG("Unknown typesupport identifier");
  return NULL;
}

rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return NULL;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return NULL)
  RMW_CONNEXT_EXTRACT_MESSAGE_TYPESUPPORT(type_supports, type_support)

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  if (qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG("QoS 'avoid_ros_namespace_conventions' is not implemented");
    return NULL;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  auto participant = static_cast<DDSDomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return NULL;
  }

  DDS_DomainParticipantQos participant_qos;
  DDS_ReturnCode_t status = participant->get_qos(participant_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get participant qos");
    return NULL;
  }
  // Past this point, a failure results in unrolling code in the goto fail block.
  rmw_publisher_t * publisher = nullptr;
  DDS_TypeCode * type_code = nullptr;
  void * buf = nullptr;
  DDSDynamicDataTypeSupport * ddts = nullptr;
  DDS_PublisherQos publisher_qos;
  DDSPublisher * dds_publisher = nullptr;
  DDSTopic * topic = nullptr;
  DDSTopicDescription * topic_description = nullptr;
  DDS_DataWriterQos datawriter_qos;
  DDSDataWriter * topic_writer = nullptr;
  DDSDynamicDataWriter * dynamic_writer = nullptr;
  DDS_DynamicData * dynamic_data = nullptr;
  CustomPublisherInfo * custom_publisher_info = nullptr;
  std::string type_name = _create_type_name(type_support->data,
      type_support->typesupport_identifier);

  // memory allocations for namespacing
  rcutils_string_array_t name_tokens;
  char * partition_str = nullptr;
  char * topic_str = nullptr;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // Start initializing elements.
  publisher = rmw_publisher_allocate();
  if (!publisher) {
    RMW_SET_ERROR_MSG("failed to allocate memory for publisher");
    goto fail;
  }
  publisher->can_loan_messages = false;

  type_code = _create_type_code(
    type_name, type_support->data, type_support->typesupport_identifier);
  if (!type_code) {
    // error string was set within the function
    goto fail;
  }

  // Allocate memory for the DDSDynamicDataTypeSupport object.
  buf = rmw_allocate(sizeof(DDSDynamicDataTypeSupport));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the DDSDynamicDataTypeSupport in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(
    ddts, buf,
    goto fail,
    DDSDynamicDataTypeSupport, type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT)
  buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.
  status = ddts->register_type(participant, type_name.c_str());
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to register type");
    // Delete ddts to prevent the goto fail block from trying to unregister_type.
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      ddts->~DDSDynamicDataTypeSupport(), DDSDynamicDataTypeSupport)
    rmw_free(ddts);
    ddts = nullptr;
    goto fail;
  }

  status = participant->get_default_publisher_qos(publisher_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default publisher qos");
    goto fail;
  }

  // allocates memory, but doesn't have to be freed.
  // partition operater takes ownership of it.
  printf("Original publisher topic name: %s\n", topic_name);
  if (rcutils_split_last(topic_name, '/', allocator, &name_tokens) != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    goto fail;
  }
  partition_str = NULL;
  topic_str = NULL;
  if (name_tokens.size == 2) {
    partition_str = name_tokens.data[0];
    topic_str = name_tokens.data[1];
  } else {
    // We want to make sure at this point that the topic name
    // has already been prefixed with the appropriate token `rt`, etc.
    // In this case, even without specifying an explicit namespace
    // the given topic name should have at least one slash.
    RMW_SET_ERROR_MSG("Split function on topic name failed.");
    goto fail;
  }

  // we have to set the partition array to length 1
  // and then set the partition_str in it
  printf("Creating DDS Partition with %s\n", partition_str);
  publisher_qos.partition.name.ensure_length(1, 1);
  publisher_qos.partition.name[0] = partition_str;

  dds_publisher = participant->create_publisher(
    publisher_qos, NULL, DDS_STATUS_MASK_NONE);
  if (!dds_publisher) {
    RMW_SET_ERROR_MSG("failed to create publisher");
    goto fail;
  }

  topic_description = participant->lookup_topicdescription(topic_str);
  if (!topic_description) {
    DDS_TopicQos default_topic_qos;
    status = participant->get_default_topic_qos(default_topic_qos);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to get default topic qos");
      goto fail;
    }

    topic = participant->create_topic(
      topic_str, type_name.c_str(), default_topic_qos, NULL, DDS_STATUS_MASK_NONE);
    if (!topic) {
      RMW_SET_ERROR_MSG("failed to create topic");
      goto fail;
    }
  } else {
    DDS_Duration_t timeout = DDS_Duration_t::from_seconds(0);
    topic = participant->find_topic(topic_str, timeout);
    if (!topic) {
      RMW_SET_ERROR_MSG("failed to find topic");
      goto fail;
    }
  }

  if (!get_datawriter_qos(participant, *qos_profile, datawriter_qos)) {
    // error string was set within the function
    goto fail;
  }

  topic_writer = dds_publisher->create_datawriter(
    topic, datawriter_qos, NULL, DDS_STATUS_MASK_NONE);
  if (!topic_writer) {
    RMW_SET_ERROR_MSG("failed to create data writer");
    goto fail;
  }

  dynamic_writer = DDSDynamicDataWriter::narrow(topic_writer);
  if (!dynamic_writer) {
    RMW_SET_ERROR_MSG("failed to narrow data writer");
    goto fail;
  }

  dynamic_data = ddts->create_data();
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("failed to create data");
    goto fail;
  }

  // Allocate memory for the CustomPublisherInfo object.
  buf = rmw_allocate(sizeof(CustomPublisherInfo));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the CustomPublisherInfo in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(custom_publisher_info, buf, goto fail, CustomPublisherInfo, )
  buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.
  custom_publisher_info->dynamic_data_type_support_ = ddts;
  custom_publisher_info->dds_publisher_ = dds_publisher;
  custom_publisher_info->data_writer_ = topic_writer;
  custom_publisher_info->dynamic_writer_ = dynamic_writer;
  custom_publisher_info->type_code_ = type_code;
  custom_publisher_info->untyped_members_ = type_support->data;
  custom_publisher_info->dynamic_data = dynamic_data;
  custom_publisher_info->publisher_gid.implementation_identifier = rti_connext_dynamic_identifier;
  custom_publisher_info->typesupport_identifier = type_support->typesupport_identifier;
  static_assert(
    sizeof(ConnextPublisherGID) <= RMW_GID_STORAGE_SIZE,
    "RMW_GID_STORAGE_SIZE insufficient to store the rmw_connext_dynamic_cpp GID implemenation."
  );
  // Zero the gid memory before using it.
  memset(custom_publisher_info->publisher_gid.data, 0, RMW_GID_STORAGE_SIZE);
  {
    auto publisher_gid =
      reinterpret_cast<ConnextPublisherGID *>(custom_publisher_info->publisher_gid.data);
    publisher_gid->publication_handle = topic_writer->get_instance_handle();
  }
  custom_publisher_info->publisher_gid.implementation_identifier = rti_connext_dynamic_identifier;

  publisher->implementation_identifier = rti_connext_dynamic_identifier;
  publisher->data = custom_publisher_info;
  publisher->topic_name = reinterpret_cast<const char *>(rmw_allocate(strlen(topic_name) + 1));
  if (!publisher->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(publisher->topic_name), topic_name, strlen(topic_name) + 1);

  node_info->publisher_listener->add_information(
    node_info->participant->get_instance_handle(),
    dds_publisher->get_instance_handle(),
    topic_name,
    type_name,
    EntityType::Publisher);
  node_info->publisher_listener->trigger_graph_guard_condition();

  return publisher;
fail:
  // Something went wrong, unwind anything that's already been done.
  if (custom_publisher_info) {
    rmw_free(custom_publisher_info);
  }
  if (dynamic_data) {
    if (ddts) {
      if (ddts->delete_data(dynamic_data) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to delete dynamic data during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    } else {
      std::stringstream ss;
      ss << "leaking dynamic data during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (topic_writer) {
    if (dds_publisher) {
      if (dds_publisher->delete_datawriter(topic_writer) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to delete datawriter during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    } else {
      std::stringstream ss;
      ss << "leaking datawriter during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (participant) {
    if (dds_publisher) {
      if (participant->delete_publisher(dds_publisher) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to delete publisher during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
    if (ddts) {
      // TODO(wjwwood) Cannot unregister and free type here, in case another topic is using the
      // same type.
      // Should be cleaned up when the node is destroyed.
      // If we figure out a way to unregister types when they are not being used, then we can
      // add this code back in:
      /*
      if (ddts->unregister_type(participant, type_name.c_str()) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to unregister type during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
      // Call destructor directly since we used a placement new to construct it.
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        ddts->~DDSDynamicDataTypeSupport(), DDSDynamicDataTypeSupport)
      rmw_free(ddts);
      */
    }
  } else if (dds_publisher || ddts) {
    std::stringstream ss;
    ss << "leaking type registration and/or publisher during handling of failure at " <<
      __FILE__ << ":" << __LINE__ << '\n';
    (std::cerr << ss.str()).flush();
  }
  if (type_code) {
    if (destroy_type_code(type_code) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "leaking type code during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
      ss.clear();
      ss << "  error: " << rmw_get_error_string().str << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (publisher) {
    rmw_publisher_free(publisher);
  }
  if (buf) {
    rmw_free(buf);
  }

  // cleanup namespacing
  if (rcutils_string_array_fini(&name_tokens) != RCUTILS_RET_OK) {
    fprintf(stderr, "Failed to destroy the token string array\n");
  }

  return NULL;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void) publisher;
  (void) ros_message;
  (void) allocation;

  RMW_SET_ERROR_MSG("rmw_publish_loaned_message not implemented for rmw_connext_dynamic_cpp");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  (void) publisher;
  (void) type_support;
  (void) ros_message;

  RMW_SET_ERROR_MSG("rmw_borrow_loaned_message not implemented for rmw_connext_dynamic_cpp");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  (void) publisher;
  (void) loaned_message;

  RMW_SET_ERROR_MSG(
    "rmw_return_loaned_message_from_publisher not implemented for rmw_connext_dynamic_cpp");
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);
  if (!info) {
    RMW_SET_ERROR_MSG("publisher internal data is invalid");
    return RMW_RET_ERROR;
  }
  DDS::DataWriter * data_writer = info->data_writer_;
  if (!data_writer) {
    RMW_SET_ERROR_MSG("publisher internal data writer is invalid");
    return RMW_RET_ERROR;
  }
  DDS::DataWriterQos dds_qos;
  DDS::ReturnCode_t status = data_writer->get_qos(dds_qos);
  if (DDS::RETCODE_OK != status) {
    RMW_SET_ERROR_MSG("publisher can't get data writer qos policies");
    return RMW_RET_ERROR;
  }

  if (!data_writer) {
    RMW_SET_ERROR_MSG("publisher internal dds publisher is invalid");
    return RMW_RET_ERROR;
  }
  switch (dds_qos.history.kind) {
    case DDS_KEEP_LAST_HISTORY_QOS:
      qos->history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      break;
    case DDS_KEEP_ALL_HISTORY_QOS:
      qos->history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      break;
    default:
      qos->history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
      break;
  }
  switch (dds_qos.durability.kind) {
    case DDS_TRANSIENT_LOCAL_DURABILITY_QOS:
      qos->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      break;
    case DDS_VOLATILE_DURABILITY_QOS:
      qos->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
      break;
    default:
      qos->durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
      break;
  }
  switch (dds_qos.reliability.kind) {
    case DDS_BEST_EFFORT_RELIABILITY_QOS:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      break;
    case DDS_RELIABLE_RELIABILITY_QOS:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      break;
    default:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
      break;
  }
  qos->depth = static_cast<size_t>(dds_qos.history.depth);

  return RMW_RET_OK;
}

void *
rmw_allocate_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  size_t message_size)
{
  (void) publisher;
  (void) type_support;
  (void) message_size;

  return nullptr;
}

rmw_ret_t
rmw_deallocate_loaned_message(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  (void) publisher;
  (void) loaned_message;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!publisher) {
    RMW_SET_ERROR_MSG("publisher handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }
  auto participant = static_cast<DDSDomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return RMW_RET_ERROR;
  }

  auto custom_publisher_info = static_cast<CustomPublisherInfo *>(publisher->data);
  if (custom_publisher_info) {
    node_info->publisher_listener->remove_information(
      custom_publisher_info->dds_publisher_->get_instance_handle(), EntityType::Publisher);
    node_info->publisher_listener->trigger_graph_guard_condition();
    DDSDynamicDataTypeSupport * ddts = custom_publisher_info->dynamic_data_type_support_;
    if (ddts) {
      if (custom_publisher_info->dynamic_data) {
        if (ddts->delete_data(custom_publisher_info->dynamic_data) != DDS_RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete dynamic data");
          return RMW_RET_ERROR;
        }
        custom_publisher_info->dynamic_data = nullptr;
      }
      std::string type_name = _create_type_name(
        custom_publisher_info->untyped_members_,
        custom_publisher_info->typesupport_identifier);
      // TODO(wjwwood) Cannot unregister and free type here, in case another topic is using the
      // same type.
      // Should be cleaned up when the node is destroyed.
      // If we figure out a way to unregister types when they are not being used, then we can
      // add this code back in:
      /*
      if (ddts->unregister_type(participant, type_name.c_str()) != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG(("failed to unregister type: " + type_name).c_str());
        return RMW_RET_ERROR;
      }
      rmw_free(ddts);
      */
    }
    custom_publisher_info->dynamic_data_type_support_ = nullptr;
    DDSPublisher * dds_publisher = custom_publisher_info->dds_publisher_;
    if (dds_publisher) {
      auto data_writer = custom_publisher_info->data_writer_;
      if (data_writer) {
        if (dds_publisher->delete_datawriter(data_writer) != DDS_RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete datawriter");
          return RMW_RET_ERROR;
        }
      }
      custom_publisher_info->data_writer_ = nullptr;
      custom_publisher_info->dynamic_writer_ = nullptr;
      if (dds_publisher->delete_contained_entities() != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to delete contained entities for publisher");
        return RMW_RET_ERROR;
      }
      if (participant->delete_publisher(dds_publisher) != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to delete publisher");
        return RMW_RET_ERROR;
      }
    }
    custom_publisher_info->dds_publisher_ = nullptr;
    if (custom_publisher_info->type_code_) {
      if (destroy_type_code(custom_publisher_info->type_code_) != RMW_RET_OK) {
        // Error string already set.
        return RMW_RET_ERROR;
      }
    }
    custom_publisher_info->type_code_ = nullptr;
    rmw_free(custom_publisher_info);
  }
  if (publisher->topic_name) {
    rmw_free(const_cast<char *>(publisher->topic_name));
  }
  publisher->data = nullptr;
  rmw_publisher_free(publisher);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish(const rmw_publisher_t * publisher, const void * ros_message)
{
  if (!publisher) {
    RMW_SET_ERROR_MSG("publisher handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!ros_message) {
    RMW_SET_ERROR_MSG("ros message handle is null");
    return RMW_RET_ERROR;
  }

  CustomPublisherInfo * publisher_info = static_cast<CustomPublisherInfo *>(publisher->data);
  if (!publisher_info) {
    RMW_SET_ERROR_MSG("publisher info handle is null");
    return RMW_RET_ERROR;
  }
  DDSDynamicDataTypeSupport * ddts = publisher_info->dynamic_data_type_support_;
  if (!ddts) {
    RMW_SET_ERROR_MSG("dynamic data type support handle is null");
    return RMW_RET_ERROR;
  }
  DDSDynamicDataWriter * dynamic_writer = publisher_info->dynamic_writer_;
  if (!dynamic_writer) {
    RMW_SET_ERROR_MSG("data writer handle is null");
    return RMW_RET_ERROR;
  }
  DDS_TypeCode * type_code = publisher_info->type_code_;
  if (!type_code) {
    RMW_SET_ERROR_MSG("type code handle is null");
    return RMW_RET_ERROR;
  }
  DDS_DynamicData * dynamic_data = publisher_info->dynamic_data;
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("data handle is null");
    return RMW_RET_ERROR;
  }

  DDS_ReturnCode_t status = dynamic_data->clear_all_members();
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to clear all members");
    return RMW_RET_ERROR;
  }
  bool published = _publish(
    dynamic_data, ros_message, publisher_info->untyped_members_,
    publisher_info->typesupport_identifier);
  if (!published) {
    // error string was set within the function
    return RMW_RET_ERROR;
  }

  status = dynamic_writer->write(*dynamic_data, DDS_HANDLE_NIL);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to write");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher, const rmw_serialized_message_t * serialized_message)
{
  (void) publisher;
  (void) serialized_message;

  RMW_SET_ERROR_MSG(
    "rmw_publish_serialized_message is not implemented for rmw_connext_dynamic_cpp");

  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void) publisher;
  (void) ros_message;
  (void) allocation;

  RMW_SET_ERROR_MSG("rmw_publish_loaned_message is not implemented for rmw_connext_dynamic_cpp");
  return RMW_RET_ERROR;
}

rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  bool ignore_local_publications)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return NULL;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return NULL)

  RMW_CONNEXT_EXTRACT_MESSAGE_TYPESUPPORT(type_supports, type_support)

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  if (qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG("QoS 'avoid_ros_namespace_conventions' is not implemented");
    return NULL;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  auto participant = static_cast<DDSDomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return NULL;
  }

  std::string type_name = _create_type_name(
    type_support->data, type_support->typesupport_identifier);

  DDS_DomainParticipantQos participant_qos;
  DDS_ReturnCode_t status = participant->get_qos(participant_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get participant qos");
    return NULL;
  }
  // Past this point, a failure results in unrolling code in the goto fail block.
  rmw_subscription_t * subscription = nullptr;
  DDS_TypeCode * type_code = nullptr;
  void * buf = nullptr;
  DDSDynamicDataTypeSupport * ddts = nullptr;
  DDS_SubscriberQos subscriber_qos;
  DDSSubscriber * dds_subscriber = nullptr;
  DDSTopic * topic;
  DDSTopicDescription * topic_description = nullptr;
  DDSDataReader * topic_reader = nullptr;
  DDSReadCondition * read_condition = nullptr;
  DDSDynamicDataReader * dynamic_reader = nullptr;
  DDS_DataReaderQos datareader_qos;
  DDS_DynamicData * dynamic_data = nullptr;
  CustomSubscriberInfo * custom_subscriber_info = nullptr;
  // Begin initialization of elements.
  subscription = rmw_subscription_allocate();
  if (!subscription) {
    RMW_SET_ERROR_MSG("failed to allocate memory for subscription");
    goto fail;
  }

  type_code =
    _create_type_code(type_name, type_support->data, type_support->typesupport_identifier);
  if (!type_code) {
    // error string was set within the function
    goto fail;
  }

  // Allocate memory for the DDSDynamicDataTypeSupport object.
  buf = rmw_allocate(sizeof(DDSDynamicDataTypeSupport));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the DDSDynamicDataTypeSupport in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(
    ddts, buf,
    goto fail,
    DDSDynamicDataTypeSupport, type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT);
  buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.
  status = ddts->register_type(participant, type_name.c_str());
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to register type");
    // Delete ddts to prevent the goto fail block from trying to unregister_type.
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      ddts->~DDSDynamicDataTypeSupport(), DDSDynamicDataTypeSupport)
    rmw_free(ddts);
    ddts = nullptr;
    goto fail;
  }

  status = participant->get_default_subscriber_qos(subscriber_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default subscriber qos");
    goto fail;
  }

  dds_subscriber = participant->create_subscriber(
    subscriber_qos, NULL, DDS_STATUS_MASK_NONE);
  if (!dds_subscriber) {
    RMW_SET_ERROR_MSG("failed to create subscriber");
    goto fail;
  }

  topic_description = participant->lookup_topicdescription(topic_name);
  if (!topic_description) {
    DDS_TopicQos default_topic_qos;
    status = participant->get_default_topic_qos(default_topic_qos);
    if (status != DDS_RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to get default topic qos");
      goto fail;
    }

    topic = participant->create_topic(
      topic_name, type_name.c_str(), default_topic_qos, NULL, DDS_STATUS_MASK_NONE);
    if (!topic) {
      RMW_SET_ERROR_MSG("failed to create topic");
      goto fail;
    }
  } else {
    DDS_Duration_t timeout = DDS_Duration_t::from_seconds(0);
    topic = participant->find_topic(topic_name, timeout);
    if (!topic) {
      RMW_SET_ERROR_MSG("failed to find topic");
      goto fail;
    }
  }

  if (!get_datareader_qos(participant, *qos_profile, datareader_qos)) {
    // error string was set within the function
    goto fail;
  }

  topic_reader = dds_subscriber->create_datareader(
    topic, datareader_qos, NULL, DDS_STATUS_MASK_NONE);
  if (!topic_reader) {
    RMW_SET_ERROR_MSG("failed to create datareader");
    goto fail;
  }

  read_condition = topic_reader->create_readcondition(
    DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  dynamic_reader = DDSDynamicDataReader::narrow(topic_reader);
  if (!dynamic_reader) {
    RMW_SET_ERROR_MSG("failed to narrow datareader");
    goto fail;
  }

  dynamic_data = ddts->create_data();
  if (!dynamic_data) {
    RMW_SET_ERROR_MSG("failed to create data");
    goto fail;
  }

  // Allocate memory for the CustomSubscriberInfo object.
  buf = rmw_allocate(sizeof(CustomSubscriberInfo));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the CustomSubscriberInfo in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(custom_subscriber_info, buf, goto fail, CustomSubscriberInfo, )
  buf = nullptr;  // Only free the casted pointer; don't need the buf anymore.
  custom_subscriber_info->dynamic_data_type_support_ = ddts;
  custom_subscriber_info->dynamic_reader_ = dynamic_reader;
  custom_subscriber_info->data_reader_ = topic_reader;
  custom_subscriber_info->read_condition_ = read_condition;
  custom_subscriber_info->dds_subscriber_ = dds_subscriber;
  custom_subscriber_info->ignore_local_publications = ignore_local_publications;
  custom_subscriber_info->type_code_ = type_code;
  custom_subscriber_info->untyped_members_ = type_support->data;
  custom_subscriber_info->dynamic_data = dynamic_data;
  custom_subscriber_info->typesupport_identifier = type_support->typesupport_identifier;

  subscription->implementation_identifier = rti_connext_dynamic_identifier;
  subscription->data = custom_subscriber_info;

  subscription->topic_name = reinterpret_cast<const char *>(rmw_allocate(strlen(topic_name) + 1));
  if (!subscription->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(subscription->topic_name), topic_name, strlen(topic_name) + 1);

  node_info->subscriber_listener->add_information(
    node_info->participant->get_instance_handle(),
    dds_subscriber->get_instance_handle(),
    topic_name,
    type_name,
    EntityType::Subscriber);
  node_info->subscriber_listener->trigger_graph_guard_condition();

  subscription->can_loan_messages = false;
  return subscription;
fail:
  // Something has gone wrong, unroll what has been done.
  if (custom_subscriber_info) {
    rmw_free(custom_subscriber_info);
  }
  if (dynamic_data) {
    if (ddts) {
      if (ddts->delete_data(dynamic_data) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to delete dynamic data during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    } else {
      std::stringstream ss;
      ss << "leaking dynamic data during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (topic_reader) {
    if (read_condition) {
      if (topic_reader->delete_readcondition(read_condition) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "leaking readcondition while handling failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
    if (dds_subscriber) {
      if (dds_subscriber->delete_datareader(topic_reader) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to delete datareader during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    } else {
      std::stringstream ss;
      ss << "leaking datareader during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (participant) {
    if (dds_subscriber) {
      if (participant->delete_subscriber(dds_subscriber) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to delete subscriber during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
    if (ddts) {
      // TODO(wjwwood) Cannot unregister and free type here, in case another topic is using the
      // same type.
      // Should be cleaned up when the node is destroyed.
      // If we figure out a way to unregister types when they are not being used, then we can
      // add this code back in:
      /*
      if (ddts->unregister_type(participant, type_name.c_str()) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "failed to unregister type during handling of failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
      // Call destructor directly since we used a placement new to construct it.
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        ddts->~DDSDynamicDataTypeSupport(), DDSDynamicDataTypeSupport)
      rmw_free(ddts);
      */
    }
  } else if (dds_subscriber || ddts) {
    std::stringstream ss;
    ss << "leaking type registration and/or publisher during handling of failure at " <<
      __FILE__ << ":" << __LINE__ << '\n';
    (std::cerr << ss.str()).flush();
  }
  if (type_code) {
    if (destroy_type_code(type_code) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "leaking type code during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
      ss.clear();
      ss << "  error: " << rmw_get_error_string().str << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (subscription) {
    rmw_subscription_free(subscription);
  }
  if (buf) {
    rmw_free(buf);
  }
  return NULL;
}

rmw_ret_t
rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }
  auto participant = static_cast<DDSDomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return RMW_RET_ERROR;
  }

  auto custom_subscription_info = static_cast<CustomSubscriberInfo *>(subscription->data);
  if (custom_subscription_info) {
    node_info->subscriber_listener->remove_information(
      custom_subscription_info->dds_subscriber_->get_instance_handle(), EntityType::Subscriber);
    node_info->subscriber_listener->trigger_graph_guard_condition();
    DDSDynamicDataTypeSupport * ddts = custom_subscription_info->dynamic_data_type_support_;
    if (ddts) {
      if (custom_subscription_info->dynamic_data) {
        if (ddts->delete_data(custom_subscription_info->dynamic_data) != DDS_RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete dynamic data");
          return RMW_RET_ERROR;
        }
        custom_subscription_info->dynamic_data = nullptr;
      }
      std::string type_name = _create_type_name(
        custom_subscription_info->untyped_members_,
        custom_subscription_info->typesupport_identifier);
      // TODO(wjwwood) Cannot unregister and free type here, in case another topic is using the
      // same type.
      // Should be cleaned up when the node is destroyed.
      // If we figure out a way to unregister types when they are not being used, then we can
      // add this code back in:
      /*
      if (ddts->unregister_type(participant, type_name.c_str()) != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG(("failed to unregister type: " + type_name).c_str());
        return RMW_RET_ERROR;
      }
      rmw_free(ddts);
      */
    }
    custom_subscription_info->dynamic_data_type_support_ = nullptr;
    DDSSubscriber * dds_subscriber = custom_subscription_info->dds_subscriber_;
    if (dds_subscriber) {
      auto data_reader = custom_subscription_info->data_reader_;
      if (data_reader) {
        auto read_condition = custom_subscription_info->read_condition_;
        if (read_condition) {
          if (data_reader->delete_readcondition(read_condition) != DDS_RETCODE_OK) {
            RMW_SET_ERROR_MSG("failed to delete readcondition");
            return RMW_RET_ERROR;
          }
          custom_subscription_info->read_condition_ = nullptr;
        }
        if (dds_subscriber->delete_datareader(data_reader) != DDS_RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete datareader");
          return RMW_RET_ERROR;
        }
      }
      custom_subscription_info->data_reader_ = nullptr;
      custom_subscription_info->dynamic_reader_ = nullptr;
      if (participant->delete_subscriber(dds_subscriber) != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to delete subscriber");
        return RMW_RET_ERROR;
      }
    }
    custom_subscription_info->dds_subscriber_ = nullptr;
    if (custom_subscription_info->type_code_) {
      if (destroy_type_code(custom_subscription_info->type_code_) != RMW_RET_OK) {
        return RMW_RET_ERROR;
      }
    }
    custom_subscription_info->type_code_ = nullptr;
    rmw_free(custom_subscription_info);
  }
  subscription->data = nullptr;
  if (subscription->topic_name) {
    rmw_free(const_cast<char *>(subscription->topic_name));
  }
  rmw_subscription_free(subscription);
  return RMW_RET_OK;
}

rmw_ret_t
_take_impl(const rmw_subscription_t * subscription, void * ros_message, bool * taken,
  DDS_InstanceHandle_t * sending_publication_handle)
{
  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!ros_message) {
    RMW_SET_ERROR_MSG("ros message handle is null");
    return RMW_RET_ERROR;
  }
  if (!taken) {
    RMW_SET_ERROR_MSG("taken handle is null");
    return RMW_RET_ERROR;
  }

  CustomSubscriberInfo * subscriber_info =
    static_cast<CustomSubscriberInfo *>(subscription->data);
  if (!subscriber_info) {
    RMW_SET_ERROR_MSG("subscriber info handle is null");
    return RMW_RET_ERROR;
  }
  DDSDynamicDataTypeSupport * ddts = subscriber_info->dynamic_data_type_support_;
  if (!ddts) {
    RMW_SET_ERROR_MSG("dynamic data type support handle is null");
    return RMW_RET_ERROR;
  }
  DDSDynamicDataReader * dynamic_reader = subscriber_info->dynamic_reader_;
  if (!dynamic_reader) {
    RMW_SET_ERROR_MSG("data reader handle is null");
    return RMW_RET_ERROR;
  }
  DDS_TypeCode * type_code = subscriber_info->type_code_;
  if (!type_code) {
    RMW_SET_ERROR_MSG("type code handle is null");
    return RMW_RET_ERROR;
  }

  DDS_DynamicDataSeq dynamic_data_sequence;
  DDS_SampleInfoSeq sample_infos;
  DDS_ReturnCode_t status = dynamic_reader->take(
    dynamic_data_sequence,
    sample_infos,
    1,
    DDS_ANY_SAMPLE_STATE,
    DDS_ANY_VIEW_STATE,
    DDS_ANY_INSTANCE_STATE);
  if (status == DDS_RETCODE_NO_DATA) {
    *taken = false;
    return RMW_RET_OK;
  }
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to take sample");
    return RMW_RET_ERROR;
  }

  bool ignore_sample = false;
  DDS_SampleInfo & sample_info = sample_infos[0];
  if (!sample_info.valid_data) {
    // skip sample without data
    ignore_sample = true;
  } else if (subscriber_info->ignore_local_publications) {
    // compare the lower 12 octets of the guids from the sender and this receiver
    // if they are equal the sample has been sent from this process and should be ignored
    DDS_GUID_t sender_guid = sample_info.original_publication_virtual_guid;
    DDS_InstanceHandle_t receiver_instance_handle = dynamic_reader->get_instance_handle();
    ignore_sample = true;
    for (size_t i = 0; i < 12; ++i) {
      DDS_Octet * sender_element = &(sender_guid.value[i]);
      DDS_Octet * receiver_element = &(reinterpret_cast<DDS_Octet *>(&receiver_instance_handle)[i]);
      if (*sender_element != *receiver_element) {
        ignore_sample = false;
        break;
      }
    }
  }
  if (sample_info.valid_data && sending_publication_handle != nullptr) {
    *sending_publication_handle = sample_info.publication_handle;
  }

  bool success = true;
  if (!ignore_sample) {
    success = _take(&dynamic_data_sequence[0], ros_message, subscriber_info->untyped_members_,
        subscriber_info->typesupport_identifier);
    if (success) {
      *taken = true;
    }
  }

  dynamic_reader->return_loan(dynamic_data_sequence, sample_infos);

  if (!success) {
    // error string was set within the function
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take(const rmw_subscription_t * subscription, void * ros_message, bool * taken)
{
  return _take_impl(subscription, ros_message, taken, nullptr);
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  if (!message_info) {
    RMW_SET_ERROR_MSG("message info is null");
    return RMW_RET_ERROR;
  }
  DDS_InstanceHandle_t sending_publication_handle;
  auto ret = _take_impl(subscription, ros_message, taken, &sending_publication_handle);
  if (ret != RMW_RET_OK) {
    // Error string is already set.
    return RMW_RET_ERROR;
  }

  rmw_gid_t * sender_gid = &message_info->publisher_gid;
  sender_gid->implementation_identifier = rti_connext_dynamic_identifier;
  memset(sender_gid->data, 0, RMW_GID_STORAGE_SIZE);
  auto detail = reinterpret_cast<ConnextPublisherGID *>(sender_gid->data);
  detail->publication_handle = sending_publication_handle;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken)
{
  (void) subscription;
  (void) serialized_message;
  (void) taken;

  RMW_SET_ERROR_MSG(
    "rmw_take_serialized_message is not implemented for rmw_connext_dyamic_cpp");

  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  (void) subscription;
  (void) serialized_message;
  (void) taken;
  (void) message_info;

  RMW_SET_ERROR_MSG(
    "rmw_take_serialized_message_with_info is not implemented for rmw_connext_dynamic_cpp");

  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void) subscription;
  (void) loaned_message;
  (void) taken;
  (void) allocation;

  RMW_SET_ERROR_MSG("rmw_take_loaned_message not implemented for rmw_connext_dynamic_cpp");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void) subscription;
  (void) loaned_message;
  (void) taken;
  (void) message_info;
  (void) allocation;

  RMW_SET_ERROR_MSG(
    "rmw_take_loaned_message_with_info not implemented for rmw_connext_dynamic_cpp");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  (void) subscription;
  (void) loaned_message;

  RMW_SET_ERROR_MSG(
    "rmw_return_loaned_message_from_subscription not implemented for rmw_connext_dynamic_cpp");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  (void) ros_message;
  (void) type_support;
  (void) serialized_message;

  RMW_SET_ERROR_MSG(
    "rmw_serialize is not implemented for rmw_connext_dynamic_cpp");

  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  (void) serialized_message;
  (void) type_support;
  (void) ros_message;

  RMW_SET_ERROR_MSG(
    "rmw_deserialize is not implemented for rmw_connext_dynamic_cpp");

  return RMW_RET_ERROR;
}

rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  return create_guard_condition(rti_connext_dynamic_identifier, context);
}

rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition)
{
  return destroy_guard_condition(rti_connext_dynamic_identifier, guard_condition);
}

rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition_handle)
{
  return trigger_guard_condition(rti_connext_dynamic_identifier, guard_condition_handle);
}

rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  return create_wait_set(rti_connext_dynamic_identifier, context, max_conditions);
}

rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  return destroy_wait_set(rti_connext_dynamic_identifier, wait_set);
}

rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  return wait<CustomSubscriberInfo, ConnextDynamicServiceInfo, ConnextDynamicClientInfo>
           (rti_connext_dynamic_identifier, subscriptions, guard_conditions, services, clients,
           wait_set,
           wait_timeout);
}

rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return NULL;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return NULL)

  RMW_CONNEXT_EXTRACT_SERVICE_TYPESUPPORT(type_supports, type_support)

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  if (qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG("QoS 'avoid_ros_namespace_conventions' is not implemented");
    return NULL;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  auto participant = static_cast<DDSDomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return NULL;
  }

  const void * untyped_request_members =
    get_request_ptr(type_support->data, type_support->typesupport_identifier);
  if (!untyped_request_members) {
    RMW_SET_ERROR_MSG("couldn't get request members");
    return NULL;
  }
  const void * untyped_response_members = get_response_ptr(type_support->data,
      type_support->typesupport_identifier);
  if (!untyped_response_members) {
    RMW_SET_ERROR_MSG("couldn't get response members");
    return NULL;
  }

  std::string request_type_name = _create_type_name(untyped_request_members,
      type_support->typesupport_identifier);
  std::string response_type_name = _create_type_name(untyped_response_members,
      type_support->typesupport_identifier);

  DDS_DomainParticipantQos participant_qos;
  DDS_ReturnCode_t status = participant->get_qos(participant_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get participant qos");
    return NULL;
  }
  // Past this point, a failure results in unrolling code in the goto fail block.
  rmw_client_t * client = nullptr;
  DDS_TypeCode * request_type_code = nullptr;
  void * buf = nullptr;
  DDS::DynamicDataTypeSupport * request_type_support = nullptr;
  DDS_TypeCode * response_type_code = nullptr;
  DDS::DynamicDataTypeSupport * response_type_support = nullptr;
  DDS_DataReaderQos datareader_qos;
  DDS_DataWriterQos datawriter_qos;
  connext::Requester<DDS_DynamicData, DDS_DynamicData> * requester = nullptr;
  DDSDataReader * response_datareader = nullptr;
  DDSReadCondition * read_condition = nullptr;
  ConnextDynamicClientInfo * client_info = nullptr;

  // Begin initializing elements
  client = rmw_client_allocate();
  if (!client) {
    RMW_SET_ERROR_MSG("failed to allocate memory for client");
    goto fail;
  }

  request_type_code = _create_type_code(request_type_name, untyped_request_members,
      type_support->typesupport_identifier);
  if (!request_type_code) {
    // error string was set within the function
    RMW_SET_ERROR_MSG("Could not set type code for request");
    goto fail;
  }

  // Allocate memory for the DDS::DynamicDataTypeSupport object.
  buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }

  response_type_code = _create_type_code(response_type_name, untyped_response_members,
      type_support->typesupport_identifier);
  if (!response_type_code) {
    // error string was set within the function
    RMW_SET_ERROR_MSG("Could not set type code for response");
    goto fail;
  }

  // Use a placement new to construct the DDS::DynamicDataTypeSupport in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(
    request_type_support, buf,
    goto fail,
    DDS::DynamicDataTypeSupport, request_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT)
  buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.

  if (!request_type_support->is_valid()) {
    RMW_SET_ERROR_MSG("failed to construct dynamic data type support for request");
    goto fail;
  }
  // Allocate memory for the DDS::DynamicDataTypeSupport object.
  buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the DDS::DynamicDataTypeSupport in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(
    response_type_support, buf,
    goto fail,
    DDS::DynamicDataTypeSupport, response_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT)
  buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.

  if (!response_type_support->is_valid()) {
    RMW_SET_ERROR_MSG("failed to construct dynamic data type support for response");
    goto fail;
  }

  // create requester
  {
    if (!get_datareader_qos(participant, *qos_profile, datareader_qos)) {
      // error string was set within the function
      goto fail;
    }
    if (!get_datawriter_qos(participant, *qos_profile, datawriter_qos)) {
      // error string was set within the function
      goto fail;
    }

    connext::RequesterParams requester_params(participant);
    requester_params.service_name(service_name);
    requester_params.request_type_support(request_type_support);
    requester_params.reply_type_support(response_type_support);
    requester_params.datareader_qos(datareader_qos);
    requester_params.datawriter_qos(datawriter_qos);

    // Allocate memory for the Requester object.
    using Requester = connext::Requester<DDS_DynamicData, DDS_DynamicData>;
    buf = rmw_allocate(sizeof(connext::Requester<DDS_DynamicData, DDS_DynamicData>));
    if (!buf) {
      RMW_SET_ERROR_MSG("failed to allocate memory");
      goto fail;
    }
    // Use a placement new to construct the Requester in the preallocated buffer.
    RMW_TRY_PLACEMENT_NEW(
      requester, buf,
      goto fail,
      Requester, requester_params)
    buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.
  }

  response_datareader = requester->get_reply_datareader();
  if (!response_datareader) {
    RMW_SET_ERROR_MSG("failed to get response datareader");
    goto fail;
  }

  read_condition = response_datareader->create_readcondition(
    DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  // Allocate memory for the ConnextDynamicClientInfo object.
  buf = rmw_allocate(sizeof(ConnextDynamicClientInfo));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the ConnextDynamicClientInfo in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(client_info, buf, goto fail, ConnextDynamicClientInfo, )
  buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.
  client_info->requester_ = requester;
  client_info->response_datareader_ = response_datareader;
  client_info->read_condition_ = read_condition;
  client_info->request_type_support_ = request_type_support;
  client_info->response_type_support_ = response_type_support;
  client_info->response_type_code_ = response_type_code;
  client_info->request_type_code_ = request_type_code;
  client_info->untyped_request_members_ = untyped_request_members;
  client_info->untyped_response_members_ = untyped_response_members;
  client_info->typesupport_identifier = type_support->typesupport_identifier;

  client->implementation_identifier = rti_connext_dynamic_identifier;
  client->data = client_info;
  client->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  if (!client->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(client->service_name), service_name, strlen(service_name) + 1);
  return client;
fail:
  if (response_datareader) {
    if (read_condition) {
      if (response_datareader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
        fprintf(stderr, "leaking readcondition while handling failure\n");
      }
    }
  }
  if (client) {
    rmw_client_free(client);
  }
  if (request_type_code) {
    if (destroy_type_code(request_type_code) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "leaking type code during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
      ss.clear();
      ss << "  error: " << rmw_get_error_string().str << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (request_type_support) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      request_type_support->~DynamicDataTypeSupport(), DynamicDataTypeSupport)
    rmw_free(request_type_support);
  }
  if (response_type_code) {
    if (destroy_type_code(response_type_code) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "leaking type code during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
      ss.clear();
      ss << "  error: " << rmw_get_error_string().str << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (response_type_support) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      response_type_support->~DynamicDataTypeSupport(), DynamicDataTypeSupport)
    rmw_free(response_type_support);
  }
  if (requester) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      requester->~Requester(), "connext::Requester<DDS_DynamicData, DDS_DynamicData>")
    rmw_free(requester);
  }
  if (client_info) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      client_info->~ConnextDynamicClientInfo(), ConnextDynamicClientInfo)
    rmw_free(client_info);
  }
  if (buf) {
    rmw_free(buf);
  }
  return NULL;
}

rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  (void)node;
  if (!client) {
    RMW_SET_ERROR_MSG("client handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client handle,
    client->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  auto result = RMW_RET_OK;
  ConnextDynamicClientInfo * client_info = static_cast<ConnextDynamicClientInfo *>(client->data);
  if (client_info) {
    auto response_datareader = client_info->response_datareader_;
    if (response_datareader) {
      auto read_condition = client_info->read_condition_;
      if (response_datareader->delete_readcondition(read_condition) != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to delete readcondition");
        return RMW_RET_ERROR;
      }
      client_info->read_condition_ = nullptr;
    }
    if (client_info->request_type_code_) {
      if (destroy_type_code(client_info->request_type_code_) != RMW_RET_OK) {
        RMW_SET_ERROR_MSG("failed to destroy type code");
        return RMW_RET_ERROR;
      }
    }
    if (client_info->response_type_code_) {
      if (destroy_type_code(client_info->response_type_code_) != RMW_RET_OK) {
        RMW_SET_ERROR_MSG("failed to destroy type code");
        return RMW_RET_ERROR;
      }
    }
    // TODO(dirk-thomas) the deletion break something within Connext
    // afterwards it is not possible to create another client of the same type
    // if (client_info->request_type_support_) {
    //   RMW_TRY_DESTRUCTOR(
    //     client_info->request_type_support_->~DynamicDataTypeSupport(),
    //     DynamicDataTypeSupport, result = RMW_RET_ERROR)
    //   rmw_free(client_info->request_type_support_);
    // }
    // if (client_info->response_type_support_) {
    //   RMW_TRY_DESTRUCTOR(
    //     client_info->response_type_support_->~DynamicDataTypeSupport(),
    //     DynamicDataTypeSupport, result = RMW_RET_ERROR)
    //   rmw_free(client_info->response_type_support_);
    // }
    if (client_info->requester_) {
      RMW_TRY_DESTRUCTOR(
        client_info->requester_->~Requester(),
        "connext::Requester<DDS_DynamicData, DDS_DynamicData>",
        result = RMW_RET_ERROR)
      rmw_free(client_info->requester_);
    }
    if (client->service_name) {
      rmw_free(const_cast<char *>(client->service_name));
    }
  }

  if (client_info) {
    RMW_TRY_DESTRUCTOR(
      client_info->~ConnextDynamicClientInfo(), ConnextDynamicClientInfo, result = RMW_RET_ERROR)
    rmw_free(client_info);
  }

  rmw_client_free(client);

  // TODO(wjwwood): if multiple destructors fail, some of the error messages could be suppressed.
  return result;
}

rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id)
{
  if (!client) {
    RMW_SET_ERROR_MSG("client handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client handle,
    client->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!ros_request) {
    RMW_SET_ERROR_MSG("ros request handle is null");
    return RMW_RET_ERROR;
  }

  ConnextDynamicClientInfo * client_info = static_cast<ConnextDynamicClientInfo *>(client->data);
  if (!client_info) {
    RMW_SET_ERROR_MSG("client info handle is null");
    return RMW_RET_ERROR;
  }
  connext::Requester<DDS_DynamicData, DDS_DynamicData> * requester = client_info->requester_;
  if (!requester) {
    RMW_SET_ERROR_MSG("requester handle is null");
    return RMW_RET_ERROR;
  }

  DDS::DynamicData * sample = client_info->request_type_support_->create_data();
  if (!sample) {
    RMW_SET_ERROR_MSG("failed to create data");
    return RMW_RET_ERROR;
  }
  DDS::WriteParams_t writeParams;
  connext::WriteSampleRef<DDS::DynamicData> request(*sample, writeParams);

  bool published = _publish(
    sample, ros_request, client_info->untyped_request_members_,
    client_info->typesupport_identifier);
  if (!published) {
    // error string was set within the function
    if (client_info->request_type_support_->delete_data(sample) != DDS_RETCODE_OK) {
      std::stringstream ss;
      ss << "leaking dynamic data object while handling error at " <<
        __FILE__ << ":" << __LINE__;
      (std::cerr << ss.str()).flush();
    }
    return RMW_RET_ERROR;
  }

  requester->send_request(request);
  *sequence_id = ((int64_t)request.identity().sequence_number.high) << 32 |
    request.identity().sequence_number.low;

  if (client_info->request_type_support_->delete_data(sample) != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to delete dynamic data object");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return NULL;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return NULL)
  if (node->implementation_identifier != rti_connext_dynamic_identifier) {
    RMW_SET_ERROR_MSG("node handle is not from this rmw implementation");
    return NULL;
  }

  RMW_CONNEXT_EXTRACT_SERVICE_TYPESUPPORT(type_supports, type_support)

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  if (qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG("QoS 'avoid_ros_namespace_conventions' is not implemented");
    return NULL;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  auto participant = static_cast<DDSDomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return NULL;
  }

  const void * untyped_request_members = get_request_ptr(type_support->data,
      type_support->typesupport_identifier);
  const void * untyped_response_members = get_response_ptr(type_support->data,
      type_support->typesupport_identifier);
  std::string request_type_name = _create_type_name(untyped_request_members,
      type_support->typesupport_identifier);
  std::string response_type_name = _create_type_name(untyped_response_members,
      type_support->typesupport_identifier);

  DDS_DomainParticipantQos participant_qos;
  DDS_ReturnCode_t status = participant->get_qos(participant_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get participant qos");
    return NULL;
  }
  // Past this point, a failure results in unrolling code in the goto fail block.
  rmw_service_t * service = nullptr;
  DDS_TypeCode * request_type_code = nullptr;
  void * buf = nullptr;
  DDS::DynamicDataTypeSupport * request_type_support = nullptr;
  DDS_TypeCode * response_type_code = nullptr;
  DDS::DynamicDataTypeSupport * response_type_support = nullptr;
  DDS_DataReaderQos datareader_qos;
  DDS_DataWriterQos datawriter_qos;
  connext::Replier<DDS_DynamicData, DDS_DynamicData> * replier = nullptr;
  DDSDataReader * request_datareader = nullptr;
  DDSReadCondition * read_condition = nullptr;
  ConnextDynamicServiceInfo * server_info = nullptr;
  // Begin initializing elements
  service = rmw_service_allocate();
  if (!service) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }

  request_type_code = _create_type_code(request_type_name, untyped_request_members,
      type_support->typesupport_identifier);
  if (!request_type_code) {
    // error string was set within the function
    RMW_SET_ERROR_MSG("Could not set type code for response");
    goto fail;
  }
  // Allocate memory for the DDS::DynamicDataTypeSupport object.
  buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the DDS::DynamicDataTypeSupport in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(
    request_type_support, buf,
    goto fail,
    DDS::DynamicDataTypeSupport, request_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT)
  buf = nullptr;  // Only free the casted pointer; don't need the buf anymore.

  response_type_code = _create_type_code(response_type_name, untyped_response_members,
      type_support->typesupport_identifier);
  if (!response_type_code) {
    // error string was set within the function
    RMW_SET_ERROR_MSG("Could not set type code for response");
    goto fail;
  }
  // Allocate memory for the DDS::DynamicDataTypeSupport object.
  buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the DDS::DynamicDataTypeSupport in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(
    response_type_support, buf,
    goto fail,
    DDS::DynamicDataTypeSupport, response_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT)
  buf = nullptr;  // Only free the casted pointer; don't need the buf anymore.

  {
    if (!get_datareader_qos(participant, *qos_profile, datareader_qos)) {
      // error string was set within the function
      goto fail;
    }
    if (!get_datawriter_qos(participant, *qos_profile, datawriter_qos)) {
      // error string was set within the function
      goto fail;
    }

    // create requester
    connext::ReplierParams<DDS_DynamicData, DDS_DynamicData> replier_params(participant);
    replier_params.service_name(service_name);
    replier_params.request_type_support(request_type_support);
    replier_params.reply_type_support(response_type_support);
    replier_params.datareader_qos(datareader_qos);
    replier_params.datawriter_qos(datawriter_qos);

    // Allocate memory for the Replier object.
    using Replier = connext::Replier<DDS_DynamicData, DDS_DynamicData>;
    buf = rmw_allocate(sizeof(Replier));
    if (!buf) {
      RMW_SET_ERROR_MSG("failed to allocate memory");
      goto fail;
    }
    // Use a placement new to construct the Replier in the preallocated buffer.
    RMW_TRY_PLACEMENT_NEW(replier, buf, goto fail, Replier, replier_params)
    buf = nullptr;  // Only free casted pointer; don't need the buf pointer anymore.
  }

  request_datareader = replier->get_request_datareader();
  if (!request_datareader) {
    RMW_SET_ERROR_MSG("failed to get request datareader");
    goto fail;
  }

  read_condition = request_datareader->create_readcondition(
    DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  // Allocate memory for the ConnextDynamicServiceInfo object.
  buf = rmw_allocate(sizeof(ConnextDynamicServiceInfo));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the ConnextDynamicServiceInfo in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(server_info, buf, goto fail, ConnextDynamicServiceInfo, )
  buf = nullptr;  // Only free the casted pointer; don't need the buf pointer anymore.
  server_info->replier_ = replier;
  server_info->request_datareader_ = request_datareader;
  server_info->read_condition_ = read_condition;
  server_info->response_type_support_ = response_type_support;
  server_info->untyped_request_members_ = untyped_request_members;
  server_info->untyped_response_members_ = untyped_response_members;
  server_info->typesupport_identifier = type_support->typesupport_identifier;

  service->implementation_identifier = rti_connext_dynamic_identifier;
  service->data = server_info;
  service->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  if (!service->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(service->service_name), service_name, strlen(service_name) + 1);
  return service;
fail:
  if (request_datareader) {
    if (read_condition) {
      if (request_datareader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
        fprintf(stderr, "leaking readcondition while handling failure\n");
      }
    }
  }
  if (service) {
    rmw_service_free(service);
  }
  if (request_type_code) {
    if (destroy_type_code(request_type_code) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "leaking type code during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
      ss.clear();
      ss << "  error: " << rmw_get_error_string().str << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (request_type_support) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      request_type_support->~DynamicDataTypeSupport(), DynamicDataTypeSupport)
    rmw_free(request_type_support);
  }
  if (response_type_code) {
    if (destroy_type_code(response_type_code) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "leaking type code during handling of failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
      ss.clear();
      ss << "  error: " << rmw_get_error_string().str << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (response_type_support) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      response_type_support->~DynamicDataTypeSupport(), DynamicDataTypeSupport)
    rmw_free(response_type_support);
  }
  if (replier) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      replier->~Replier(), "connext::Replier<DDS_DynamicData, DDS_DynamicData>")
    rmw_free(replier);
  }
  if (server_info) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      server_info->~ConnextDynamicServiceInfo(), ConnextDynamicServiceInfo)
    rmw_free(server_info);
  }
  if (buf) {
    rmw_free(buf);
  }
  return NULL;
}

rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  (void)node;
  if (!service) {
    RMW_SET_ERROR_MSG("service handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service handle,
    service->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  auto result = RMW_RET_OK;
  auto service_info = static_cast<ConnextDynamicServiceInfo *>(service->data);
  if (service_info) {
    auto request_datareader = service_info->request_datareader_;
    if (request_datareader) {
      auto read_condition = service_info->read_condition_;
      if (request_datareader->delete_readcondition(read_condition) != DDS_RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to delete readcondition");
        return RMW_RET_ERROR;
      }
      service_info->read_condition_ = nullptr;
    }
    if (service_info->request_type_code_) {
      if (destroy_type_code(service_info->request_type_code_) != RMW_RET_OK) {
        RMW_SET_ERROR_MSG("failed to destroy type code");
        return RMW_RET_ERROR;
      }
    }
    if (service_info->response_type_code_) {
      if (destroy_type_code(service_info->response_type_code_) != RMW_RET_OK) {
        RMW_SET_ERROR_MSG("failed to destroy type code");
        return RMW_RET_ERROR;
      }
    }
    if (service_info->request_type_support_) {
      RMW_TRY_DESTRUCTOR(
        service_info->request_type_support_->~DynamicDataTypeSupport(),
        DynamicDataTypeSupport, result = RMW_RET_ERROR)
      rmw_free(service_info->request_type_support_);
    }
    if (service_info->response_type_support_) {
      RMW_TRY_DESTRUCTOR(
        service_info->response_type_support_->~DynamicDataTypeSupport(),
        DynamicDataTypeSupport, result = RMW_RET_ERROR)
      rmw_free(service_info->response_type_support_);
    }
    if (service_info->replier_) {
      RMW_TRY_DESTRUCTOR(
        service_info->replier_->~Replier(),
        "connext::Replier<DDS_DynamicData, DDS_DynamicData>",
        result = RMW_RET_ERROR)
      rmw_free(service_info->replier_);
    }
    if (service->service_name) {
      rmw_free(const_cast<char *>(service->service_name));
    }
  }
  if (service_info) {
    RMW_TRY_DESTRUCTOR(service_info->~ConnextDynamicServiceInfo(),
      ConnextDynamicServiceInfo, result = RMW_RET_ERROR)
    rmw_free(service_info);
  }

  rmw_service_free(service);

  // TODO(wjwwood): if the function returns early some memory leaks will occur.
  return result;
}

rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_request,
  bool * taken)
{
  if (!service) {
    RMW_SET_ERROR_MSG("service handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service handle,
    service->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!request_header) {
    RMW_SET_ERROR_MSG("ros request header handle is null");
    return RMW_RET_ERROR;
  }
  if (!ros_request) {
    RMW_SET_ERROR_MSG("ros request handle is null");
    return RMW_RET_ERROR;
  }
  if (!taken) {
    RMW_SET_ERROR_MSG("taken handle is null");
    return RMW_RET_ERROR;
  }

  ConnextDynamicServiceInfo * service_info =
    static_cast<ConnextDynamicServiceInfo *>(service->data);
  if (!service_info) {
    RMW_SET_ERROR_MSG("service info handle is null");
    return RMW_RET_ERROR;
  }
  connext::Replier<DDS_DynamicData, DDS_DynamicData> * replier = service_info->replier_;
  if (!replier) {
    RMW_SET_ERROR_MSG("replier handle is null");
    return RMW_RET_ERROR;
  }

  connext::LoanedSamples<DDS::DynamicData> requests = replier->take_requests(1);
  if (requests.begin() != requests.end() && requests.begin()->info().valid_data) {
    bool success = _take(
      &requests.begin()->data(), ros_request, service_info->untyped_request_members_,
      service_info->typesupport_identifier);
    if (!success) {
      // error string was set within the function
      return RMW_RET_ERROR;
    }

    size_t SAMPLE_IDENTITY_SIZE = 16;
    memcpy(
      &request_header->writer_guid[0], requests.begin()->identity().writer_guid.value,
      SAMPLE_IDENTITY_SIZE);

    request_header->sequence_number =
      ((int64_t)requests.begin()->identity().sequence_number.high) << 32 |
      requests.begin()->identity().sequence_number.low;
    *taken = true;
  } else {
    *taken = false;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_request_id_t * request_header,
  void * ros_response,
  bool * taken)
{
  if (!client) {
    RMW_SET_ERROR_MSG("client handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client handle,
    client->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!request_header) {
    RMW_SET_ERROR_MSG("ros request header handle is null");
    return RMW_RET_ERROR;
  }
  if (!ros_response) {
    RMW_SET_ERROR_MSG("ros response handle is null");
    return RMW_RET_ERROR;
  }
  if (!taken) {
    RMW_SET_ERROR_MSG("taken handle is null");
    return RMW_RET_ERROR;
  }

  ConnextDynamicClientInfo * client_info =
    static_cast<ConnextDynamicClientInfo *>(client->data);
  if (!client_info) {
    RMW_SET_ERROR_MSG("client info handle is null");
    return RMW_RET_ERROR;
  }
  connext::Requester<DDS_DynamicData, DDS_DynamicData> * requester = client_info->requester_;
  if (!requester) {
    RMW_SET_ERROR_MSG("requester handle is null");
    return RMW_RET_ERROR;
  }

  connext::LoanedSamples<DDS::DynamicData> replies = requester->take_replies(1);
  if (replies.begin() != replies.end() && replies.begin()->info().valid_data) {
    bool success = _take(
      &replies.begin()->data(), ros_response, client_info->untyped_response_members_,
      client_info->typesupport_identifier);
    if (!success) {
      // error string was set within the function
      return RMW_RET_ERROR;
    }

    request_header->sequence_number =
      (((int64_t)replies.begin()->related_identity().sequence_number.high) << 32) |
      replies.begin()->related_identity().sequence_number.low;
    *taken = true;
  } else {
    *taken = false;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  if (!service) {
    RMW_SET_ERROR_MSG("service handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service handle,
    service->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)

  if (!request_header) {
    RMW_SET_ERROR_MSG("ros request header handle is null");
    return RMW_RET_ERROR;
  }
  if (!ros_response) {
    RMW_SET_ERROR_MSG("ros response handle is null");
    return RMW_RET_ERROR;
  }

  ConnextDynamicServiceInfo * service_info =
    static_cast<ConnextDynamicServiceInfo *>(service->data);
  if (!service_info) {
    RMW_SET_ERROR_MSG("service info handle is null");
    return RMW_RET_ERROR;
  }
  connext::Replier<DDS_DynamicData, DDS_DynamicData> * replier = service_info->replier_;
  if (!replier) {
    RMW_SET_ERROR_MSG("replier handle is null");
    return RMW_RET_ERROR;
  }

  DDS::DynamicData * sample = service_info->response_type_support_->create_data();
  if (!sample) {
    RMW_SET_ERROR_MSG("failed to create data");
    return RMW_RET_ERROR;
  }
  DDS::WriteParams_t writeParams;
  connext::WriteSampleRef<DDS::DynamicData> response(*sample, writeParams);

  bool published = _publish(sample, ros_response, service_info->untyped_response_members_,
      service_info->typesupport_identifier);
  if (!published) {
    // error string was set within the function
    if (service_info->response_type_support_->delete_data(sample) != DDS_RETCODE_OK) {
      std::stringstream ss;
      ss << "leaking dynamic data object while handling error at " <<
        __FILE__ << ":" << __LINE__;
      (std::cerr << ss.str()).flush();
    }
    return RMW_RET_ERROR;
  }

  DDS_SampleIdentity_t request_identity;

  size_t SAMPLE_IDENTITY_SIZE = 16;
  memcpy(request_identity.writer_guid.value, &request_header->writer_guid[0], SAMPLE_IDENTITY_SIZE);

  request_identity.sequence_number.high = (int32_t)(
    (request_header->sequence_number & 0xFFFFFFFF00000000) >> 32);
  request_identity.sequence_number.low = (uint32_t)(request_header->sequence_number & 0xFFFFFFFF);

  replier->send_reply(response, request_identity);

  if (service_info->response_type_support_->delete_data(sample) != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to delete dynamic data object");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}


rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  return get_topic_names_and_types(
    rti_connext_dynamic_identifier,
    node,
    allocator,
    no_demangle,
    topic_names_and_types);
}

rmw_ret_t
rmw_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types)
{
  return get_service_names_and_types(
    rti_connext_dynamic_identifier,
    node,
    allocator,
    service_names_and_types);
}

rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names)
{
  return get_node_names(
    rti_connext_dynamic_identifier, node, node_names);
}

rmw_ret_t
rmw_count_publishers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  return count_publishers(rti_connext_dynamic_identifier, node, topic_name, count);
}

rmw_ret_t
rmw_count_subscribers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  return count_subscribers(rti_connext_dynamic_identifier, node, topic_name, count);
}

rmw_ret_t
rmw_get_publishers_info_by_topic(
    const rmw_node_t * node,
    rcutils_allocator_t * allocator,
    const char * topic_name,
    bool no_mangle,
    rmw_topic_endpoint_info_array_t * publishers_info)
{
  return get_publishers_info_by_topic(rti_connext_dynamic_identifier, node, allocator, topic_name, no_mangle, publishers_info);
}

rmw_ret_t
rmw_get_subscriptions_info_by_topic(
    const rmw_node_t * node,
    rcutils_allocator_t * allocator,
    const char * topic_name,
    bool no_mangle,
    rmw_topic_endpoint_info_array_t * subscriptions_info)
{
  return get_subscriptions_info_by_topic(rti_connext_dynamic_identifier, node, allocator, topic_name, no_mangle, subscriptions_info);
}

rmw_ret_t
rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  if (!publisher) {
    RMW_SET_ERROR_MSG("publisher is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)
  if (!gid) {
    RMW_SET_ERROR_MSG("gid is null");
    return RMW_RET_ERROR;
  }

  const CustomPublisherInfo * publisher_info =
    static_cast<const CustomPublisherInfo *>(publisher->data);
  if (!publisher_info) {
    RMW_SET_ERROR_MSG("publisher info handle is null");
    return RMW_RET_ERROR;
  }

  *gid = publisher_info->publisher_gid;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_compare_gids_equal(const rmw_gid_t * gid1, const rmw_gid_t * gid2, bool * result)
{
  if (!gid1) {
    RMW_SET_ERROR_MSG("gid1 is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid1,
    gid1->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)
  if (!gid2) {
    RMW_SET_ERROR_MSG("gid2 is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid2,
    gid2->implementation_identifier,
    rti_connext_dynamic_identifier,
    return RMW_RET_ERROR)
  if (!result) {
    RMW_SET_ERROR_MSG("result is null");
    return RMW_RET_ERROR;
  }
  auto detail1 = reinterpret_cast<const ConnextPublisherGID *>(gid1->data);
  if (!detail1) {
    RMW_SET_ERROR_MSG("gid1 is invalid");
    return RMW_RET_ERROR;
  }
  auto detail2 = reinterpret_cast<const ConnextPublisherGID *>(gid2->data);
  if (!detail2) {
    RMW_SET_ERROR_MSG("gid2 is invalid");
    return RMW_RET_ERROR;
  }
  auto matches =
    DDS_InstanceHandle_equals(&detail1->publication_handle, &detail2->publication_handle);
  *result = (matches == DDS_BOOLEAN_TRUE);
  return RMW_RET_OK;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return nullptr;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return nullptr)

  return node_get_graph_guard_condition(node);
}

rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  // TODO(wjwwood): remove this once local graph changes are detected.
  RMW_SET_ERROR_MSG("not implemented");
  return RMW_RET_ERROR;

  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION)
  if (!client) {
    RMW_SET_ERROR_MSG("client handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client handle,
    client->implementation_identifier, rti_connext_dynamic_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION)

  if (!is_available) {
    RMW_SET_ERROR_MSG("is_available is null");
    return RMW_RET_ERROR;
  }

  ConnextDynamicClientInfo * client_info =
    static_cast<ConnextDynamicClientInfo *>(client->data);
  if (!client_info) {
    RMW_SET_ERROR_MSG("client info handle is null");
    return RMW_RET_ERROR;
  }

  *is_available = false;
  // In the Connext RPC implementation, a server is ready when:
  //   - At least one server is subscribed to the request topic.
  //   - At least one server is publishing to the reponse topic.
  size_t number_of_request_subscribers = 0;
  rmw_ret_t ret = rmw_count_subscribers(
    node,
    client_info->requester_->get_request_datawriter()->get_topic()->get_name(),
    &number_of_request_subscribers);
  if (ret != RMW_RET_OK) {
    // error string already set
    return ret;
  }
#ifdef DISCOVERY_DEBUG_LOGGING
  printf("Checking for service server:\n");
  printf(" - %s: %zu\n",
    client_info->requester_->get_request_datawriter()->get_topic()->get_name(),
    number_of_request_subscribers);
#endif
  if (number_of_request_subscribers == 0) {
    // not ready
    return RMW_RET_OK;
  }

  size_t number_of_response_publishers = 0;
  ret = rmw_count_publishers(
    node,
    client_info->response_datareader_->get_topicdescription()->get_name(),
    &number_of_response_publishers);
  if (ret != RMW_RET_OK) {
    // error string already set
    return ret;
  }
#ifdef DISCOVERY_DEBUG_LOGGING
  printf(" - %s: %zu\n",
    client_info->response_datareader_->get_topicdescription()->get_name(),
    number_of_response_publishers);
#endif
  if (number_of_response_publishers == 0) {
    // not ready
    return RMW_RET_OK;
  }

  // all conditions met, there is a service server available
  *is_available = true;
  return RMW_RET_OK;
}
}  // extern "C"
