// Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

#ifdef __clang__
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wmismatched-tags"
#endif
#if defined(_MSC_VER)
# pragma warning(push)
# pragma warning(disable: 4099)
#endif
#include <ccpp_dds_dcps.h>
#if defined(_MSC_VER)
# pragma warning(pop)
#endif
#ifdef __clang__
# pragma GCC diagnostic pop
#endif
#include <dds_dcps.h>
#include <string>

#include "rosidl_typesupport_opensplice_c/identifier.h"
#include "rosidl_typesupport_opensplice_cpp/identifier.hpp"
#include "rosidl_typesupport_opensplice_cpp/misc.hpp"
#include "rosidl_typesupport_opensplice_cpp/impl/error_checking.hpp"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"

#include "event_converter.hpp"
#include "identifier.hpp"
#include "qos.hpp"
#include "types.hpp"
#include "typesupport_macros.hpp"

using rosidl_typesupport_opensplice_cpp::impl::check_get_default_publisher_qos;
using rosidl_typesupport_opensplice_cpp::impl::check_get_default_topic_qos;
using rosidl_typesupport_opensplice_cpp::impl::check_delete_datawriter;
using rosidl_typesupport_opensplice_cpp::impl::check_delete_publisher;
using rosidl_typesupport_opensplice_cpp::impl::check_delete_topic;
using rosidl_typesupport_opensplice_cpp::process_topic_name;

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_message_bounds_t * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) type_support;
  (void) message_bounds;
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_ERROR;
}

rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return nullptr;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, opensplice_cpp_identifier,
    return nullptr)

  RMW_OPENSPLICE_EXTRACT_MESSAGE_TYPESUPPORT(
    type_supports, type_support)

  if (!topic_name || strlen(topic_name) == 0) {
    RMW_SET_ERROR_MSG("publisher topic is null or empty string");
    return nullptr;
  }

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  if (!publisher_options) {
    RMW_SET_ERROR_MSG("publisher_options is null");
    return nullptr;
  }

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return NULL;
  }

  const message_type_support_callbacks_t * callbacks =
    static_cast<const message_type_support_callbacks_t *>(type_support->data);
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return NULL;
  }
  std::string type_name = create_type_name(callbacks);

  const char * error_string = callbacks->register_type(participant, type_name.c_str());
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to register the type: ") + error_string).c_str());
    return nullptr;
  }

  DDS::PublisherQos publisher_qos;
  DDS::ReturnCode_t status;
  status = participant->get_default_publisher_qos(publisher_qos);
  if (nullptr != check_get_default_publisher_qos(status)) {
    RMW_SET_ERROR_MSG(check_get_default_publisher_qos(status));
    return nullptr;
  }
  // Past this point, a failure results in unrolling code in the goto fail block.
  rmw_publisher_t * publisher = nullptr;
  DDS::Publisher * dds_publisher = nullptr;
  DDS::TopicQos default_topic_qos;
  DDS::Topic * topic = nullptr;
  DDS::DataWriterQos datawriter_qos;
  DDS::DataWriter * topic_writer = nullptr;
  void * info_buf = nullptr;
  void * listener_buf = nullptr;
  OpenSplicePublisherListener * publisher_listener = nullptr;
  OpenSpliceStaticPublisherInfo * publisher_info = nullptr;
  std::string topic_str;

  // Begin initializing elements.
  publisher = rmw_publisher_allocate();
  if (!publisher) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_publisher_t");
    goto fail;
  }

  if (!process_topic_name(
      topic_name, qos_profile->avoid_ros_namespace_conventions, topic_str))
  {
    RMW_SET_ERROR_MSG("failed to process topic name");
    goto fail;
  }

  listener_buf = rmw_allocate(sizeof(OpenSplicePublisherListener));
  if (!listener_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory for publisher listener");
    goto fail;
  }
  // Use a placement new to construct the PublisherListener in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(publisher_listener, listener_buf, goto fail, OpenSplicePublisherListener, )
  listener_buf = nullptr;  // Only free the buffer pointer.

  dds_publisher = participant->create_publisher(publisher_qos, publisher_listener,
      DDS::PUBLICATION_MATCHED_STATUS);
  if (!dds_publisher) {
    RMW_SET_ERROR_MSG("failed to create publisher");
    goto fail;
  }

  status = participant->get_default_topic_qos(default_topic_qos);
  if (nullptr != check_get_default_topic_qos(status)) {
    RMW_SET_ERROR_MSG(check_get_default_topic_qos(status));
    goto fail;
  }

  topic = participant->create_topic(
    topic_str.c_str(), type_name.c_str(), default_topic_qos, NULL, DDS::STATUS_MASK_NONE);
  if (!topic) {
    RMW_SET_ERROR_MSG("failed to create topic");
    goto fail;
  }

  if (!get_datawriter_qos(dds_publisher, *qos_profile, datawriter_qos)) {
    goto fail;
  }

  topic_writer = dds_publisher->create_datawriter(
    topic, datawriter_qos, NULL, DDS::STATUS_MASK_NONE);
  if (!topic_writer) {
    RMW_SET_ERROR_MSG("failed to create datawriter");
    goto fail;
  }

  info_buf = static_cast<OpenSpliceStaticPublisherInfo *>(
    rmw_allocate(sizeof(OpenSpliceStaticPublisherInfo)));

  if (!info_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }

  RMW_TRY_PLACEMENT_NEW(publisher_info, info_buf, goto fail, OpenSpliceStaticPublisherInfo, )
  publisher_info->dds_topic = topic;
  publisher_info->dds_publisher = dds_publisher;
  publisher_info->topic_writer = topic_writer;
  publisher_info->callbacks = callbacks;
  publisher_info->listener = publisher_listener;
  publisher_listener = nullptr;
  static_assert(
    sizeof(OpenSplicePublisherGID) <= RMW_GID_STORAGE_SIZE,
    "RMW_GID_STORAGE_SIZE insufficient to store the rmw_opensplice_cpp GID implemenation."
  );
  // Zero the data memory.
  memset(publisher_info->publisher_gid.data, 0, RMW_GID_STORAGE_SIZE);
  {
    auto publisher_gid =
      reinterpret_cast<OpenSplicePublisherGID *>(publisher_info->publisher_gid.data);
    publisher_gid->publication_handle = topic_writer->get_instance_handle();
  }
  publisher_info->publisher_gid.implementation_identifier = opensplice_cpp_identifier;

  publisher->implementation_identifier = opensplice_cpp_identifier;
  publisher->data = publisher_info;
  publisher->topic_name = reinterpret_cast<const char *>(rmw_allocate(strlen(topic_name) + 1));
  if (!publisher->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(publisher->topic_name), topic_name, strlen(topic_name) + 1);
  publisher->options = *publisher_options;

  publisher->can_loan_messages = false;
  return publisher;
fail:
  if (publisher) {
    rmw_publisher_free(publisher);
  }
  if (dds_publisher) {
    if (topic_writer) {
      status = dds_publisher->delete_datawriter(topic_writer);
      if (nullptr != check_delete_datawriter(status)) {
        fprintf(stderr, "%s\n", check_delete_datawriter(status));
      }
    }
    status = participant->delete_publisher(dds_publisher);
    if (nullptr != check_delete_publisher(status)) {
      fprintf(stderr, "%s\n", check_delete_publisher(status));
    }
  }
  if (topic) {
    status = participant->delete_topic(topic);
    if (nullptr != check_delete_topic(status)) {
      fprintf(stderr, "%s\n", check_delete_topic(status));
    }
  }
  if (publisher_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      publisher_listener->~OpenSplicePublisherListener(), OpenSplicePublisherListener)
    rmw_free(publisher_listener);
  }
  if (publisher_info) {
    if (publisher_info->listener) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        publisher_info->listener->~OpenSplicePublisherListener(), OpenSplicePublisherListener)
      rmw_free(publisher_info->listener);
      publisher_info->listener = nullptr;
    }
    rmw_free(publisher_info);
  }
  if (listener_buf) {
    rmw_free(listener_buf);
  }

  return nullptr;
}

rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  if (!publisher) {
    RMW_SET_ERROR_MSG("publisher handle is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (!subscription_count) {
    RMW_SET_ERROR_MSG("subscription_count is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto info = static_cast<OpenSpliceStaticPublisherInfo *>(publisher->data);
  if (!info) {
    RMW_SET_ERROR_MSG("publisher internal data is invalid");
    return RMW_RET_ERROR;
  }
  if (!info->listener) {
    RMW_SET_ERROR_MSG("publisher internal listener is invalid");
    return RMW_RET_ERROR;
  }

  *subscription_count = info->listener->current_count();

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<OpenSpliceStaticPublisherInfo *>(publisher->data);
  if (!info) {
    RMW_SET_ERROR_MSG("publisher internal data is invalid");
    return RMW_RET_ERROR;
  }
  DDS::DataWriter * data_writer = info->topic_writer;
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

  dds_qos_to_rmw_qos(dds_qos, qos);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<OpenSpliceStaticPublisherInfo *>(publisher->data);
  if (nullptr == info) {
    RMW_SET_ERROR_MSG("publisher internal data is invalid");
    return RMW_RET_ERROR;
  }
  if (nullptr == info->topic_writer) {
    RMW_SET_ERROR_MSG("publisher internal datawriter is invalid");
    return RMW_RET_ERROR;
  }

  rmw_ret_t result = check_dds_ret_code(info->topic_writer->assert_liveliness());
  if (RMW_RET_OK != result) {
    RMW_SET_ERROR_MSG("failed to assert liveliness of datawriter");
  }
  return result;
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

  RMW_SET_ERROR_MSG("rmw_borrow_loaned_message not implemented for rmw_opensplice_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  (void) publisher;
  (void) loaned_message;

  RMW_SET_ERROR_MSG(
    "rmw_return_loaned_message_from_publisher not implemented for rmw_opensplice_cpp");
  return RMW_RET_UNSUPPORTED;
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
    node->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  if (!publisher) {
    RMW_SET_ERROR_MSG("pointer handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return RMW_RET_ERROR;
  }
  auto result = RMW_RET_OK;
  OpenSpliceStaticPublisherInfo * publisher_info =
    static_cast<OpenSpliceStaticPublisherInfo *>(publisher->data);
  if (publisher_info) {
    DDS::Publisher * dds_publisher = publisher_info->dds_publisher;
    if (dds_publisher) {
      DDS::DataWriter * topic_writer = publisher_info->topic_writer;
      if (topic_writer) {
        DDS::ReturnCode_t status = dds_publisher->delete_datawriter(topic_writer);
        if (nullptr != check_delete_datawriter(status)) {
          RMW_SET_ERROR_MSG(check_delete_datawriter(status));
          result = RMW_RET_ERROR;
        }
      }
      DDS::ReturnCode_t status = participant->delete_publisher(dds_publisher);
      if (nullptr != check_delete_publisher(status)) {
        RMW_SET_ERROR_MSG(check_delete_publisher(status));
        result = RMW_RET_ERROR;
      }
    }
    DDS::Topic * topic = publisher_info->dds_topic;
    if (topic) {
      DDS::ReturnCode_t status = participant->delete_topic(topic);
      if (nullptr != check_delete_topic(status)) {
        fprintf(stderr, "%s\n", check_delete_topic(status));
        result = RMW_RET_ERROR;
      }
    }

    if (publisher_info->listener) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        publisher_info->listener->~OpenSplicePublisherListener(), OpenSplicePublisherListener)
      rmw_free(publisher_info->listener);
      publisher_info->listener = nullptr;
    }

    rmw_free(publisher_info);
  }
  if (publisher->topic_name) {
    rmw_free(const_cast<char *>(publisher->topic_name));
  }
  rmw_publisher_free(publisher);
  return result;
}
}  // extern "C"
