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

#include <string>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rmw/validate_full_topic_name.h"

#include "rmw/impl/cpp/macros.hpp"

#include "rmw_connext_shared_cpp/qos.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

#include "rmw_connext_cpp/identifier.hpp"

#include "process_topic_and_service_names.hpp"
#include "type_support_common.hpp"
#include "rmw_connext_cpp/connext_static_publisher_info.hpp"

// include patched generated code from the build folder
#include "connext_static_serialized_dataSupport.h"

// Uncomment this to get extra console output about discovery.
// This affects code in this file, but there is a similar variable in:
//   rmw_connext_shared_cpp/shared_functions.cpp
// #define DISCOVERY_DEBUG_LOGGING 1

extern "C"
{
rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) type_support;
  (void) message_bounds;
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier,
    rti_connext_identifier,
    return nullptr);
  RMW_CONNEXT_EXTRACT_MESSAGE_TYPESUPPORT(type_supports, type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (0 == strlen(topic_name)) {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  if (!qos_profile->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  const message_type_support_callbacks_t * callbacks =
    static_cast<const message_type_support_callbacks_t *>(type_support->data);

  std::string type_name = _create_type_name(callbacks);
  // Past this point, a failure results in unrolling code in the goto fail block.
  DDS::TypeCode * type_code = nullptr;
  DDS::DataWriterQos datawriter_qos;
  DDS::PublisherQos publisher_qos;
  DDS::ReturnCode_t status;
  DDS::Publisher * dds_publisher = nullptr;
  DDS::DataWriter * topic_writer = nullptr;
  DDS::Topic * topic = nullptr;
  DDS::TopicDescription * topic_description = nullptr;
  void * info_buf = nullptr;
  void * listener_buf = nullptr;
  ConnextPublisherListener * publisher_listener = nullptr;
  ConnextStaticPublisherInfo * publisher_info = nullptr;
  rmw_publisher_t * publisher = nullptr;
  std::string mangled_name = "";
  rmw_qos_profile_t actual_qos_profile;

  char * topic_str = nullptr;

  // Begin initializing elements
  publisher = rmw_publisher_allocate();
  if (!publisher) {
    RMW_SET_ERROR_MSG("failed to allocate publisher");
    goto fail;
  }
  publisher->can_loan_messages = false;

  type_code = callbacks->get_type_code();
  if (!type_code) {
    RMW_SET_ERROR_MSG("failed to fetch type code\n");
    goto fail;
  }
  // This is a non-standard RTI Connext function
  // It allows to register an external type to a static data writer
  // In this case, we register the custom message type to a data writer,
  // which only publishes DDS_Octets
  // The purpose of this is to send only raw data DDS_Octets over the wire,
  // advertise the topic however with a type of the message, e.g. std_msgs::msg::dds_::String
  status = ConnextStaticSerializedDataSupport_register_external_type(
    participant, type_name.c_str(), type_code);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to register external type");
    goto fail;
  }

  status = participant->get_default_publisher_qos(publisher_qos);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default publisher qos");
    goto fail;
  }

  // allocating memory for topic_str
  if (!_process_topic_name(
      topic_name,
      qos_profile->avoid_ros_namespace_conventions,
      &topic_str))
  {
    goto fail;
  }

  // Allocate memory for the PublisherListener object.
  listener_buf = rmw_allocate(sizeof(ConnextPublisherListener));
  if (!listener_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory for publisher listener");
    goto fail;
  }
  // Use a placement new to construct the PublisherListener in the preallocated buffer.
  // cppcheck-suppress syntaxError
  RMW_TRY_PLACEMENT_NEW(publisher_listener, listener_buf, goto fail, ConnextPublisherListener, )
  listener_buf = nullptr;  // Only free the buffer pointer.

  dds_publisher = participant->create_publisher(
    publisher_qos, publisher_listener, DDS::PUBLICATION_MATCHED_STATUS);
  if (!dds_publisher) {
    RMW_SET_ERROR_MSG("failed to create publisher");
    goto fail;
  }

  topic_description = participant->lookup_topicdescription(topic_str);
  if (!topic_description) {
    DDS::TopicQos default_topic_qos;
    status = participant->get_default_topic_qos(default_topic_qos);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to get default topic qos");
      goto fail;
    }

    topic = participant->create_topic(
      topic_str, type_name.c_str(),
      default_topic_qos, NULL, DDS::STATUS_MASK_NONE);
    if (!topic) {
      RMW_SET_ERROR_MSG("failed to create topic");
      goto fail;
    }
  } else {
    DDS::Duration_t timeout = DDS::Duration_t::from_seconds(0);
    topic = participant->find_topic(topic_str, timeout);
    if (!topic) {
      RMW_SET_ERROR_MSG("failed to find topic");
      goto fail;
    }
  }
  DDS::String_free(topic_str);
  topic_str = nullptr;

  if (!get_datawriter_qos(participant, *qos_profile, datawriter_qos)) {
    // error string was set within the function
    goto fail;
  }

  topic_writer = dds_publisher->create_datawriter(
    topic, datawriter_qos, NULL, DDS::STATUS_MASK_NONE);
  if (!topic_writer) {
    RMW_SET_ERROR_MSG("failed to create datawriter");
    goto fail;
  }

  // Allocate memory for the ConnextStaticPublisherInfo object.
  info_buf = rmw_allocate(sizeof(ConnextStaticPublisherInfo));
  if (!info_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory for publisher info");
    goto fail;
  }
  // Use a placement new to construct the ConnextStaticPublisherInfo in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(publisher_info, info_buf, goto fail, ConnextStaticPublisherInfo, )
  info_buf = nullptr;  // Only free the publisher_info pointer; don't need the buf pointer anymore.
  publisher_info->dds_publisher_ = dds_publisher;
  publisher_info->topic_writer_ = topic_writer;
  publisher_info->callbacks_ = callbacks;
  publisher_info->publisher_gid.implementation_identifier = rti_connext_identifier;
  publisher_info->listener_ = publisher_listener;
  publisher_listener = nullptr;
  static_assert(
    sizeof(ConnextPublisherGID) <= RMW_GID_STORAGE_SIZE,
    "RMW_GID_STORAGE_SIZE insufficient to store the rmw_connext_cpp GID implemenation."
  );
  // Zero the data memory.
  memset(publisher_info->publisher_gid.data, 0, RMW_GID_STORAGE_SIZE);
  {
    auto publisher_gid =
      reinterpret_cast<ConnextPublisherGID *>(publisher_info->publisher_gid.data);
    publisher_gid->publication_handle = topic_writer->get_instance_handle();
  }
  publisher_info->publisher_gid.implementation_identifier = rti_connext_identifier;

  publisher->implementation_identifier = rti_connext_identifier;
  publisher->data = publisher_info;
  publisher->topic_name = reinterpret_cast<const char *>(rmw_allocate(strlen(topic_name) + 1));
  if (!publisher->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for topic name");
    goto fail;
  }
  memcpy(const_cast<char *>(publisher->topic_name), topic_name, strlen(topic_name) + 1);
  publisher->options = *publisher_options;

  if (!qos_profile->avoid_ros_namespace_conventions) {
    mangled_name = topic_writer->get_topic()->get_name();
  } else {
    mangled_name = topic_name;
  }
  status = topic_writer->get_qos(datawriter_qos);
  if (DDS::RETCODE_OK != status) {
    RMW_SET_ERROR_MSG("topic_writer can't get data reader qos policies");
    goto fail;
  }
  dds_qos_to_rmw_qos(datawriter_qos, &actual_qos_profile);
  node_info->publisher_listener->add_information(
    node_info->participant->get_instance_handle(),
    dds_publisher->get_instance_handle(),
    mangled_name,
    type_name,
    actual_qos_profile,
    EntityType::Publisher);
  node_info->publisher_listener->trigger_graph_guard_condition();

// TODO(karsten1987): replace this block with logging macros
#ifdef DISCOVERY_DEBUG_LOGGING
  fprintf(stderr, "******* Creating Publisher Details: ********\n");
  fprintf(stderr, "Publisher topic %s\n", topic_writer->get_topic()->get_name());
  fprintf(stderr, "Publisher address %p\n", static_cast<void *>(dds_publisher));
  fprintf(stderr, "******\n");
#endif

  return publisher;
fail:
  if (topic_str) {
    DDS::String_free(topic_str);
    topic_str = nullptr;
  }
  if (publisher) {
    rmw_publisher_free(publisher);
  }
  if (dds_publisher) {
    if (topic_writer) {
      if (dds_publisher->delete_datawriter(topic_writer) != DDS::RETCODE_OK) {
        std::stringstream ss;
        ss << "leaking datawriter while handling failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
    if (participant->delete_publisher(dds_publisher) != DDS::RETCODE_OK) {
      std::stringstream ss;
      ss << "leaking publisher while handling failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (publisher_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      publisher_listener->~ConnextPublisherListener(), ConnextPublisherListener)
    rmw_free(publisher_listener);
  }
  if (publisher_info) {
    if (publisher_info->listener_) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        publisher_info->listener_->~ConnextPublisherListener(), ConnextPublisherListener)
      rmw_free(publisher_info->listener_);
    }
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      publisher_info->~ConnextStaticPublisherInfo(), ConnextStaticPublisherInfo)
    rmw_free(publisher_info);
  }
  if (info_buf) {
    rmw_free(info_buf);
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
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rti_connext_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<ConnextStaticPublisherInfo *>(publisher->data);

  *subscription_count = info->listener_->current_count();

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier,
    rti_connext_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<ConnextStaticPublisherInfo *>(publisher->data);
  DDS::DataWriter * data_writer = info->topic_writer_;

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

  auto info = static_cast<ConnextStaticPublisherInfo *>(publisher->data);
  if (nullptr == info) {
    RMW_SET_ERROR_MSG("publisher internal data is invalid");
    return RMW_RET_ERROR;
  }
  if (nullptr == info->topic_writer_) {
    RMW_SET_ERROR_MSG("publisher internal datawriter is invalid");
    return RMW_RET_ERROR;
  }

  if (info->topic_writer_->assert_liveliness() != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to assert liveliness of datawriter");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
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

  RMW_SET_ERROR_MSG("rmw_borrow_loaned_message not implemented for rmw_connext_cpp");
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
    "rmw_return_loaned_message_from_publisher not implemented for rmw_connext_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier,
    rti_connext_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier,
    rti_connext_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_ret_t ret = RMW_RET_OK;
  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  // TODO(wjwwood): need to figure out when to unregister types with the participant.
  ConnextStaticPublisherInfo * publisher_info =
    static_cast<ConnextStaticPublisherInfo *>(publisher->data);
  node_info->publisher_listener->remove_information(
    publisher_info->dds_publisher_->get_instance_handle(), EntityType::Publisher);
  node_info->publisher_listener->trigger_graph_guard_condition();
  DDS::Publisher * dds_publisher = publisher_info->dds_publisher_;

  if (dds_publisher->delete_datawriter(publisher_info->topic_writer_) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to delete datawriter");
    ret = RMW_RET_ERROR;
  }
  if (participant->delete_publisher(dds_publisher) != DDS::RETCODE_OK) {
    if (RMW_RET_OK == ret) {
      RMW_SET_ERROR_MSG("failed to delete publisher");
      ret = RMW_RET_ERROR;
    } else {
      RMW_SAFE_FWRITE_TO_STDERR("failed to delete publisher\n");
    }
  }

  ConnextPublisherListener * pub_listener = publisher_info->listener_;
  if (RMW_RET_OK == ret) {
    RMW_TRY_DESTRUCTOR(
      pub_listener->~ConnextPublisherListener(),
      ConnextPublisherListener, ret = RMW_RET_ERROR);
  } else {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      pub_listener->~ConnextPublisherListener(),
      ConnextPublisherListener);
  }
  rmw_free(pub_listener);

  if (RMW_RET_OK == ret) {
    RMW_TRY_DESTRUCTOR(
      publisher_info->~ConnextStaticPublisherInfo(),
      ConnextStaticPublisherInfo, ret = RMW_RET_ERROR);
  } else {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      publisher_info->~ConnextStaticPublisherInfo(),
      ConnextStaticPublisherInfo);
  }
  rmw_free(publisher_info);
  rmw_free(const_cast<char *>(publisher->topic_name));
  rmw_publisher_free(publisher);

  return ret;
}
}  // extern "C"
