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

#include "identifier.hpp"
#include "qos.hpp"
#include "types.hpp"
#include "typesupport_macros.hpp"

using rosidl_typesupport_opensplice_cpp::impl::check_get_default_datareader_qos;
using rosidl_typesupport_opensplice_cpp::impl::check_get_default_topic_qos;
using rosidl_typesupport_opensplice_cpp::impl::check_delete_datareader;
using rosidl_typesupport_opensplice_cpp::impl::check_delete_subscriber;
using rosidl_typesupport_opensplice_cpp::impl::check_delete_topic;
using rosidl_typesupport_opensplice_cpp::process_topic_name;

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_message_bounds_t * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) type_support;
  (void) message_bounds;
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_fini_subscription_allocation(rmw_subscription_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_ERROR;
}

rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return nullptr;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, opensplice_cpp_identifier,
    return NULL)

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

  if (!subscription_options) {
    RMW_SET_ERROR_MSG("subscription_options is null");
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

  DDS::SubscriberQos subscriber_qos;
  DDS::ReturnCode_t status = participant->get_default_subscriber_qos(subscriber_qos);
  if (nullptr != check_get_default_datareader_qos(status)) {
    RMW_SET_ERROR_MSG(check_get_default_datareader_qos(status));
    return nullptr;
  }

  // Past this point, a failure results in unrolling code in the goto fail block.
  rmw_subscription_t * subscription = nullptr;
  DDS::Subscriber * dds_subscriber = nullptr;
  DDS::TopicQos default_topic_qos;
  DDS::Topic * topic = nullptr;
  DDS::DataReaderQos datareader_qos;
  DDS::DataReader * topic_reader = nullptr;
  DDS::ReadCondition * read_condition = nullptr;
  void * info_buf = nullptr;
  void * listener_buf = nullptr;
  OpenSpliceStaticSubscriberInfo * subscriber_info = nullptr;
  OpenSpliceSubscriberListener * subscriber_listener = nullptr;
  std::string topic_str;

  // Begin initializing elements.
  subscription = rmw_subscription_allocate();
  if (!subscription) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_subscription_t");
    goto fail;
  }
  if (!process_topic_name(
      topic_name, qos_profile->avoid_ros_namespace_conventions, topic_str))
  {
    RMW_SET_ERROR_MSG("failed to process topic name");
    goto fail;
  }

  listener_buf = rmw_allocate(sizeof(OpenSpliceSubscriberListener));
  if (!listener_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory for subscriber listener");
    goto fail;
  }
  // Use a placement new to construct the ConnextSubscriberListener in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(subscriber_listener, listener_buf, goto fail, OpenSpliceSubscriberListener,
  )
  listener_buf = nullptr;  // Only free the buffer pointer.

  dds_subscriber = participant->create_subscriber(subscriber_qos, subscriber_listener,
      DDS::SUBSCRIPTION_MATCHED_STATUS);
  if (!dds_subscriber) {
    RMW_SET_ERROR_MSG("failed to create subscriber");
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

  if (!get_datareader_qos(dds_subscriber, *qos_profile, datareader_qos)) {
    goto fail;
  }

  topic_reader = dds_subscriber->create_datareader(
    topic, datareader_qos, NULL, DDS::STATUS_MASK_NONE);
  if (!topic_reader) {
    RMW_SET_ERROR_MSG("failed to create topic reader");
    goto fail;
  }

  read_condition = topic_reader->create_readcondition(
    DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  // Allocate memory for the OpenSpliceStaticSubscriberInfo object.
  info_buf = rmw_allocate(sizeof(OpenSpliceStaticSubscriberInfo));
  if (!info_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the instance in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(subscriber_info, info_buf, goto fail, OpenSpliceStaticSubscriberInfo, )
  info_buf = nullptr;  // Only free the subscriber_info pointer; don't need the buf pointer anymore.
  subscriber_info->dds_topic = topic;
  subscriber_info->dds_subscriber = dds_subscriber;
  subscriber_info->topic_reader = topic_reader;
  subscriber_info->read_condition = read_condition;
  subscriber_info->callbacks = callbacks;
  subscriber_info->listener = subscriber_listener;
  subscriber_listener = nullptr;

  subscription->implementation_identifier = opensplice_cpp_identifier;
  subscription->data = subscriber_info;

  subscription->topic_name = reinterpret_cast<const char *>(rmw_allocate(strlen(topic_name) + 1));
  if (!subscription->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(subscription->topic_name), topic_name, strlen(topic_name) + 1);
  subscription->options = *subscription_options;

  subscription->can_loan_messages = false;
  return subscription;
fail:
  if (dds_subscriber) {
    if (topic_reader) {
      if (read_condition) {
        if (topic_reader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
          fprintf(stderr, "leaking readcondition while handling failure\n");
        }
      }
      status = dds_subscriber->delete_datareader(topic_reader);
      if (nullptr != check_delete_datareader(status)) {
        fprintf(stderr, "%s\n", check_delete_datareader(status));
      }
      status = participant->delete_subscriber(dds_subscriber);
      if (nullptr != check_delete_subscriber(status)) {
        fprintf(stderr, "%s\n", check_delete_subscriber(status));
      }
    }
  }
  if (subscriber_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      subscriber_listener->~OpenSpliceSubscriberListener(), OpenSpliceSubscriberListener)
    rmw_free(subscriber_listener);
  }
  if (topic) {
    status = participant->delete_topic(topic);
    if (nullptr != check_delete_topic(status)) {
      fprintf(stderr, "%s\n", check_delete_topic(status));
    }
  }
  if (subscriber_info) {
    if (subscriber_info->listener) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        subscriber_info->listener->~OpenSpliceSubscriberListener(), OpenSpliceSubscriberListener)
      rmw_free(subscriber_info->listener);
      subscriber_info->listener = nullptr;
    }
    rmw_free(subscriber_info);
  }
  if (subscription) {
    rmw_subscription_free(subscription);
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
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription handle is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (!publisher_count) {
    RMW_SET_ERROR_MSG("publisher_count is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto info = static_cast<OpenSpliceStaticSubscriberInfo *>(subscription->data);
  if (!info) {
    RMW_SET_ERROR_MSG("subscriber internal data is invalid");
    return RMW_RET_ERROR;
  }
  if (!info->listener) {
    RMW_SET_ERROR_MSG("subscriber internal listener is invalid");
    return RMW_RET_ERROR;
  }

  *publisher_count = info->listener->current_count();

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<OpenSpliceStaticSubscriberInfo *>(subscription->data);
  if (!info) {
    RMW_SET_ERROR_MSG("subscription internal data is invalid");
    return RMW_RET_ERROR;
  }
  DDS::DataReader * data_reader = info->topic_reader;
  if (!data_reader) {
    RMW_SET_ERROR_MSG("subscription internal data reader is invalid");
    return RMW_RET_ERROR;
  }
  DDS::DataReaderQos dds_qos;
  DDS::ReturnCode_t status = data_reader->get_qos(dds_qos);
  if (DDS::RETCODE_OK != status) {
    RMW_SET_ERROR_MSG("subscription can't get data reader qos policies");
    return RMW_RET_ERROR;
  }

  dds_qos_to_rmw_qos(dds_qos, qos);

  return RMW_RET_OK;
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
    node->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, opensplice_cpp_identifier,
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
  OpenSpliceStaticSubscriberInfo * subscription_info =
    static_cast<OpenSpliceStaticSubscriberInfo *>(subscription->data);
  if (subscription_info) {
    DDS::Subscriber * dds_subscriber = subscription_info->dds_subscriber;
    if (dds_subscriber) {
      DDS::DataReader * topic_reader = subscription_info->topic_reader;
      if (topic_reader) {
        DDS::ReadCondition * read_condition = subscription_info->read_condition;
        if (read_condition) {
          if (topic_reader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
            RMW_SET_ERROR_MSG("failed to delete readcondition");
            result = RMW_RET_ERROR;
          }
          subscription_info->read_condition = nullptr;
        }
        if (topic_reader->delete_contained_entities() != DDS::RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete contained entities of datareader");
          result = RMW_RET_ERROR;
        }
        DDS::ReturnCode_t status = dds_subscriber->delete_datareader(topic_reader);
        if (nullptr != check_delete_datareader(status)) {
          RMW_SET_ERROR_MSG(check_delete_datareader(status));
          result = RMW_RET_ERROR;
        }
      }

      DDS::ReturnCode_t status = participant->delete_subscriber(dds_subscriber);
      if (nullptr != check_delete_subscriber(status)) {
        RMW_SET_ERROR_MSG(check_delete_subscriber(status));
        result = RMW_RET_ERROR;
      }
    }
    DDS::Topic * topic = subscription_info->dds_topic;
    if (topic) {
      DDS::ReturnCode_t status = participant->delete_topic(topic);
      if (nullptr != check_delete_topic(status)) {
        fprintf(stderr, "%s\n", check_delete_topic(status));
        result = RMW_RET_ERROR;
      }
    }

    if (subscription_info->listener) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        subscription_info->listener->~OpenSpliceSubscriberListener(), OpenSpliceSubscriberListener)
      rmw_free(subscription_info->listener);
      subscription_info->listener = nullptr;
    }

    rmw_free(subscription_info);
  }
  if (subscription->topic_name) {
    rmw_free(const_cast<char *>(subscription->topic_name));
  }
  rmw_subscription_free(subscription);
  return result;
}
}  // extern "C"
