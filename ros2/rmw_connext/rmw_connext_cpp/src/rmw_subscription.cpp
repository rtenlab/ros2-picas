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
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#include "rmw_connext_shared_cpp/qos.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

#include "rmw_connext_cpp/identifier.hpp"

#include "process_topic_and_service_names.hpp"
#include "type_support_common.hpp"
#include "rmw_connext_cpp/connext_static_subscriber_info.hpp"

// include patched generated code from the build folder
#include "connext_static_serialized_dataSupport.h"

// Uncomment this to get extra console output about discovery.
// This affects code in this file, but there is a similar variable in:
//   rmw_connext_shared_cpp/shared_functions.cpp
// #define DISCOVERY_DEBUG_LOGGING 1

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
    return NULL;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_identifier,
    return NULL)

  RMW_CONNEXT_EXTRACT_MESSAGE_TYPESUPPORT(type_supports, type_support, NULL)

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  if (!subscription_options) {
    RMW_SET_ERROR_MSG("subscription_options is null");
    return nullptr;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
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
  std::string type_name = _create_type_name(callbacks);
  // Past this point, a failure results in unrolling code in the goto fail block.
  DDS::TypeCode * type_code = nullptr;
  DDS::DataReaderQos datareader_qos;
  DDS::SubscriberQos subscriber_qos;
  DDS::ReturnCode_t status;
  DDS::Subscriber * dds_subscriber = nullptr;
  DDS::Topic * topic = nullptr;
  DDS::TopicDescription * topic_description = nullptr;
  DDS::DataReader * topic_reader = nullptr;
  DDS::ReadCondition * read_condition = nullptr;
  void * info_buf = nullptr;
  void * listener_buf = nullptr;
  ConnextSubscriberListener * subscriber_listener = nullptr;
  ConnextStaticSubscriberInfo * subscriber_info = nullptr;
  rmw_subscription_t * subscription = nullptr;
  std::string mangled_name;

  char * topic_str = nullptr;

  // Begin initializing elements.
  subscription = rmw_subscription_allocate();
  if (!subscription) {
    RMW_SET_ERROR_MSG("failed to allocate subscription");
    goto fail;
  }

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

  status = participant->get_default_subscriber_qos(subscriber_qos);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default subscriber qos");
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

  // Allocate memory for the SubscriberListener object.
  listener_buf = rmw_allocate(sizeof(ConnextSubscriberListener));
  if (!listener_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory for subscriber listener");
    goto fail;
  }
  // Use a placement new to construct the ConnextSubscriberListener in the preallocated buffer.
  // cppcheck-suppress syntaxError
  RMW_TRY_PLACEMENT_NEW(subscriber_listener, listener_buf, goto fail, ConnextSubscriberListener, )
  listener_buf = nullptr;  // Only free the buffer pointer.

  dds_subscriber = participant->create_subscriber(
    subscriber_qos, subscriber_listener, DDS::SUBSCRIPTION_MATCHED_STATUS);
  if (!dds_subscriber) {
    RMW_SET_ERROR_MSG("failed to create subscriber");
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

  if (!get_datareader_qos(participant, *qos_profile, datareader_qos)) {
    // error string was set within the function
    goto fail;
  }

  topic_reader = dds_subscriber->create_datareader(
    topic, datareader_qos,
    NULL, DDS::STATUS_MASK_NONE);
  if (!topic_reader) {
    RMW_SET_ERROR_MSG("failed to create datareader");
    goto fail;
  }

  read_condition = topic_reader->create_readcondition(
    DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  // Allocate memory for the ConnextStaticSubscriberInfo object.
  info_buf = rmw_allocate(sizeof(ConnextStaticSubscriberInfo));
  if (!info_buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the ConnextStaticSubscriberInfo in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(subscriber_info, info_buf, goto fail, ConnextStaticSubscriberInfo, )
  info_buf = nullptr;  // Only free the subscriber_info pointer; don't need the buf pointer anymore.
  subscriber_info->dds_subscriber_ = dds_subscriber;
  subscriber_info->topic_reader_ = topic_reader;
  subscriber_info->read_condition_ = read_condition;
  subscriber_info->callbacks_ = callbacks;
  subscriber_info->listener_ = subscriber_listener;
  subscriber_listener = nullptr;

  subscription->implementation_identifier = rti_connext_identifier;
  subscription->data = subscriber_info;

  subscription->topic_name = reinterpret_cast<const char *>(
    rmw_allocate(strlen(topic_name) + 1));
  if (!subscription->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(subscription->topic_name), topic_name, strlen(topic_name) + 1);

  subscription->options = *subscription_options;

  if (!qos_profile->avoid_ros_namespace_conventions) {
    mangled_name =
      topic_reader->get_topicdescription()->get_name();
  } else {
    mangled_name = topic_name;
  }
  node_info->subscriber_listener->add_information(
    node_info->participant->get_instance_handle(),
    dds_subscriber->get_instance_handle(),
    mangled_name,
    type_name,
    EntityType::Subscriber);
  node_info->subscriber_listener->trigger_graph_guard_condition();

// TODO(karsten1987): replace this block with logging macros
#ifdef DISCOVERY_DEBUG_LOGGING
  fprintf(stderr, "******* Creating Subscriber Details: ********\n");
  fprintf(stderr, "Subscriber topic %s\n", topic_reader->get_topicdescription()->get_name());
  fprintf(stderr, "Subscriber address %p\n", static_cast<void *>(dds_subscriber));
  fprintf(stderr, "******\n");
#endif

  subscription->can_loan_messages = false;
  return subscription;
fail:
  if (topic_str) {
    DDS::String_free(topic_str);
    topic_str = nullptr;
  }
  if (subscription) {
    rmw_subscription_free(subscription);
  }
  // Assumption: participant is valid.
  if (dds_subscriber) {
    if (topic_reader) {
      if (read_condition) {
        if (topic_reader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
          std::stringstream ss;
          ss << "leaking readcondition while handling failure at " <<
            __FILE__ << ":" << __LINE__ << '\n';
          (std::cerr << ss.str()).flush();
        }
      }
      if (dds_subscriber->delete_datareader(topic_reader) != DDS::RETCODE_OK) {
        std::stringstream ss;
        ss << "leaking datareader while handling failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
    if (participant->delete_subscriber(dds_subscriber) != DDS::RETCODE_OK) {
      std::stringstream ss;
      std::cerr << "leaking subscriber while handling failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (subscriber_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      subscriber_listener->~ConnextSubscriberListener(), ConnextSubscriberListener)
    rmw_free(subscriber_listener);
  }
  if (subscriber_info) {
    if (subscriber_info->listener_) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        subscriber_info->listener_->~ConnextSubscriberListener(), ConnextSubscriberListener)
      rmw_free(subscriber_info->listener_);
    }
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      subscriber_info->~ConnextStaticSubscriberInfo(), ConnextStaticSubscriberInfo)
    rmw_free(subscriber_info);
  }
  if (info_buf) {
    rmw_free(info_buf);
  }
  if (listener_buf) {
    rmw_free(listener_buf);
  }

  return NULL;
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

  auto info = static_cast<ConnextStaticSubscriberInfo *>(subscription->data);
  if (!info) {
    RMW_SET_ERROR_MSG("subscriber internal data is invalid");
    return RMW_RET_ERROR;
  }
  if (!info->listener_) {
    RMW_SET_ERROR_MSG("subscriber internal listener is invalid");
    return RMW_RET_ERROR;
  }

  *publisher_count = info->listener_->current_count();

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<ConnextStaticSubscriberInfo *>(subscription->data);
  if (!info) {
    RMW_SET_ERROR_MSG("subscription internal data is invalid");
    return RMW_RET_ERROR;
  }
  DDS::DataReader * data_reader = info->topic_reader_;
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
    node->implementation_identifier, rti_connext_identifier,
    return RMW_RET_ERROR)

  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rti_connext_identifier,
    return RMW_RET_ERROR)

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return RMW_RET_ERROR;
  }
  // TODO(wjwwood): need to figure out when to unregister types with the participant.
  auto result = RMW_RET_OK;
  ConnextStaticSubscriberInfo * subscriber_info =
    static_cast<ConnextStaticSubscriberInfo *>(subscription->data);
  if (subscriber_info) {
    node_info->subscriber_listener->remove_information(
      subscriber_info->dds_subscriber_->get_instance_handle(), EntityType::Subscriber);
    node_info->subscriber_listener->trigger_graph_guard_condition();
    auto dds_subscriber = subscriber_info->dds_subscriber_;
    if (dds_subscriber) {
      auto topic_reader = subscriber_info->topic_reader_;
      if (topic_reader) {
        auto read_condition = subscriber_info->read_condition_;
        if (read_condition) {
          if (topic_reader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
            RMW_SET_ERROR_MSG("failed to delete readcondition");
            result = RMW_RET_ERROR;
          }
          subscriber_info->read_condition_ = nullptr;
        }
        if (dds_subscriber->delete_datareader(topic_reader) != DDS::RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete datareader");
          result = RMW_RET_ERROR;
        }
        subscriber_info->topic_reader_ = nullptr;
      } else if (subscriber_info->read_condition_) {
        RMW_SET_ERROR_MSG("cannot delete readcondition because the datareader is null");
        result = RMW_RET_ERROR;
      }
      if (participant->delete_subscriber(dds_subscriber) != DDS::RETCODE_OK) {
        RMW_SET_ERROR_MSG("failed to delete subscriber");
        result = RMW_RET_ERROR;
      }
      subscriber_info->dds_subscriber_ = nullptr;
    } else if (subscriber_info->topic_reader_) {
      RMW_SET_ERROR_MSG("cannot delete datareader because the subscriber is null");
      result = RMW_RET_ERROR;
    }
    RMW_TRY_DESTRUCTOR(
      subscriber_info->~ConnextStaticSubscriberInfo(),
      ConnextStaticSubscriberInfo, result = RMW_RET_ERROR)
    rmw_free(subscriber_info);
    subscription->data = nullptr;
  }
  if (subscription->topic_name) {
    rmw_free(const_cast<char *>(subscription->topic_name));
  }
  rmw_subscription_free(subscription);

  return result;
}
}  // extern "C"
