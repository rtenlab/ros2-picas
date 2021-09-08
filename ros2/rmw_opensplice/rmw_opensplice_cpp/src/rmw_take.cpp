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

#include <string>
#include <FooCdrDataReader.h>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"

#include "identifier.hpp"
#include "types.hpp"
#include "rosidl_typesupport_opensplice_cpp/u__instanceHandle.h"

RMW_LOCAL
rmw_ret_t
take(
  const rmw_subscription_t * subscription, void * ros_message, bool * taken,
  DDS::InstanceHandle_t * sending_publication_handle,
  rmw_subscription_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) allocation;

  // Note: gid is allowed to be nullptr, if unused.
  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  if (ros_message == nullptr) {
    RMW_SET_ERROR_MSG("ros_message argument cannot be null");
    return RMW_RET_ERROR;
  }

  if (taken == nullptr) {
    RMW_SET_ERROR_MSG("taken argument cannot be null");
    return RMW_RET_ERROR;
  }

  OpenSpliceStaticSubscriberInfo * subscriber_info =
    static_cast<OpenSpliceStaticSubscriberInfo *>(subscription->data);
  if (!subscriber_info) {
    RMW_SET_ERROR_MSG("subscriber info handle is null");
    return RMW_RET_ERROR;
  }
  DDS::DataReader * topic_reader = subscriber_info->topic_reader;
  if (!topic_reader) {
    RMW_SET_ERROR_MSG("topic reader handle is null");
    return RMW_RET_ERROR;
  }
  const message_type_support_callbacks_t * callbacks = subscriber_info->callbacks;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  const char * error_string = callbacks->take(
    topic_reader,
    subscription->options.ignore_local_publications,
    ros_message, taken, sending_publication_handle);
  // If no data was taken, that's not captured as an error here, but instead taken is set to false.
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to take: ") + error_string).c_str());
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

RMW_LOCAL
void
report_serialize_take_error(
  const OpenSpliceStaticSubscriberInfo * subscriber_info,
  DDS::ReturnCode_t status)
{
  DDS::TopicDescription_var td = subscriber_info->topic_reader->get_topicdescription();
  DDS::String_var tn = td->get_type_name();
  std::string prefix(tn);
  const char * error_string;

  switch (status) {
    case DDS::RETCODE_ERROR:
      error_string = "_DataReader.take failed with: an internal error has occurred";
      break;
    case DDS::RETCODE_ALREADY_DELETED:
      error_string = "_DataReader.take failed with: this DataReader has already been deleted";
      break;
    case DDS::RETCODE_OUT_OF_RESOURCES:
      error_string = "_DataReader.take failed with: out of resources";
      break;
    case DDS::RETCODE_NOT_ENABLED:
      error_string = "_DataReader.take failed with: DataReader is not enabled";
      break;
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      error_string = "_DataReader.take failed with: "
        "a precondition is not met, one of: "
        "max_samples > maximum and max_samples != LENGTH_UNLIMITED, or "
        "the two sequences do not have matching parameters (length, maximum, release), or "
        "maximum > 0 and release is false.";
      break;
    default:
      error_string = "_DataReader.take failed with unknown return code";
      break;
  }

  RMW_SET_ERROR_MSG((std::string("failed to take: ") + prefix + error_string).c_str());
}

RMW_LOCAL
rmw_ret_t
take_serialized_sample(
  const OpenSpliceStaticSubscriberInfo * subscriber_info,
  rmw_serialized_message_t * serialized_message,
  bool ignore_local_publications,
  bool * taken,
  DDS::InstanceHandle_t * sending_publication_handle,
  rmw_subscription_allocation_t * allocation)
{
  (void) allocation;
  rmw_ret_t ret = RMW_RET_OK;
  DDS::CDRSample sample;
  DDS::SampleInfo info;
  DDS::DataReader * topic_reader = subscriber_info->topic_reader;
  DDS::OpenSplice::FooCdrDataReader cdr_reader(topic_reader);

  *taken = false;

  DDS::ReturnCode_t status = cdr_reader.take_cdr(
    sample, info, DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);

  if (status == DDS::RETCODE_OK) {
    *taken = info.valid_data;
  } else if (status == DDS::RETCODE_NO_DATA) {
    *taken = false;
  } else {
    report_serialize_take_error(subscriber_info, status);
    return RMW_RET_ERROR;
  }

  if (*taken) {
    DDS::InstanceHandle_t sender_handle = info.publication_handle;
    auto sender_gid = u_instanceHandleToGID(sender_handle);
    if (ignore_local_publications) {
      // compare the system id from the sender and this receiver
      // if they are equal the sample has been sent from this process and should be ignored
      DDS::InstanceHandle_t receiver_handle = topic_reader->get_instance_handle();
      auto receiver_gid = u_instanceHandleToGID(receiver_handle);
      if (sender_gid.systemId == receiver_gid.systemId) {
        // ignore sample
        *taken = false;
      }
    }
    // This is nullptr when being used with plain rmw_take, so check first.
    if (sending_publication_handle) {
      *static_cast<DDS::InstanceHandle_t *>(sending_publication_handle) = sender_handle;
    }
  }

  if (*taken) {
    DDS::CDRBlob & blob = sample.blob;
    auto buffer_size = blob.length();
    if (serialized_message->buffer_capacity < buffer_size) {
      ret = rmw_serialized_message_resize(serialized_message, buffer_size);
    }
    if (ret == RMW_RET_OK) {
      serialized_message->buffer_length = buffer_size;
      memcpy(serialized_message->buffer, blob.get_buffer(FALSE), buffer_size);
    } else {
      *taken = false;
    }
  }

  return ret;
}

RMW_LOCAL
rmw_ret_t
take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  DDS::InstanceHandle_t * sending_publication_handle,
  rmw_subscription_allocation_t * allocation)
{
  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription handle is null");
    return RMW_RET_ERROR;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  if (serialized_message == nullptr) {
    RMW_SET_ERROR_MSG("serialized_message argument cannot be null");
    return RMW_RET_ERROR;
  }

  if (taken == nullptr) {
    RMW_SET_ERROR_MSG("taken argument cannot be null");
    return RMW_RET_ERROR;
  }

  OpenSpliceStaticSubscriberInfo * subscriber_info =
    static_cast<OpenSpliceStaticSubscriberInfo *>(subscription->data);
  if (!subscriber_info) {
    RMW_SET_ERROR_MSG("subscriber info handle is null");
    return RMW_RET_ERROR;
  }

  const message_type_support_callbacks_t * callbacks = subscriber_info->callbacks;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  return take_serialized_sample(
    subscriber_info,
    serialized_message,
    subscription->options.ignore_local_publications,
    taken,
    sending_publication_handle,
    allocation);
}

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return take(subscription, ros_message, taken, nullptr, allocation);
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  if (!message_info) {
    RMW_SET_ERROR_MSG("message info is null");
    return RMW_RET_ERROR;
  }
  DDS::InstanceHandle_t sending_publication_handle;
  auto ret = take(subscription, ros_message, taken, &sending_publication_handle, allocation);
  if (ret != RMW_RET_OK) {
    // Error string is already set.
    return RMW_RET_ERROR;
  }

  rmw_gid_t * sender_gid = &message_info->publisher_gid;
  sender_gid->implementation_identifier = opensplice_cpp_identifier;
  memset(sender_gid->data, 0, RMW_GID_STORAGE_SIZE);
  auto detail = reinterpret_cast<OpenSplicePublisherGID *>(sender_gid->data);
  detail->publication_handle = sending_publication_handle;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return take_serialized_message(subscription, serialized_message, taken, nullptr, allocation);
}

rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  if (!message_info) {
    RMW_SET_ERROR_MSG("message info is null");
    return RMW_RET_ERROR;
  }
  DDS::InstanceHandle_t sending_publication_handle;
  auto ret = take_serialized_message(
    subscription, serialized_message, taken, &sending_publication_handle, allocation);
  if (ret != RMW_RET_OK) {
    // Error string is already set.
    return RMW_RET_ERROR;
  }

  rmw_gid_t * sender_gid = &message_info->publisher_gid;
  sender_gid->implementation_identifier = opensplice_cpp_identifier;
  memset(sender_gid->data, 0, RMW_GID_STORAGE_SIZE);
  auto detail = reinterpret_cast<OpenSplicePublisherGID *>(sender_gid->data);
  detail->publication_handle = sending_publication_handle;

  return RMW_RET_OK;
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

  RMW_SET_ERROR_MSG("rmw_take_loaned_message not implemented for rmw_opensplice_cpp");
  return RMW_RET_UNSUPPORTED;
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

  RMW_SET_ERROR_MSG("rmw_take_loaned_message_with_info not implemented for rmw_opensplice_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  (void) subscription;
  (void) loaned_message;

  RMW_SET_ERROR_MSG(
    "rmw_release_loaned_message_from_subscription not implemented for rmw_opensplice_cpp");
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
