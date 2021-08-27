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

#include <limits>

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/types.h"

#include "rmw_connext_shared_cpp/types.hpp"

#include "rmw_connext_cpp/connext_static_subscriber_info.hpp"
#include "rmw_connext_cpp/identifier.hpp"

// include patched generated code from the build folder
#include "./connext_static_serialized_dataSupport.h"
#include "./connext_static_serialized_data.h"

static bool
is_local_publication(
  const DDS::SampleInfo & sample_info,
  DDS::DataReader * dds_data_reader)
{
  bool is_local = true;
  // compare the lower 12 octets of the guids from the sender and this receiver
  // if they are equal the sample has been sent from this process and should be ignored
  DDS::GUID_t sender_guid = sample_info.original_publication_virtual_guid;
  DDS::InstanceHandle_t receiver_instance_handle = dds_data_reader->get_instance_handle();
  for (size_t i = 0; i < 12; ++i) {
    DDS::Octet * sender_element = &(sender_guid.value[i]);
    DDS::Octet * receiver_element =
      &(reinterpret_cast<DDS::Octet *>(&receiver_instance_handle)[i]);
    if (*sender_element != *receiver_element) {
      is_local = false;
      break;
    }
  }
  return is_local;
}

static bool
take(
  DDS::DataReader * dds_data_reader,
  bool ignore_local_publications,
  rcutils_uint8_array_t * cdr_stream,
  bool * taken,
  void * sending_publication_handle,
  rmw_subscription_allocation_t * allocation)
{
  (void) allocation;
  if (!dds_data_reader) {
    RMW_SET_ERROR_MSG("dds_data_reader is null");
    return false;
  }
  if (!cdr_stream) {
    RMW_SET_ERROR_MSG("cdr stream handle is null");
    return false;
  }
  if (!taken) {
    RMW_SET_ERROR_MSG("taken handle is null");
    return false;
  }

  ConnextStaticSerializedDataDataReader * data_reader =
    ConnextStaticSerializedDataDataReader::narrow(dds_data_reader);
  if (!data_reader) {
    RMW_SET_ERROR_MSG("failed to narrow data reader");
    return false;
  }

  ConnextStaticSerializedDataSeq dds_messages;
  DDS::SampleInfoSeq sample_infos;
  bool ignore_sample = false;

  DDS::ReturnCode_t status = data_reader->take(
    dds_messages,
    sample_infos,
    1,
    DDS::ANY_SAMPLE_STATE,
    DDS::ANY_VIEW_STATE,
    DDS::ANY_INSTANCE_STATE);
  if (status == DDS::RETCODE_NO_DATA) {
    data_reader->return_loan(dds_messages, sample_infos);
    *taken = false;
    return true;
  }
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("take failed");
    data_reader->return_loan(dds_messages, sample_infos);
    return false;
  }

  DDS::SampleInfo & sample_info = sample_infos[0];
  if (!sample_info.valid_data) {
    // skip sample without data
    ignore_sample = true;
  } else if (ignore_local_publications && is_local_publication(sample_info, dds_data_reader)) {
    ignore_sample = true;
  }
  if (sample_info.valid_data && sending_publication_handle) {
    *static_cast<DDS::InstanceHandle_t *>(sending_publication_handle) =
      sample_info.publication_handle;
  }

  if (!ignore_sample) {
    cdr_stream->buffer_length = dds_messages[0].serialized_data.length();
    cdr_stream->buffer_capacity = cdr_stream->buffer_length;
    // TODO(karsten1987): This malloc has to go!
    cdr_stream->buffer =
      reinterpret_cast<uint8_t *>(malloc(cdr_stream->buffer_length * sizeof(uint8_t)));

    if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
      RMW_SET_ERROR_MSG("cdr_stream->buffer_length unexpectedly larger than max unsiged int value");
      data_reader->return_loan(dds_messages, sample_infos);
      *taken = false;
      return false;
    }
    memcpy(cdr_stream->buffer, &dds_messages[0].serialized_data[0], cdr_stream->buffer_length);

    *taken = true;
  } else {
    *taken = false;
  }

  data_reader->return_loan(dds_messages, sample_infos);

  return status == DDS::RETCODE_OK;
}

extern "C"
{
rmw_ret_t
_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  DDS::InstanceHandle_t * sending_publication_handle,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(
    subscription, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rti_connext_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_ARGUMENT_FOR_NULL(
    ros_message, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_ARGUMENT_FOR_NULL(
    taken, RMW_RET_INVALID_ARGUMENT);

  ConnextStaticSubscriberInfo * subscriber_info =
    static_cast<ConnextStaticSubscriberInfo *>(subscription->data);
  if (!subscriber_info) {
    RMW_SET_ERROR_MSG("subscriber info handle is null");
    return RMW_RET_ERROR;
  }
  DDS::DataReader * topic_reader = subscriber_info->topic_reader_;
  if (!topic_reader) {
    RMW_SET_ERROR_MSG("topic reader handle is null");
    return RMW_RET_ERROR;
  }
  const message_type_support_callbacks_t * callbacks = subscriber_info->callbacks_;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  // fetch the incoming message as cdr stream
  rcutils_uint8_array_t cdr_stream = rcutils_get_zero_initialized_uint8_array();
  if (!take(
      topic_reader, subscription->options.ignore_local_publications, &cdr_stream, taken,
      sending_publication_handle, allocation))
  {
    RMW_SET_ERROR_MSG("error occured while taking message");
    return RMW_RET_ERROR;
  }
  // convert the cdr stream to the message
  if (*taken && !callbacks->to_message(&cdr_stream, ros_message)) {
    RMW_SET_ERROR_MSG("can't convert cdr stream to ros message");
    return RMW_RET_ERROR;
  }

  // the call to take allocates memory for the serialized message
  // we have to free this here again
  free(cdr_stream.buffer);

  return RMW_RET_OK;
}

rmw_ret_t
_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void) allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(
    subscription, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rti_connext_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_ARGUMENT_FOR_NULL(
    message_sequence, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_ARGUMENT_FOR_NULL(
    message_info_sequence, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_ARGUMENT_FOR_NULL(
    taken, RMW_RET_INVALID_ARGUMENT);

  if (count == 0u) {
    RMW_SET_ERROR_MSG("count cant be 0");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_sequence->capacity) {
    RMW_SET_ERROR_MSG("insufficient capacity in message sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_info_sequence->capacity) {
    RMW_SET_ERROR_MSG("insufficient capacity in message info sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > static_cast<size_t>((std::numeric_limits<DDS_Long>::max)())) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "cannot take %ld samples at once, limit is %d",
      count, (std::numeric_limits<DDS_Long>::max)());
    return RMW_RET_ERROR;
  }

  ConnextStaticSubscriberInfo * subscriber_info =
    static_cast<ConnextStaticSubscriberInfo *>(subscription->data);
  if (!subscriber_info) {
    RMW_SET_ERROR_MSG("subscriber info handle is null");
    return RMW_RET_ERROR;
  }
  DDS::DataReader * topic_reader = subscriber_info->topic_reader_;
  if (!topic_reader) {
    RMW_SET_ERROR_MSG("topic reader handle is null");
    return RMW_RET_ERROR;
  }
  const message_type_support_callbacks_t * callbacks = subscriber_info->callbacks_;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  DDS::DataReader * dds_data_reader = topic_reader;
  bool ignore_local_publications = subscription->options.ignore_local_publications;

  ConnextStaticSerializedDataDataReader * data_reader =
    ConnextStaticSerializedDataDataReader::narrow(topic_reader);
  if (!data_reader) {
    RMW_SET_ERROR_MSG("failed to narrow data reader");
    return RMW_RET_ERROR;
  }

  ConnextStaticSerializedDataSeq dds_messages;
  DDS::SampleInfoSeq sample_infos;

  *taken = 0;

  DDS::ReturnCode_t status = data_reader->take(
    dds_messages,
    sample_infos,
    static_cast<DDS_Long>(count),
    DDS::ANY_SAMPLE_STATE,
    DDS::ANY_VIEW_STATE,
    DDS::ANY_INSTANCE_STATE);

  if (status == DDS::RETCODE_NO_DATA) {
    data_reader->return_loan(dds_messages, sample_infos);
    return RMW_RET_OK;
  }
  if (status != DDS::RETCODE_OK) {
    data_reader->return_loan(dds_messages, sample_infos);
    RMW_SET_ERROR_MSG("take failed");
    return RMW_RET_ERROR;
  }

  for (int ii = 0; ii < dds_messages.length(); ++ii) {
    bool ignore_sample = false;
    const DDS::SampleInfo & sample_info = sample_infos[ii];
    if (!sample_info.valid_data) {
      ignore_sample = true;
    } else if (ignore_local_publications && is_local_publication(sample_info, dds_data_reader)) {
      ignore_sample = true;
    }

    if (!ignore_sample) {
      rcutils_uint8_array_t cdr_stream = rcutils_get_zero_initialized_uint8_array();

      cdr_stream.buffer_length = dds_messages[ii].serialized_data.length();
      cdr_stream.buffer_capacity = dds_messages[ii].serialized_data.length();
      cdr_stream.buffer = reinterpret_cast<uint8_t *>(&dds_messages[ii].serialized_data[0]);

      if (callbacks->to_message(&cdr_stream, message_sequence->data[*taken])) {
        rmw_gid_t * sender_gid = &message_info_sequence->data[*taken].publisher_gid;
        sender_gid->implementation_identifier = rti_connext_identifier;
        memset(sender_gid->data, 0, RMW_GID_STORAGE_SIZE);
        auto detail = reinterpret_cast<ConnextPublisherGID *>(sender_gid->data);
        detail->publication_handle = sample_info.publication_handle;
        (*taken)++;
      }
    }
  }

  message_sequence->size = *taken;
  message_info_sequence->size = *taken;

  data_reader->return_loan(dds_messages, sample_infos);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return _take(subscription, ros_message, taken, nullptr, allocation);
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_FOR_NULL_WITH_MSG(
    message_info, "message info is null",
    return RMW_RET_INVALID_ARGUMENT);
  DDS::InstanceHandle_t sending_publication_handle;
  auto ret = _take(subscription, ros_message, taken, &sending_publication_handle, allocation);
  if (ret != RMW_RET_OK) {
    // Error string is already set.
    return ret;
  }

  rmw_gid_t * sender_gid = &message_info->publisher_gid;
  sender_gid->implementation_identifier = rti_connext_identifier;
  memset(sender_gid->data, 0, RMW_GID_STORAGE_SIZE);
  auto detail = reinterpret_cast<ConnextPublisherGID *>(sender_gid->data);
  detail->publication_handle = sending_publication_handle;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  return _take_sequence(
    subscription, count, message_sequence, message_info_sequence, taken,
    allocation);
}

rmw_ret_t
_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  DDS::InstanceHandle_t * sending_publication_handle,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(
    subscription, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rti_connext_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION)

  RMW_CHECK_ARGUMENT_FOR_NULL(
    serialized_message, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_ARGUMENT_FOR_NULL(
    taken, RMW_RET_INVALID_ARGUMENT);

  ConnextStaticSubscriberInfo * subscriber_info =
    static_cast<ConnextStaticSubscriberInfo *>(subscription->data);
  if (!subscriber_info) {
    RMW_SET_ERROR_MSG("subscriber info handle is null");
    return RMW_RET_ERROR;
  }
  DDS::DataReader * topic_reader = subscriber_info->topic_reader_;
  if (!topic_reader) {
    RMW_SET_ERROR_MSG("topic reader handle is null");
    return RMW_RET_ERROR;
  }
  const message_type_support_callbacks_t * callbacks = subscriber_info->callbacks_;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  // fetch the incoming message as cdr stream
  if (!take(
      topic_reader, subscription->options.ignore_local_publications, serialized_message, taken,
      sending_publication_handle, allocation))
  {
    RMW_SET_ERROR_MSG("error occured while taking message");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return _take_serialized_message(subscription, serialized_message, taken, nullptr, allocation);
}

rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(
    message_info, RMW_RET_INVALID_ARGUMENT);

  DDS::InstanceHandle_t sending_publication_handle;
  auto ret = _take_serialized_message(
    subscription, serialized_message, taken,
    &sending_publication_handle, allocation);
  if (ret != RMW_RET_OK) {
    // Error string is already set.
    return ret;
  }

  rmw_gid_t * sender_gid = &message_info->publisher_gid;
  sender_gid->implementation_identifier = rti_connext_identifier;
  memset(sender_gid->data, 0, RMW_GID_STORAGE_SIZE);
  auto detail = reinterpret_cast<ConnextPublisherGID *>(sender_gid->data);
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

  RMW_SET_ERROR_MSG("rmw_take_loaned_message not implemented for rmw_connext_cpp");
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

  RMW_SET_ERROR_MSG("rmw_take_loaned_message_with_info not implemented for rmw_connext_cpp");
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
    "rmw_release_loaned_message not implemented for rmw_connext_cpp");
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
