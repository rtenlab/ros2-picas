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
#include <FooCdrDataWriter.h>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"

#include "identifier.hpp"
#include "types.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
RMW_LOCAL
void
report_serialize_publish_error(
  const OpenSpliceStaticPublisherInfo * publisher_info,
  DDS::ReturnCode_t status)
{
  DDS::Topic_var tp = publisher_info->topic_writer->get_topic();
  DDS::String_var tn = tp->get_type_name();
  std::string prefix(tn);
  const char * error_string;

  switch (status) {
    case DDS::RETCODE_ERROR:
      error_string = "_DataWriter.write: an internal error has occurred";
      break;
    case DDS::RETCODE_ALREADY_DELETED:
      error_string = "_DataWriter.write: DataWriter has already been deleted";
      break;
    case DDS::RETCODE_OUT_OF_RESOURCES:
      error_string = "_DataWriter.write: out of resources";
      break;
    case DDS::RETCODE_NOT_ENABLED:
      error_string = "_DataWriter.write: DataWriter is not enabled";
      break;
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      error_string = "_DataWriter.write: precondition not met";
      break;
    case DDS::RETCODE_TIMEOUT:
      error_string = "_DataWriter.write: writing resulted in blocking and"
        " then exceeded the timeout set by the "
        "max_blocking_time of the ReliabilityQosPolicy";
      break;
    default:
      error_string = "_DataWriter.take failed with unknown return code";
      break;
  }

  RMW_SET_ERROR_MSG((std::string("failed to publish: ") + prefix + error_string).c_str());
}

rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) allocation;

  if (!publisher) {
    RMW_SET_ERROR_MSG("publisher handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)
  if (!ros_message) {
    RMW_SET_ERROR_MSG("ros message handle is null");
    return RMW_RET_ERROR;
  }

  const OpenSpliceStaticPublisherInfo * publisher_info =
    static_cast<const OpenSpliceStaticPublisherInfo *>(publisher->data);
  if (!publisher_info) {
    RMW_SET_ERROR_MSG("publisher info handle is null");
    return RMW_RET_ERROR;
  }
  DDS::DataWriter * topic_writer = publisher_info->topic_writer;
  const message_type_support_callbacks_t * callbacks = publisher_info->callbacks;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  const char * error_string = callbacks->publish(topic_writer, ros_message);
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to publish:") + error_string).c_str());
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) allocation;

  if (!publisher) {
    RMW_SET_ERROR_MSG("publisher handle is null");
    return RMW_RET_ERROR;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher handle,
    publisher->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  if (!serialized_message) {
    RMW_SET_ERROR_MSG("ros serialized_message handle is null");
    return RMW_RET_ERROR;
  }

  OpenSpliceStaticPublisherInfo * publisher_info =
    static_cast<OpenSpliceStaticPublisherInfo *>(publisher->data);
  if (!publisher_info) {
    RMW_SET_ERROR_MSG("publisher info handle is null");
    return RMW_RET_ERROR;
  }

  DDS::DataWriter * topic_writer = publisher_info->topic_writer;

  const message_type_support_callbacks_t * callbacks = publisher_info->callbacks;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  DDS::OpenSplice::FooCdrDataWriter cdr_writer(topic_writer);

  DDS::CDRSample sample;

  unsigned char * buffer = static_cast<unsigned char *>(serialized_message->buffer);
  unsigned int length = static_cast<unsigned int>(serialized_message->buffer_length);

  sample.blob = DDS::CDRBlob(length, length, buffer, false);

  DDS::ReturnCode_t status = cdr_writer.write_cdr(sample, DDS::HANDLE_NIL);
  if (status != DDS::RETCODE_OK) {
    report_serialize_publish_error(publisher_info, status);
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
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

  RMW_SET_ERROR_MSG("rmw_publish_loaned_message not implemented for rmw_opensplice_cpp");
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
