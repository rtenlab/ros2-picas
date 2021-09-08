// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__IMPL__ERROR_CHECKING_HPP_
#define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__IMPL__ERROR_CHECKING_HPP_

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

namespace rosidl_typesupport_opensplice_cpp
{
namespace impl
{

inline const char *
check_get_default_topic_qos(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::get_default_topic_qos: an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::get_default_topic_qos: "
             "the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::get_default_topic_qos: out of resources";
    default:
      return "DomainParticipant::get_default_topic_qos: unknown return code";
  }
}

inline const char *
check_delete_topic(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::delete_topic: an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "DomainParticipant::delete_topic: parameter a_topic is not a valid Topic_ptr";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::delete_topic: the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::delete_topic: out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return
        "DomainParticipant::delete_topic: precondition not met: "
        "the operation is called on a different DomainParticipant than was used to create it, "
        "or the Topic is still referenced by other objects";
    default:
      return "DomainParticipant::delete_topic: unknown return code";
  }
}

inline const char *
check_delete_contentfilteredtopic(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::delete_contentfilteredtopic: an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "DomainParticipant::delete_contentfilteredtopic: "
             "the parameter a_contentfilteredtopic is not a valid ContentFilteredTopic_ptr";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::delete_contentfilteredtopic: "
             "the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::delete_contentfilteredtopic: out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return
        "DomainParticipant::delete_contentfilteredtopic: precondition not met: "
        "the operation is called on a different DomainParticipant than was used to create it, "
        "or the Topic is still referenced by other objects";
    default:
      return "DomainParticipant::delete_contentfilteredtopic: unknown return code";
  }
}

inline const char *
check_get_default_publisher_qos(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::get_default_publisher_qos: an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::get_default_publisher_qos: "
             "the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::get_default_publisher_qos: out of resources";
    default:
      return "DomainParticipant::get_default_publisher_qos: unknown return code";
  }
}

inline const char *
check_delete_publisher(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::delete_publisher: an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "DomainParticipant::delete_publisher: the parameter p is not a valid Publisher_ptr";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::delete_publisher: "
             "the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::delete_publisher: out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return
        "DomainParticipant::delete_publisher: precondition not met: "
        "the operation is called on a different DomainParticipant than was used to create it, "
        "or the Publisher contains one or more DataWriter objects";
    default:
      return "DomainParticipant::delete_publisher: unknown return code";
  }
}

inline const char *
check_get_default_subscriber_qos(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::get_default_subscriber_qos: an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::get_default_subscriber_qos: "
             "the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::get_default_subscriber_qos: out of resources";
    default:
      return "DomainParticipant::get_default_subscriber_qos: unknown return code";
  }
}

inline const char *
check_get_default_datawriter_qos(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "Publisher::get_default_datawriter_qos: an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "Publisher::get_default_datawriter_qos: "
             "the Publisher has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "Publisher::get_default_datawriter_qos: out of resources";
    default:
      return "Publisher::get_default_datawriter_qos: unknown return code";
  }
}

inline const char *
check_delete_datawriter(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "Publisher::delete_datawriter: an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "Publisher::delete_datawriter: "
             "the parameter a_datawriter is not a valid DataWriter_ptr";
    case DDS::RETCODE_ALREADY_DELETED:
      return "Publisher::delete_datawriter: "
             "the Publisher has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "Publisher::delete_datawriter: out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "Publisher::delete_datawriter: precondition not met: "
             "the operation is called on a different Publisher than was used to create it";
    default:
      return "Publisher::delete_datawriter: unknown return code";
  }
}

inline const char *
check_delete_subscriber(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::delete_subscriber: an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "Subscriber::delete_subscriber: the parameter s is not a valid Subscriber_ptr";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::delete_subscriber: "
             "the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::delete_subscriber: out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return
        "DomainParticipant::delete_subscriber: precondition not met: "
        "the operation is called on a different DomainParticipant than was used to create it, "
        "or the Subscriber contains one or more DataReader objects";
    default:
      return "DomainParticipant::delete_subscriber: unknown return code";
  }
}

inline const char *
check_get_default_datareader_qos(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "DomainParticipant::get_default_subscriber_qos: an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "DomainParticipant::get_default_subscriber_qos: "
             "the DomainParticipant has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "DomainParticipant::get_default_subscriber_qos: out of resources";
    default:
      return "DomainParticipant::get_default_subscriber_qos: unknown return code";
  }
}

inline const char *
check_delete_datareader(DDS::ReturnCode_t status)
{
  switch (status) {
    case DDS::RETCODE_OK:
      return nullptr;
    case DDS::RETCODE_ERROR:
      return "Subscriber::delete_datareader: an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "Subscriber::delete_datareader: "
             "the parameter a_datareader is not a valid DataReader_ptr";
    case DDS::RETCODE_ALREADY_DELETED:
      return "Subscriber::delete_datareader: "
             "the Subscriber has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "Subscriber::delete_datareader: out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return
        "Subscriber::delete_datareader: precondition not met: "
        "the operation is called on a different Subscriber than the one which created it, or "
        "the DataReader contains one or more ReadCondition or QueryCondition objects, or "
        "the DataReader still contains unreturned loans";
    default:
      return "Subscriber::delete_datareader: unknown return code";
  }
}

}  // namespace impl
}  // namespace rosidl_typesupport_opensplice_cpp

#endif  // ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__IMPL__ERROR_CHECKING_HPP_
