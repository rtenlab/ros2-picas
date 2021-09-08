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

#ifndef ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__REQUESTER_HPP_
#define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__REQUESTER_HPP_

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

#include <atomic>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>

#include "rmw/error_handling.h"
#include "rmw/types.h"

#include "rosidl_typesupport_opensplice_cpp/impl/error_checking.hpp"
#include "rosidl_typesupport_opensplice_cpp/message_type_support.h"
#include "rosidl_typesupport_opensplice_cpp/service_type_support.h"
#include "rosidl_typesupport_opensplice_cpp/misc.hpp"

namespace rosidl_typesupport_opensplice_cpp
{

template<typename T>
class Sample;

template<typename T>
class TemplateDataReader;

template<typename T>
class TemplateDataWriter;

template<typename RequestT, typename ResponseT>
class Requester
{
public:
  Requester(
    DDS::DomainParticipant * participant, const std::string & service_name,
    const std::string & service_type_name)
  : participant_(participant), service_name_(service_name),
    service_type_name_(service_type_name), response_datareader_(nullptr),
    request_datawriter_(nullptr), response_topic_(nullptr),
    content_filtered_response_topic_(nullptr), request_topic_(nullptr),
    response_subscriber_(nullptr), request_publisher_(nullptr),
    sequence_number_(0)
  {}

  const char * init(
    const DDS::DataReaderQos * datareader_qos,
    const DDS::DataWriterQos * datawriter_qos,
    bool avoid_ros_namespace_conventions) noexcept
  {
    std::random_device rd;
    std::default_random_engine e1(rd());
    // NOTE: use extra parentheses to avoid macro expansion. On Windows,
    // max and min are defined as macros in <windows.h>
    // See http://stackoverflow.com/a/2561377/470581
    std::uniform_int_distribution<uint64_t> uniform_dist(
      (std::numeric_limits<uint64_t>::min)(),
      (std::numeric_limits<uint64_t>::max)());
    writer_guid_.first = uniform_dist(e1);
    writer_guid_.second = uniform_dist(e1);

    // NOTE(esteve): OpenSplice's query encoding on Windows seems to have issues with
    // arguments that produce queries that are too long
    // https://github.com/PrismTech/opensplice/issues/21
    std::stringstream ss;
    ss << "client_guid_0_ = " << writer_guid_.first << " AND client_guid_1_ = " <<
      writer_guid_.second;

    std::string query(ss.str());

    DDS::StringSeq args;
    args.length(0);

    DDS::ReturnCode_t status;
    DDS::TopicQos default_topic_qos;
    DDS::PublisherQos publisher_qos;
    DDS::SubscriberQos subscriber_qos;
    std::string service_str;
    std::string request_type_name = service_type_name_ + "_Request_";
    std::string request_topic_name;
    std::string response_type_name = service_type_name_ + "_Response_";
    std::string response_topic_name;
    std::string content_filtered_topic_name;
    const char * estr = nullptr;

    if (!process_service_name(
        service_name_.c_str(), avoid_ros_namespace_conventions, service_str,
        request_topic_name, response_topic_name))
    {
      estr = "process_service_name: failed";
      goto fail;
    }

    content_filtered_topic_name = service_str +
      std::to_string(writer_guid_.first) + "_" + std::to_string(writer_guid_.second);

    // Create request Publisher and DataWriter
    status = participant_->get_default_publisher_qos(publisher_qos);
    if (nullptr != (estr = impl::check_get_default_publisher_qos(status))) {
      goto fail;
    }

    request_publisher_ =
      participant_->create_publisher(publisher_qos, NULL, DDS::STATUS_MASK_NONE);
    if (!request_publisher_) {
      estr = "DomainParticipant::create_publisher: failed for request";
      goto fail;
    }

    status = participant_->get_default_topic_qos(default_topic_qos);
    if (nullptr != (estr = impl::check_get_default_topic_qos(status))) {
      goto fail;
    }

    request_topic_ = participant_->create_topic(
      request_topic_name.c_str(), request_type_name.c_str(), default_topic_qos, NULL,
      DDS::STATUS_MASK_NONE);
    if (!request_topic_) {
      estr = "DomainParticipant::create_topic: failed for request";
      goto fail;
    }

    request_datawriter_ = request_publisher_->create_datawriter(
      request_topic_, *datawriter_qos, NULL, DDS::STATUS_MASK_NONE);
    if (!request_datawriter_) {
      estr = "Publisher::create_datawriter: failed for request";
      goto fail;
    }

    // Create response Subscriber and DataReader
    status = participant_->get_default_subscriber_qos(subscriber_qos);
    if (nullptr != (estr = impl::check_get_default_datareader_qos(status))) {
      goto fail;
    }

    response_subscriber_ = participant_->create_subscriber(
      subscriber_qos, NULL, DDS::STATUS_MASK_NONE);
    if (!response_subscriber_) {
      estr = "DomainParticipant::create_subscriber: failed for response";
      goto fail;
    }

    response_topic_ = participant_->create_topic(
      response_topic_name.c_str(), response_type_name.c_str(), default_topic_qos, NULL,
      DDS::STATUS_MASK_NONE);
    if (!response_topic_) {
      estr = "DomainParticipant::create_topic: failed for response";
      goto fail;
    }
    content_filtered_response_topic_ = participant_->create_contentfilteredtopic(
      content_filtered_topic_name.c_str(), response_topic_,
      query.c_str(),
      args);
    if (!content_filtered_response_topic_) {
      estr = "DomainParticipant::create_contentfilteredtopic: failed";
      goto fail;
    }

    response_datareader_ = response_subscriber_->create_datareader(
      content_filtered_response_topic_,
      *datareader_qos, NULL, DDS::STATUS_MASK_NONE);
    if (!response_datareader_) {
      estr = "Subscriber::create_datawriter: failed for response";
      goto fail;
    }

    return nullptr;
fail:
    if (response_datareader_) {
      // Assumption: subscriber is not null at this point.
      status = response_subscriber_->delete_datareader(response_datareader_);
      if (nullptr != impl::check_delete_datareader(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_datareader(status));
      }
    }
    if (response_subscriber_) {
      status = participant_->delete_subscriber(response_subscriber_);
      if (nullptr != impl::check_delete_subscriber(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_subscriber(status));
      }
    }
    if (request_datawriter_) {
      // Assumption: publisher is not null at this point.
      status = request_publisher_->delete_datawriter(request_datawriter_);
      if (nullptr != impl::check_delete_datawriter(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_datawriter(status));
      }
    }
    if (request_publisher_) {
      status = participant_->delete_publisher(request_publisher_);
      if (nullptr != impl::check_delete_publisher(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_publisher(status));
      }
    }
    if (content_filtered_response_topic_) {
      status = participant_->delete_contentfilteredtopic(content_filtered_response_topic_);
      if (nullptr != impl::check_delete_contentfilteredtopic(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_contentfilteredtopic(status));
      }
    }
    if (response_topic_) {
      status = participant_->delete_topic(response_topic_);
      if (nullptr != impl::check_delete_topic(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_topic(status));
      }
    }
    if (request_topic_) {
      status = participant_->delete_topic(request_topic_);
      if (nullptr != impl::check_delete_topic(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_topic(status));
      }
    }
    return estr;
  }

  const char * teardown() noexcept
  {
    const char * estr = nullptr;
    DDS::ReturnCode_t status;
    if (response_datareader_) {
      // Assumption: subscriber is not null at this point.
      status = response_subscriber_->delete_datareader(response_datareader_);
      if (nullptr != impl::check_delete_datareader(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_datareader(status));
        estr = "Error from Subscriber::delete_datareader in requester teardown";
      }
    }
    if (response_subscriber_) {
      status = participant_->delete_subscriber(response_subscriber_);
      if (nullptr != impl::check_delete_subscriber(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_subscriber(status));
        if (estr) {
          // print the old error string if it needs to be overwritten
          fprintf(stderr, "%s\n", estr);
        }
        estr = "Error from Participant::delete_subscriber in requester teardown";
      }
    }
    if (request_datawriter_) {
      // Assumption: publisher is not null at this point.
      status = request_publisher_->delete_datawriter(request_datawriter_);
      if (nullptr != impl::check_delete_datawriter(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_datawriter(status));
        if (estr) {
          // print the old error string if it needs to be overwritten
          fprintf(stderr, "%s\n", estr);
        }
        estr = "Error from Publisher::delete_datawriter in requester teardown";
      }
    }
    if (request_publisher_) {
      status = participant_->delete_publisher(request_publisher_);
      if (nullptr != impl::check_delete_publisher(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_publisher(status));
        if (estr) {
          // print the old error string if it needs to be overwritten
          fprintf(stderr, "%s\n", estr);
        }
        estr = "Error from Particpant::delete_publisher in requester teardown";
      }
    }
    if (content_filtered_response_topic_) {
      status = participant_->delete_contentfilteredtopic(content_filtered_response_topic_);
      if (nullptr != impl::check_delete_contentfilteredtopic(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_contentfilteredtopic(status));
        if (estr) {
          // print the old error string if it needs to be overwritten
          fprintf(stderr, "%s\n", estr);
        }
        estr = "Error from Particpant::delete_contentfilteredtopic in requester teardown";
      }
    }
    if (response_topic_) {
      status = participant_->delete_topic(response_topic_);
      if (nullptr != impl::check_delete_topic(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_topic(status));
        if (estr) {
          // print the old error string if it needs to be overwritten
          fprintf(stderr, "%s\n", estr);
        }
        estr = "Error from Particpant::delete_topic in requester teardown";
      }
    }
    if (request_topic_) {
      status = participant_->delete_topic(request_topic_);
      if (nullptr != impl::check_delete_topic(status)) {
        fprintf(stderr, "%s\n", impl::check_delete_topic(status));
        estr = "Error from Particpant::delete_topic in requester teardown";
      }
    }
    return estr;
  }

  /// Return NULL on success, otherwise an error string.
  const char *
  server_is_available(const rmw_node_t * node, bool * is_available) noexcept
  {
    (void)node;
    if (!is_available) {
      return "argument is_available is null";
    }
    *is_available = false;
    // In the OpenSplice RPC implementation, a server is ready when:
    //   - At least one reader is matched to the request writer.
    //   - At least one writer is matched to the reponse reader.
    DDS::ReturnCode_t retcode;
    DDS::PublicationMatchedStatus publication_status;
    retcode = request_datawriter_->get_publication_matched_status(publication_status);
    if (retcode != DDS::RETCODE_OK) {
      return "DataWriter::get_publication_matched_status: failed";
    }

    DDS::SubscriptionMatchedStatus subscription_status;
    retcode = response_datareader_->get_subscription_matched_status(subscription_status);
    if (retcode != DDS::RETCODE_OK) {
      return "DataReader::get_subscription_matched_status: failed";
    }

    if (publication_status.current_count == 0 ||
      subscription_status.current_count == 0)
    {
      return nullptr;
    }
    *is_available = true;
    return nullptr;
  }

  const char * take_response(Sample<ResponseT> & response, bool * taken) noexcept
  {
    return TemplateDataReader<Sample<ResponseT>>::take_sample(
      response_datareader_, response, taken);
  }

  const char * send_request(Sample<RequestT> & request) noexcept
  {
    request.sequence_number_ = ++sequence_number_;
    request.client_guid_0_ = writer_guid_.first;
    request.client_guid_1_ = writer_guid_.second;

    return TemplateDataWriter<Sample<RequestT>>::write_sample(request_datawriter_, request);
  }

  DDS::DataReader * get_response_datareader()
  {
    return response_datareader_;
  }

private:
  DDS::DomainParticipant * participant_;
  std::string service_name_;
  std::string service_type_name_;
  DDS::DataReader * response_datareader_;
  DDS::DataWriter * request_datawriter_;
  DDS::Topic * response_topic_;
  DDS::ContentFilteredTopic * content_filtered_response_topic_;
  DDS::Topic * request_topic_;
  DDS::Subscriber * response_subscriber_;
  DDS::Publisher * request_publisher_;
  std::atomic<int64_t> sequence_number_;
  std::pair<uint64_t, uint64_t> writer_guid_;
};

}  // namespace rosidl_typesupport_opensplice_cpp

#endif  // ROSIDL_TYPESUPPORT_OPENSPLICE_CPP__REQUESTER_HPP_
