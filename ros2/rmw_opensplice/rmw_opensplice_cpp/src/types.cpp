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

#include "types.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "rmw/error_handling.h"

#include "rosidl_typesupport_opensplice_cpp/misc.hpp"

#include "demangle.hpp"
#include "event_converter.hpp"
#include "namespace_prefix.hpp"

std::string
create_type_name(const message_type_support_callbacks_t * callbacks)
{
  return std::string(callbacks->message_namespace) + "::dds_::" + callbacks->message_name + "_";
}

CustomDataReaderListener::CustomDataReaderListener()
: print_discovery_logging_(false)
{
  // TODO(wjwwood): replace this with actual console logging when we have that
  char * discovery_logging_value = nullptr;
  const char * discovery_logging_env = "RMW_PRINT_DISCOVERY_LOGGING";
  size_t discovery_logging_size;
#ifndef _WIN32
  discovery_logging_value = getenv(discovery_logging_env);
  if (discovery_logging_value) {
    discovery_logging_size = strlen(discovery_logging_value);
  }
#else
  _dupenv_s(&discovery_logging_value, &discovery_logging_size, discovery_logging_env);
#endif
  if (discovery_logging_value) {
    std::string str(discovery_logging_value, discovery_logging_size);
    std::string str_lower(str);
    std::transform(
      str_lower.begin(), str_lower.end(), str_lower.begin(),
      [](unsigned char c) {
        return std::tolower(c);
      });
    if (str != "0" && str_lower != "false" && str_lower != "off") {
      print_discovery_logging_ = true;
    }
#ifdef _WIN32
    free(discovery_logging_value);
#endif
  }
}

size_t
CustomDataReaderListener::count_topic(const char * topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto count = std::count_if(
    topic_cache.getTopicGuidToInfo().begin(),
    topic_cache.getTopicGuidToInfo().end(),
    [&](auto tnt) -> bool {
      auto fqdn = _demangle_if_ros_topic(tnt.second.name);
      return fqdn == topic_name;
    });
  return (size_t) count;
}

void
CustomDataReaderListener::fill_topic_names_and_types(
  bool no_demangle,
  std::map<std::string, std::set<std::string>> & tnat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it : topic_cache.getTopicGuidToInfo()) {
    if (!no_demangle && (_get_ros_prefix_if_exists(it.second.name) !=
      rosidl_typesupport_opensplice_cpp::get_ros_topic_prefix()))
    {
      continue;
    }
    tnat[it.second.name].insert(it.second.type);
  }
}

void
CustomDataReaderListener::fill_service_names_and_types(
  std::map<std::string, std::set<std::string>> & services)
{
  for (auto it : topic_cache.getTopicGuidToInfo()) {
    std::string service_name = _demangle_service_from_topic(it.second.name);
    if (!service_name.length()) {
      // not a service
      continue;
    }
    std::string service_type = _demangle_service_type_only(it.second.type);
    if (service_type.length()) {
      services[service_name].insert(service_type);
    }
  }
}

void CustomDataReaderListener::fill_topic_names_and_types_by_participant(
  bool no_demangle,
  std::map<std::string, std::set<std::string>> & tnat,
  DDS::InstanceHandle_t & participant)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & map = topic_cache.getTopicTypesByGuid(participant);
  if (map.empty()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_opensplice_cpp",
      "No topics for participant");
    return;
  }
  for (auto & it : map) {
    if (!no_demangle && (_get_ros_prefix_if_exists(it.first) !=
      rosidl_typesupport_opensplice_cpp::get_ros_topic_prefix()))
    {
      continue;
    }
    tnat[it.first].insert(it.second.begin(), it.second.end());
  }
}

void CustomDataReaderListener::fill_service_names_and_types_by_participant(
  std::map<std::string, std::set<std::string>> & services,
  DDS::InstanceHandle_t & participant,
  const std::string & suffix)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & map = topic_cache.getTopicTypesByGuid(participant);
  if (map.empty()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_opensplice_cpp",
      "No services for participant");
    return;
  }
  for (auto & it : map) {
    std::string service_name = _demangle_service_from_topic(it.first);
    if (!service_name.length()) {
      // not a service
      continue;
    }
    // Check if the topic suffix matches and is at the end of the name
    const std::string & topic_name = it.first;
    auto suffix_position = topic_name.rfind(suffix);
    if (suffix_position == std::string::npos ||
      topic_name.length() - suffix_position - suffix.length() != 0)
    {
      continue;
    }

    for (auto & itt : it.second) {
      std::string service_type = _demangle_service_type_only(itt);
      if (service_type.length()) {
        services[service_name].insert(service_type);
      }
    }
  }
}

void
print_discovery_logging(
  const std::string & prefix,
  const std::string & name,
  const std::string & type,
  CustomDataReaderListener::EndPointType end_point_type)
{
  // filter builtin OpenSplice topics
  if (
    name.compare(0, 4, "DCPS") != 0 &&
    name.compare(0, 2, "d_") != 0 &&
    name.compare(0, 2, "q_") != 0 &&
    name.compare("CMParticipant") != 0)
  {
    std::string msg = prefix +
      ((end_point_type == CustomDataReaderListener::PublisherEP) ? "P" : "S") +
      ": " + name + " <" + type + ">";
    printf("%s\n", msg.c_str());
  }
}

void CustomDataReaderListener::add_information(
  const DDS::InstanceHandle_t & participant,
  const DDS::InstanceHandle_t & topic,
  const std::string & topic_name,
  const std::string & topic_type,
  const EndPointType endpoint_type)
{
  topic_cache.addTopic(participant, topic, topic_name, topic_type);
  if (print_discovery_logging_) {
    print_discovery_logging("+", topic_name, topic_type, endpoint_type);
  }
}

void CustomDataReaderListener::remove_information(
  const DDS::InstanceHandle_t & topic,
  const EndPointType endpoint_type)
{
  if (print_discovery_logging_) {
    TopicCache<DDS::InstanceHandle_t>::TopicInfo topic_info;
    if (topic_cache.getTopic(topic, topic_info)) {
      print_discovery_logging("-", topic_info.name, topic_info.type, endpoint_type);
    }
  }
  topic_cache.removeTopic(topic);
}

CustomPublisherListener::CustomPublisherListener(rmw_guard_condition_t * graph_guard_condition)
: graph_guard_condition_(graph_guard_condition)
{
}

void
CustomPublisherListener::on_data_available(DDS::DataReader * reader)
{
  std::lock_guard<std::mutex> lock(mutex_);
  DDS::PublicationBuiltinTopicDataDataReader * builtin_reader =
    DDS::PublicationBuiltinTopicDataDataReader::_narrow(reader);

  DDS::PublicationBuiltinTopicDataSeq data_seq;

  DDS::SampleInfoSeq info_seq;
  DDS::ReturnCode_t retcode = builtin_reader->take(
    data_seq, info_seq, DDS::LENGTH_UNLIMITED,
    DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);

  if (retcode == DDS::RETCODE_NO_DATA) {
    return;
  }
  if (retcode != DDS::RETCODE_OK) {
    fprintf(stderr, "failed to access data from the built-in reader\n");
    return;
  }

  for (DDS::ULong i = 0; i < data_seq.length(); ++i) {
    std::string topic_name = "";
    DDS::InstanceHandle_t topic = DDS_BuiltinTopicKey_to_InstanceHandle(data_seq[i].key);
    if (info_seq[i].valid_data && info_seq[i].instance_state == DDS::ALIVE_INSTANCE_STATE) {
      topic_name = data_seq[i].topic_name.in();
      DDS::InstanceHandle_t participant = DDS_BuiltinTopicKey_to_InstanceHandle(
        data_seq[i].participant_key);
      add_information(participant, topic, topic_name,
        data_seq[i].type_name.in(), PublisherEP);
    } else {
      remove_information(topic, PublisherEP);
    }
  }

  if (data_seq.length() > 0) {
    rmw_ret_t ret = rmw_trigger_guard_condition(graph_guard_condition_);
    if (ret != RMW_RET_OK) {
      fprintf(stderr, "failed to trigger graph guard condition: %s\n", rmw_get_error_string().str);
    }
  }

  builtin_reader->return_loan(data_seq, info_seq);
}

CustomSubscriberListener::CustomSubscriberListener(rmw_guard_condition_t * graph_guard_condition)
: graph_guard_condition_(graph_guard_condition)
{
}

void
CustomSubscriberListener::on_data_available(DDS::DataReader * reader)
{
  std::lock_guard<std::mutex> lock(mutex_);
  DDS::SubscriptionBuiltinTopicDataDataReader * builtin_reader =
    DDS::SubscriptionBuiltinTopicDataDataReader::_narrow(reader);

  DDS::SubscriptionBuiltinTopicDataSeq data_seq;
  DDS::SampleInfoSeq info_seq;

  DDS::ReturnCode_t retcode = builtin_reader->take(
    data_seq, info_seq, DDS::LENGTH_UNLIMITED,
    DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);

  if (retcode == DDS::RETCODE_NO_DATA) {
    return;
  }
  if (retcode != DDS::RETCODE_OK) {
    fprintf(stderr, "failed to access data from the built-in reader\n");
    return;
  }

  for (DDS::ULong i = 0; i < data_seq.length(); ++i) {
    DDS::InstanceHandle_t topic = DDS_BuiltinTopicKey_to_InstanceHandle(data_seq[i].key);
    if (info_seq[i].valid_data) {
      std::string topic_name = "";
      DDS::InstanceHandle_t participant = DDS_BuiltinTopicKey_to_InstanceHandle(
        data_seq[i].participant_key);
      if (info_seq[i].instance_state == DDS::ALIVE_INSTANCE_STATE) {
        topic_name = data_seq[i].topic_name.in();
        add_information(participant, topic, topic_name,
          data_seq[i].type_name.in(), SubscriberEP);
      } else {
        remove_information(topic, SubscriberEP);
      }
    } else {
      remove_information(topic, SubscriberEP);
    }
  }

  if (data_seq.length() > 0) {
    rmw_ret_t ret = rmw_trigger_guard_condition(graph_guard_condition_);
    if (ret != RMW_RET_OK) {
      fprintf(stderr, "failed to trigger graph guard condition: %s\n", rmw_get_error_string().str);
    }
  }

  builtin_reader->return_loan(data_seq, info_seq);
}

void OpenSplicePublisherListener::on_publication_matched(
  DDS::DataWriter_ptr writer,
  const DDS::PublicationMatchedStatus & status)
{
  (void) writer;
  current_count_ = status.current_count;
}

size_t OpenSplicePublisherListener::current_count() const
{
  return current_count_;
}

void OpenSpliceSubscriberListener::on_subscription_matched(
  DDS::DataReader_ptr reader,
  const DDS::SubscriptionMatchedStatus & status)
{
  (void) reader;
  current_count_ = status.current_count;
}

size_t OpenSpliceSubscriberListener::current_count() const
{
  return current_count_;
}

rmw_ret_t OpenSpliceStaticPublisherInfo::get_status(
  const DDS::StatusMask mask,
  void * event)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);
  switch (mask) {
    case DDS::LIVELINESS_LOST_STATUS:
      {
        DDS::LivelinessLostStatus liveliness_lost;
        DDS::ReturnCode_t dds_return_code =
          topic_writer->get_liveliness_lost_status(liveliness_lost);
        rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
        if (from_dds != RMW_RET_OK) {
          return from_dds;
        }

        auto rmw_liveliness_lost =
          static_cast<rmw_liveliness_lost_status_t *>(event);
        rmw_liveliness_lost->total_count = liveliness_lost.total_count;
        rmw_liveliness_lost->total_count_change = liveliness_lost.total_count_change;
        break;
      }
    case DDS::OFFERED_DEADLINE_MISSED_STATUS:
      {
        DDS::OfferedDeadlineMissedStatus offered_deadline_missed;
        DDS::ReturnCode_t dds_return_code =
          topic_writer->get_offered_deadline_missed_status(offered_deadline_missed);
        rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
        if (from_dds != RMW_RET_OK) {
          return from_dds;
        }

        auto rmw_offered_deadline_missed =
          static_cast<rmw_offered_deadline_missed_status_t *>(event);
        rmw_offered_deadline_missed->total_count = offered_deadline_missed.total_count;
        rmw_offered_deadline_missed->total_count_change =
          offered_deadline_missed.total_count_change;
        break;
      }
    default:
      return RMW_RET_UNSUPPORTED;
  }
  return RMW_RET_OK;
}

DDS::Entity * OpenSpliceStaticPublisherInfo::get_entity()
{
  return topic_writer;
}

rmw_ret_t OpenSpliceStaticSubscriberInfo::get_status(
  const DDS::StatusMask mask,
  void * event)
{
  switch (mask) {
    case DDS::LIVELINESS_CHANGED_STATUS:
      {
        DDS::LivelinessChangedStatus liveliness_changed;
        DDS::ReturnCode_t dds_return_code =
          topic_reader->get_liveliness_changed_status(liveliness_changed);
        rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
        if (from_dds != RMW_RET_OK) {
          return from_dds;
        }

        auto rmw_liveliness_changed_status =
          static_cast<rmw_liveliness_changed_status_t *>(event);
        rmw_liveliness_changed_status->alive_count = liveliness_changed.alive_count;
        rmw_liveliness_changed_status->not_alive_count = liveliness_changed.not_alive_count;
        rmw_liveliness_changed_status->alive_count_change = liveliness_changed.alive_count_change;
        rmw_liveliness_changed_status->not_alive_count_change =
          liveliness_changed.not_alive_count_change;
        break;
      }
    case DDS::REQUESTED_DEADLINE_MISSED_STATUS:
      {
        DDS::RequestedDeadlineMissedStatus requested_deadline_missed;
        DDS::ReturnCode_t dds_return_code =
          topic_reader->get_requested_deadline_missed_status(requested_deadline_missed);
        rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
        if (from_dds != RMW_RET_OK) {
          return from_dds;
        }

        auto rmw_requested_deadline_missed_status =
          static_cast<rmw_requested_deadline_missed_status_t *>(event);
        rmw_requested_deadline_missed_status->total_count = requested_deadline_missed.total_count;
        rmw_requested_deadline_missed_status->total_count_change =
          requested_deadline_missed.total_count_change;
        break;
      }
    default:
      return RMW_RET_UNSUPPORTED;
  }
  return RMW_RET_OK;
}

DDS::Entity * OpenSpliceStaticSubscriberInfo::get_entity()
{
  return topic_reader;
}
