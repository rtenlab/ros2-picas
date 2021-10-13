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

#ifndef RMW_CONNEXT_SHARED_CPP__TOPIC_CACHE_HPP_
#define RMW_CONNEXT_SHARED_CPP__TOPIC_CACHE_HPP_

#include <algorithm>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include "rcutils/logging_macros.h"

#include "rmw_connext_shared_cpp/guid_helper.hpp"

/**
 * Topics to types.
 */
using TopicsTypes = std::map<std::string, std::set<std::string>>;

/**
 * Topic cache data structure.
 * Manages relationships between participants and topics.
 */
template<typename GUID_t>
class TopicCache
{
public:
  /**
   * Relevant Topic information for building relationship cache.
   */
  struct TopicInfo
  {
    std::string topic_name;
    std::string topic_type;
    GUID_t participant_guid;
    GUID_t endpoint_guid;
    rmw_qos_profile_t qos_profile;
  };

  using ParticipantToTopicEndpointGuids = std::map<GUID_t, std::multiset<GUID_t>>;
  using TopicEndpointGuidToInfo = std::map<GUID_t, TopicInfo>;

  /**
   * \return a map of topic name to the vector of topic types used.
   */
  const TopicEndpointGuidToInfo & get_topic_endpoint_guid_to_info() const
  {
    return endpoint_guid_to_info_;
  }

  /**
   * \return a map of participant guid to the vector of topic names used.
   */
  const ParticipantToTopicEndpointGuids & get_participant_to_topic_endpoint_guids_map() const
  {
    return participant_to_endpoint_guids_;
  }

  /**
   * Add a topic based on discovery.
   *
   * \param participant_guid
   * \param topic_name
   * \param type_name
   * \return true if a change has been recorded
   */
  bool add_topic(
    const GUID_t & participant_guid,
    const GUID_t & endpoint_guid,
    const std::string & topic_name,
    const std::string & type_name,
    const rmw_qos_profile_t & qos_profile)
  {
    initialize_participant_map(participant_to_endpoint_guids_, participant_guid);
    if (
      rcutils_logging_logger_is_enabled_for(
        "rmw_connext_shared_cpp", RCUTILS_LOG_SEVERITY_DEBUG))
    {
      std::stringstream guid_stream;
      guid_stream << participant_guid;
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_connext_shared_cpp",
        "Adding topic '%s' with type '%s' for node '%s'",
        topic_name.c_str(), type_name.c_str(), guid_stream.str().c_str());
    }
    auto topic_endpoint_info_it = endpoint_guid_to_info_.find(endpoint_guid);
    if (topic_endpoint_info_it != endpoint_guid_to_info_.end()) {
      RCUTILS_LOG_WARN_NAMED(
        "rmw_connext_shared_cpp",
        "unique topic attempted to be added twice, ignoring");
      return false;
    }
    endpoint_guid_to_info_[endpoint_guid] =
      TopicInfo {topic_name, type_name, participant_guid, endpoint_guid, qos_profile};
    participant_to_endpoint_guids_[participant_guid].insert(endpoint_guid);
    return true;
  }

  /**
   * Remove a topic based on discovery.
   *
   * \param guid
   * \return true if a change has been recorded
   */
  bool remove_topic(const GUID_t & endpoint_guid)
  {
    auto topic_endpoint_info_it = endpoint_guid_to_info_.find(endpoint_guid);
    if (topic_endpoint_info_it == endpoint_guid_to_info_.end()) {
      RCUTILS_LOG_WARN_NAMED(
        "rmw_connext_shared_cpp",
        "unexpected topic removal.");
      return false;
    }

    const std::string & topic_name = topic_endpoint_info_it->second.topic_name;
    const std::string & type_name = topic_endpoint_info_it->second.topic_type;

    auto participant_guid = topic_endpoint_info_it->second.participant_guid;
    auto participant_to_topic_guid = participant_to_endpoint_guids_.find(participant_guid);
    if (participant_to_topic_guid == participant_to_endpoint_guids_.end()) {
      RCUTILS_LOG_WARN_NAMED(
        "rmw_connext_shared_cpp",
        "Unable to remove topic,"
        " participant guid does not exist for topic name '%s' with type '%s'",
        topic_name.c_str(), type_name.c_str());
      return false;
    }
    auto topic_guid_to_remove = participant_to_topic_guid->second.find(endpoint_guid);
    if (topic_guid_to_remove == participant_to_topic_guid->second.end()) {
      RCUTILS_LOG_WARN_NAMED(
        "rmw_connext_shared_cpp",
        "Unable to remove topic, "
        "topic guid does not exist in participant guid: topic name '%s' with type '%s'",
        topic_name.c_str(), type_name.c_str());
      return false;
    }

    endpoint_guid_to_info_.erase(topic_endpoint_info_it);
    participant_to_topic_guid->second.erase(topic_guid_to_remove);
    if (participant_to_endpoint_guids_.empty()) {
      participant_to_endpoint_guids_.erase(participant_to_topic_guid);
    }
    return true;
  }

  /**
   * Get topic types by guid.
   *
   * \param participant_guid to find topic types
   * \return topic types corresponding to that guid
   */
  TopicsTypes get_topic_types_by_guid(const GUID_t & participant_guid)
  {
    TopicsTypes topics_types;
    const auto participant_to_topic_guids =
      participant_to_endpoint_guids_.find(participant_guid);
    if (participant_to_topic_guids == participant_to_endpoint_guids_.end()) {
      return topics_types;
    }

    for (auto & endpoint_guid : participant_to_topic_guids->second) {
      auto topic_endpoint_info = endpoint_guid_to_info_.find(endpoint_guid);
      if (topic_endpoint_info == endpoint_guid_to_info_.end()) {
        continue;
      }
      const std::string & topic_name = topic_endpoint_info->second.topic_name;
      auto topic_entry = topics_types.find(topic_name);
      if (topic_entry == topics_types.end()) {
        topics_types[topic_name] = std::set<std::string>();
      }
      topics_types[topic_name].insert(topic_endpoint_info->second.topic_type);
    }
    return topics_types;
  }

private:
  /**
   * Helper function to initialize the set inside a participant map.
   *
   * \param map
   * \param participant_guid
   */
  void initialize_participant_map(
    ParticipantToTopicEndpointGuids & map,
    const GUID_t & participant_guid)
  {
    if (map.find(participant_guid) == map.end()) {
      map[participant_guid] = std::multiset<GUID_t>();
    }
  }

  /**
   * Map of topic guid to topic info.
   * Topics here are represented as one to many, DDS XTypes 1.2
   * specifies application code 'generally' uses a 1-1 relationship.
   * However, generic services such as logger and monitor, can discover
   * multiple types on the same topic.
   *
   */
  TopicEndpointGuidToInfo endpoint_guid_to_info_;

  /**
   * Map of participant GUIDS to a set of topic-type.
   */
  ParticipantToTopicEndpointGuids participant_to_endpoint_guids_;
};

#endif  // RMW_CONNEXT_SHARED_CPP__TOPIC_CACHE_HPP_
