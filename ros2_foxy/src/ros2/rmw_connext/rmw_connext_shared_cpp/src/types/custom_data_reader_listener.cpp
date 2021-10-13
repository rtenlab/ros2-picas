// Copyright 2015-2017 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "rmw/error_handling.h"

#include "rmw_connext_shared_cpp/trigger_guard_condition.hpp"
#include "rmw_connext_shared_cpp/namespace_prefix.hpp"
#include "rmw_connext_shared_cpp/demangle.hpp"
#include "rmw_connext_shared_cpp/guid_helper.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

// Uncomment this to get extra console output about discovery.
// #define DISCOVERY_DEBUG_LOGGING 1

bool CustomDataReaderListener::add_information(
  const DDS::GUID_t & participant_guid,
  const DDS::GUID_t & guid,
  const std::string & topic_name,
  const std::string & type_name,
  const rmw_qos_profile_t & qos_profile,
  EntityType entity_type)
{
  (void)entity_type;
  std::lock_guard<std::mutex> lock(mutex_);

  // store topic name and type name
  bool success = topic_cache.add_topic(participant_guid, guid, topic_name, type_name, qos_profile);

#ifdef DISCOVERY_DEBUG_LOGGING
  std::stringstream ss;
  ss << participant_guid << ":" << guid;
  printf(
    "+%s %s %s <%s>\n",
    entity_type == EntityType::Publisher ? "P" : "S",
    ss.str().c_str(),
    topic_name.c_str(),
    type_name.c_str());
#endif

  return success;
}

bool CustomDataReaderListener::remove_information(
  const DDS::GUID_t & guid,
  EntityType entity_type)
{
  (void)entity_type;
  std::lock_guard<std::mutex> lock(mutex_);

  // remove entries
  bool success = topic_cache.remove_topic(guid);

#ifdef DISCOVERY_DEBUG_LOGGING
  std::stringstream ss;
  ss << guid;
  printf(
    "-%s %s\n",
    entity_type == EntityType::Publisher ? "P" : "S",
    ss.str().c_str());
#endif

  return success;
}

bool CustomDataReaderListener::add_information(
  const DDS::InstanceHandle_t & participant_instance_handle,
  const DDS::InstanceHandle_t & instance_handle,
  const std::string & topic_name,
  const std::string & type_name,
  const rmw_qos_profile_t & qos_profile,
  EntityType entity_type)
{
  DDS::GUID_t guid, participant_guid;
  DDS_InstanceHandle_to_GUID(&guid, instance_handle);
  DDS_InstanceHandle_to_GUID(&participant_guid, participant_instance_handle);
  return add_information(participant_guid, guid, topic_name, type_name, qos_profile, entity_type);
}

bool CustomDataReaderListener::remove_information(
  const DDS::InstanceHandle_t & instance_handle,
  EntityType entity_type)
{
  DDS::GUID_t guid;
  DDS_InstanceHandle_to_GUID(&guid, instance_handle);
  return remove_information(guid, entity_type);
}

bool CustomDataReaderListener::trigger_graph_guard_condition()
{
#ifdef DISCOVERY_DEBUG_LOGGING
  printf("graph guard condition triggered...\n");
#endif
  rmw_ret_t ret = trigger_guard_condition(implementation_identifier_, graph_guard_condition_);
  if (ret != RMW_RET_OK) {
    fprintf(stderr, "failed to trigger graph guard condition: %s\n", rmw_get_error_string().str);
    return false;
  }
  return true;
}

size_t CustomDataReaderListener::count_topic(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto count = std::count_if(
    topic_cache.get_topic_endpoint_guid_to_info().begin(),
    topic_cache.get_topic_endpoint_guid_to_info().end(),
    [&](auto tnt) -> bool {
      auto fqdn = _demangle_if_ros_topic(tnt.second.topic_name);
      return fqdn == topic_name;
    });
  return (size_t) count;
}

void CustomDataReaderListener::fill_topic_endpoint_infos(
  const std::string & topic_name,
  bool no_mangle,
  std::vector<const DDSTopicEndpointInfo *> & topic_endpoint_infos)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & key_val : topic_cache.get_topic_endpoint_guid_to_info()) {
    auto fqdn = no_mangle ? key_val.second.topic_name : _demangle_if_ros_topic(
      key_val.second.topic_name);
    if (fqdn == topic_name) {
      topic_endpoint_infos.push_back(&key_val.second);
    }
  }
}

void CustomDataReaderListener::fill_topic_names_and_types(
  bool no_demangle,
  std::map<std::string, std::set<std::string>> & topic_names_to_types)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it : topic_cache.get_topic_endpoint_guid_to_info()) {
    if (!no_demangle &&
      (_get_ros_prefix_if_exists(it.second.topic_name) != ros_topic_prefix))
    {
      continue;
    }
    topic_names_to_types[it.second.topic_name].insert(it.second.topic_type);
  }
}

void
CustomDataReaderListener::fill_service_names_and_types(
  std::map<std::string, std::set<std::string>> & services)
{
  for (auto it : topic_cache.get_topic_endpoint_guid_to_info()) {
    std::string service_name = _demangle_service_from_topic(it.second.topic_name);
    if (service_name.empty()) {
      // not a service
      continue;
    }
    std::string service_type = _demangle_service_type_only(it.second.topic_type);
    if (!service_type.empty()) {
      services[service_name].insert(service_type);
    }
  }
}

void CustomDataReaderListener::fill_topic_names_and_types_by_guid(
  bool no_demangle,
  std::map<std::string, std::set<std::string>> & topic_names_to_types_by_guid,
  DDS::GUID_t & participant_guid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & map = topic_cache.get_topic_types_by_guid(participant_guid);
  if (map.empty()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_connext_shared_cpp",
      "No topics for participant_guid");
    return;
  }
  for (auto & it : map) {
    if (!no_demangle && (_get_ros_prefix_if_exists(it.first) !=
      ros_topic_prefix))
    {
      continue;
    }
    topic_names_to_types_by_guid[it.first].insert(it.second.begin(), it.second.end());
  }
}

void CustomDataReaderListener::fill_service_names_and_types_by_guid(
  std::map<std::string, std::set<std::string>> & services,
  DDS::GUID_t & participant_guid,
  const std::string & suffix)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & map = topic_cache.get_topic_types_by_guid(participant_guid);
  if (map.empty()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_connext_shared_cpp",
      "No services for participant_guid");
    return;
  }
  for (auto & it : map) {
    std::string service_name = _demangle_service_from_topic(it.first);
    if (service_name.empty()) {
      // not a service
      continue;
    }
    // Check if the topic suffix matches and is at the end of the name
    const std::string & topic_name = it.first;
    auto suffix_position = topic_name.rfind(suffix);
    if (suffix_position == std::string::npos) {
      continue;
    }

    for (auto & itt : it.second) {
      std::string service_type = _demangle_service_type_only(itt);
      if (!service_type.empty()) {
        services[service_name].insert(service_type);
      }
    }
  }
}
