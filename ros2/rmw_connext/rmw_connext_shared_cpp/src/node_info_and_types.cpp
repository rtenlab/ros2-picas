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

#include <string.h>

#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/error_handling.h"
#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/allocators.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"

#include "rmw_connext_shared_cpp/node_info_and_types.hpp"
#include "rmw_connext_shared_cpp/types.hpp"
#include "rmw_connext_shared_cpp/names_and_types_helpers.hpp"
#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw_connext_shared_cpp/guid_helper.hpp"

/**
 * Check to see if a node name and namespace match the user data QoS policy
 * of a node.
 *
 * @param user_data_qos to inspect
 * @param node_name to match
 * @param node_namespace to match
 * @return true if match
 */
bool
__is_node_match(
  DDS::UserDataQosPolicy & user_data_qos,
  const char * node_name,
  const char * node_namespace)
{
  uint8_t * buf = user_data_qos.value.get_contiguous_buffer();
  if (buf) {
    std::vector<uint8_t> kv(buf, buf + user_data_qos.value.length());
    auto map = rmw::impl::cpp::parse_key_value(kv);
    auto name_found = map.find("name");
    auto ns_found = map.find("namespace");

    if (name_found != map.end() && ns_found != map.end()) {
      std::string name(name_found->second.begin(), name_found->second.end());
      std::string ns(ns_found->second.begin(), ns_found->second.end());
      return (name == node_name) && (ns == node_namespace);
    }
  }
  return false;
}

/**
 * Get a DDS GUID key for the discovered participant which matches the
 * node_name and node_namepace supplied.
 *
 * @param node_info to discover nodes
 * @param node_name to match
 * @param node_namespace to match
 * @param key [out] guid key that matches the node name and namespace
 *
 * @return RMW_RET_OK if success, ERROR otherwise
 */
rmw_ret_t
__get_key(
  ConnextNodeInfo * node_info,
  const char * node_name,
  const char * node_namespace,
  DDS::GUID_t & key)
{
  auto participant = node_info->participant;
  RMW_CHECK_FOR_NULL_WITH_MSG(participant, "participant handle is null", return RMW_RET_ERROR);

  DDS::DomainParticipantQos dpqos;
  auto dds_ret = participant->get_qos(dpqos);
  if (dds_ret == DDS::RETCODE_OK && __is_node_match(dpqos.user_data, node_name, node_namespace)) {
    DDS_InstanceHandle_to_GUID(&key, participant->get_instance_handle());
    return RMW_RET_OK;
  }

  DDS::InstanceHandleSeq handles;
  if (participant->get_discovered_participants(handles) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("unable to fetch discovered participants.");
    return RMW_RET_ERROR;
  }

  for (DDS::Long i = 0; i < handles.length(); ++i) {
    DDS::ParticipantBuiltinTopicData pbtd;
    auto dds_ret = participant->get_discovered_participant_data(pbtd, handles[i]);
    if (dds_ret == DDS::RETCODE_OK) {
      uint8_t * buf = pbtd.user_data.value.get_contiguous_buffer();
      if (buf) {
        std::vector<uint8_t> kv(buf, buf + pbtd.user_data.value.length());
        auto map = rmw::impl::cpp::parse_key_value(kv);
        auto name_found = map.find("name");
        auto ns_found = map.find("namespace");

        if (name_found != map.end() && ns_found != map.end()) {
          std::string name(name_found->second.begin(), name_found->second.end());
          std::string ns(ns_found->second.begin(), ns_found->second.end());
          if ((name == node_name) && (ns == node_namespace)) {
            DDS_BuiltinTopicKey_to_GUID(&key, pbtd.key);
            return RMW_RET_OK;
          }
        }
      }
    } else {
      RMW_SET_ERROR_MSG("unable to fetch discovered participants data.");
      return RMW_RET_ERROR;
    }
  }
  RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
    "Node name not found: ns='%s', name='%s",
    node_namespace,
    node_name
  );
  return RMW_RET_NODE_NAME_NON_EXISTENT;
}

rmw_ret_t
validate_names_and_namespace(
  const char * node_name,
  const char * node_namespace)
{
  if (!node_name) {
    RMW_SET_ERROR_MSG("null node name");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!node_namespace) {
    RMW_SET_ERROR_MSG("null node namespace");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

rmw_ret_t
get_subscriber_names_and_types_by_node(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  if (!node) {
    RMW_SET_ERROR_MSG("null node handle");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (node->implementation_identifier != implementation_identifier) {
    RMW_SET_ERROR_MSG("node handle is not from this rmw implementation");
    return RMW_RET_ERROR;
  }

  rmw_ret_t ret = rmw_names_and_types_check_zero(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_namespace(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }

  DDS::GUID_t key;
  auto get_guid_err = __get_key(node_info, node_name, node_namespace, key);
  if (get_guid_err != RMW_RET_OK) {
    return get_guid_err;
  }

  // combine publisher and subscriber information
  std::map<std::string, std::set<std::string>> topics;
  node_info->subscriber_listener->fill_topic_names_and_types_by_guid(no_demangle, topics, key);

  return copy_topics_names_and_types(topics, allocator, no_demangle, topic_names_and_types);
}

rmw_ret_t
get_publisher_names_and_types_by_node(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  if (!node) {
    RMW_SET_ERROR_MSG("null node handle");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (node->implementation_identifier != implementation_identifier) {
    RMW_SET_ERROR_MSG("node handle is not from this rmw implementation");
    return RMW_RET_ERROR;
  }

  rmw_ret_t ret = rmw_names_and_types_check_zero(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_namespace(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }

  DDS::GUID_t key;
  auto get_guid_err = __get_key(node_info, node_name, node_namespace, key);
  if (get_guid_err != RMW_RET_OK) {
    return get_guid_err;
  }

  // combine publisher and subscriber information
  std::map<std::string, std::set<std::string>> topics;
  node_info->publisher_listener->fill_topic_names_and_types_by_guid(no_demangle, topics, key);

  return copy_topics_names_and_types(topics, allocator, no_demangle, topic_names_and_types);
}

static
rmw_ret_t
__get_service_names_and_types_by_node(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types,
  const char * suffix)
{
  if (!node) {
    RMW_SET_ERROR_MSG("null node handle");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (node->implementation_identifier != implementation_identifier) {
    RMW_SET_ERROR_MSG("node handle is not from this rmw implementation");
    return RMW_RET_ERROR;
  }

  rmw_ret_t ret = rmw_names_and_types_check_zero(service_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }

  DDS::GUID_t key;
  auto get_guid_err = __get_key(node_info, node_name, node_namespace, key);
  if (get_guid_err != RMW_RET_OK) {
    return get_guid_err;
  }

  // combine publisher and subscriber information
  std::map<std::string, std::set<std::string>> services;
  node_info->subscriber_listener->fill_service_names_and_types_by_guid(services, key, suffix);

  rmw_ret_t rmw_ret =
    copy_services_to_names_and_types(services, allocator, service_names_and_types);
  if (rmw_ret != RMW_RET_OK) {
    return rmw_ret;
  }

  return RMW_RET_OK;
}

rmw_ret_t
get_service_names_and_types_by_node(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  return __get_service_names_and_types_by_node(
    implementation_identifier,
    node,
    allocator,
    node_name,
    node_namespace,
    service_names_and_types,
    "Request");
}

rmw_ret_t
get_client_names_and_types_by_node(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  return __get_service_names_and_types_by_node(
    implementation_identifier,
    node,
    allocator,
    node_name,
    node_namespace,
    service_names_and_types,
    "Reply");
}
