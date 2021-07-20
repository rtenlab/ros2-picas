// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "rmw/allocators.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/sanity_checks.h"

#include "rcutils/logging_macros.h"

#include "names_and_types_helpers.hpp"
#include "types.hpp"

extern "C"
{
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
  uint8_t * buf = user_data_qos.value.get_buffer(false);
  if (buf) {
    std::vector<uint8_t> kv(buf, buf + user_data_qos.value.length());
    auto map = rmw::impl::cpp::parse_key_value(kv);
    auto name_found = map.find("name");
    auto ns_found = map.find("namespace");

    if (name_found != map.end() && ns_found != map.end()) {
      std::string name(name_found->second.begin(), name_found->second.end());
      std::string ns(ns_found->second.begin(), ns_found->second.end());
      return strcmp(node_name, name.c_str()) == 0 && strcmp(node_namespace, ns.c_str()) == 0;
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
 * @param key [out] key (an InstanceHandle) that matches the node name and namespace
 *
 * @return RMW_RET_OK if success, ERROR otherwise
 */
rmw_ret_t
__get_key(
  OpenSpliceStaticNodeInfo * node_info,
  const char * node_name,
  const char * node_namespace,
  DDS::InstanceHandle_t & key)
{
  DDS::DataReader_var loopup_datareader;
  DDS::ReturnCode_t status;
  auto participant = node_info->participant;
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return RMW_RET_ERROR;
  }

  DDS::DomainParticipantQos dpqos;
  auto dds_ret = participant->get_qos(dpqos);
  // @todo: ross-desmond implement self discovery
  if (dds_ret == DDS::RETCODE_OK && __is_node_match(dpqos.user_data, node_name, node_namespace)) {
    key = node_info->participant->get_instance_handle();
    return RMW_RET_OK;
  }

  // Use opensplice get_discovered_participants will get a list of participants including
  // alive or disposed. However, we don't want the disposed participants. So we use a datareader
  // to read instance alive samples from the built-in topic: "DCPSParticipant", to retrive the
  // alive node info from userdata.
  // Reference: https://github.com/ADLINK-IST/opensplice/issues/79#issuecomment-456367434

  auto builtinSubscriber = participant->get_builtin_subscriber();
  loopup_datareader = builtinSubscriber->lookup_datareader("DCPSParticipant");
  auto participantReader =
    DDS::ParticipantBuiltinTopicDataDataReader::_narrow(loopup_datareader.in());

  DDS::ParticipantBuiltinTopicDataSeq data;
  DDS::SampleInfoSeq info;

  DDS::Duration_t wait_for_historical_data_timeout = {1, 0};

  // Make sure all historical data is delivered in the DataReader
  participantReader->wait_for_historical_data(wait_for_historical_data_timeout);

  // Take samples from DCPSParticipant topic
  status = participantReader->read(data, info, DDS::LENGTH_UNLIMITED,
      DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);

  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("unable to discover participants.");
    return RMW_RET_ERROR;
  }

  bool node_found = false;
  for (DDS::ULong i = 0; i < data.length(); ++i) {
    if (info[i].instance_state == DDS::ALIVE_INSTANCE_STATE) {
      uint8_t * buf = data[i].user_data.value.get_buffer(false);
      if (buf == nullptr) {
        continue;
      }
      std::vector<uint8_t> kv(buf, buf + data[i].user_data.value.length());
      auto map = rmw::impl::cpp::parse_key_value(kv);
      auto name_found = map.find("name");
      auto ns_found = map.find("namespace");

      if (name_found != map.end() && ns_found != map.end()) {
        std::string name(name_found->second.begin(), name_found->second.end());
        std::string ns(ns_found->second.begin(), ns_found->second.end());
        RCUTILS_LOG_DEBUG_NAMED(
          "rmw_opensplice_cpp",
          "Found node %s", name.c_str());
        if (strcmp(node_name, name.c_str()) == 0 &&
          strcmp(node_namespace, ns.c_str()) == 0)
        {
          key = DDS_BuiltinTopicKey_to_InstanceHandle(data[i].key);
          node_found = true;
          break;
        }
      }
    }
  }

  participantReader->return_loan(data, info);

  if (node_found == true) {
    return RMW_RET_OK;
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
rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  rmw_ret_t ret = validate_node(node, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = rmw_names_and_types_check_zero(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_namespace(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  DDS::InstanceHandle_t key;
  auto get_guid_err = __get_key(node_info, node_name, node_namespace, key);
  if (get_guid_err != RMW_RET_OK) {
    return get_guid_err;
  }
  // combine publisher and subscriber information
  std::map<std::string, std::set<std::string>> topics;
  node_info->subscriber_listener->fill_topic_names_and_types_by_participant(no_demangle, topics,
    key);

  rmw_ret_t rmw_ret;
  rmw_ret = copy_topics_names_and_types(topics, allocator, no_demangle, topic_names_and_types);
  if (rmw_ret != RMW_RET_OK) {
    return rmw_ret;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  rmw_ret_t ret = validate_node(node, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = rmw_names_and_types_check_zero(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_namespace(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  DDS::InstanceHandle_t key;
  auto get_guid_err = __get_key(node_info, node_name, node_namespace, key);
  if (get_guid_err != RMW_RET_OK) {
    return get_guid_err;
  }

  // combine publisher and subscriber information
  std::map<std::string, std::set<std::string>> topics;
  node_info->publisher_listener->fill_topic_names_and_types_by_participant(no_demangle, topics,
    key);

  rmw_ret_t rmw_ret;
  rmw_ret = copy_topics_names_and_types(topics, allocator, no_demangle, topic_names_and_types);
  if (rmw_ret != RMW_RET_OK) {
    return rmw_ret;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  rmw_ret_t ret = validate_node(node, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = rmw_names_and_types_check_zero(service_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_namespace(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  DDS::InstanceHandle_t key;
  auto get_guid_err = __get_key(node_info, node_name, node_namespace, key);
  if (get_guid_err != RMW_RET_OK) {
    return get_guid_err;
  }

  std::map<std::string, std::set<std::string>> services;
  node_info->subscriber_listener->fill_service_names_and_types_by_participant(
    services,
    key,
    "Request");

  rmw_ret_t rmw_ret;
  rmw_ret = copy_services_to_names_and_types(services, allocator, service_names_and_types);
  if (rmw_ret != RMW_RET_OK) {
    return rmw_ret;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  rmw_ret_t ret = validate_node(node, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = rmw_names_and_types_check_zero(service_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_namespace(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  DDS::InstanceHandle_t key;
  auto get_guid_err = __get_key(node_info, node_name, node_namespace, key);
  if (get_guid_err != RMW_RET_OK) {
    return get_guid_err;
  }

  std::map<std::string, std::set<std::string>> services;
  node_info->subscriber_listener->fill_service_names_and_types_by_participant(
    services,
    key,
    "Reply");

  rmw_ret_t rmw_ret;
  rmw_ret = copy_services_to_names_and_types(services, allocator, service_names_and_types);
  if (rmw_ret != RMW_RET_OK) {
    return rmw_ret;
  }
  return RMW_RET_OK;
}
}  // extern "C"
