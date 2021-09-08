// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <map>
#include <vector>

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/impl/cpp/macros.hpp"

#include "rmw_connext_shared_cpp/demangle.hpp"
#include "rmw_connext_shared_cpp/namespace_prefix.hpp"
#include "rmw_connext_shared_cpp/topic_endpoint_info.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

struct ParticipantNameInfo
{
  std::string name;
  std::string namespace_;
};

rmw_ret_t
_validate_arguments(
  const char * identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  rmw_topic_endpoint_info_array_t * participants_info)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_topic_endpoint_info_array_check_zero(participants_info)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

std::vector<std::string>
_get_topic_fqdns(const char * topic_name, bool no_mangle)
{
  std::vector<std::string> topic_fqdns;
  topic_fqdns.push_back(topic_name);
  if (!no_mangle) {
    auto ros_prefixes = _get_all_ros_prefixes();
    // Build the list of all possible topic FQDN
    std::for_each(
      ros_prefixes.begin(), ros_prefixes.end(),
      [&topic_fqdns, &topic_name](const std::string & prefix) {
        topic_fqdns.push_back(prefix + topic_name);
      });
  }
  return topic_fqdns;
}

rmw_ret_t
_set_rmw_topic_endpoint_info(
  rmw_topic_endpoint_info_t * topic_endpoint_info,
  const DDSTopicEndpointInfo * dds_topic_endpoint_info,
  const std::map<DDS::GUID_t, ParticipantNameInfo> & participant_guid_to_name,
  bool no_mangle,
  bool is_publisher,
  rcutils_allocator_t * allocator)
{
  static_assert(
    sizeof(DDS::GUID_t) <= RMW_GID_STORAGE_SIZE,
    "RMW_GID_STORAGE_SIZE insufficient to store the rmw_connext_cpp GID implementation."
  );

  rmw_ret_t ret;

  // set endpoint type
  if (is_publisher) {
    ret = rmw_topic_endpoint_info_set_endpoint_type(topic_endpoint_info, RMW_ENDPOINT_PUBLISHER);
  } else {
    ret = rmw_topic_endpoint_info_set_endpoint_type(topic_endpoint_info, RMW_ENDPOINT_SUBSCRIPTION);
  }
  if (ret != RMW_RET_OK) {
    return ret;
  }
  // set endpoint gid
  uint8_t rmw_gid[RMW_GID_STORAGE_SIZE];
  memset(&rmw_gid, 0, RMW_GID_STORAGE_SIZE);
  memcpy(&rmw_gid, &dds_topic_endpoint_info->endpoint_guid, sizeof(DDS::GUID_t));
  ret = rmw_topic_endpoint_info_set_gid(topic_endpoint_info, rmw_gid, sizeof(DDS::GUID_t));
  if (ret != RMW_RET_OK) {
    return ret;
  }
  // set qos profile
  ret = rmw_topic_endpoint_info_set_qos_profile(
    topic_endpoint_info,
    &dds_topic_endpoint_info->qos_profile);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  // set topic type
  std::string type_name = no_mangle ? dds_topic_endpoint_info->topic_type : _demangle_if_ros_type(
    dds_topic_endpoint_info->topic_type);
  ret = rmw_topic_endpoint_info_set_topic_type(topic_endpoint_info, type_name.c_str(), allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  // Check if this participant is the same as the node that is passed
  auto guid_iter = participant_guid_to_name.find(dds_topic_endpoint_info->participant_guid);
  if (guid_iter == participant_guid_to_name.end()) {
    ret = rmw_topic_endpoint_info_set_node_name(
      topic_endpoint_info,
      "_NODE_NAME_UNKNOWN_",
      allocator);
    if (ret != RMW_RET_OK) {
      return ret;
    }
    ret = rmw_topic_endpoint_info_set_node_namespace(
      topic_endpoint_info,
      "_NODE_NAMESPACE_UNKNOWN_",
      allocator);
    if (ret != RMW_RET_OK) {
      return ret;
    }
  } else {
    ret = rmw_topic_endpoint_info_set_node_name(
      topic_endpoint_info,
      guid_iter->second.name.c_str(),
      allocator);
    if (ret != RMW_RET_OK) {
      return ret;
    }
    ret = rmw_topic_endpoint_info_set_node_namespace(
      topic_endpoint_info,
      guid_iter->second.namespace_.c_str(),
      allocator);
    if (ret != RMW_RET_OK) {
      return ret;
    }
    return ret;
  }

  return ret;
}

rmw_ret_t
_get_info_by_topic(
  const char * identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  bool is_publisher,
  rmw_topic_endpoint_info_array_t * participants_info)
{
  rmw_ret_t rmw_ret = _validate_arguments(
    identifier,
    node,
    allocator,
    topic_name,
    participants_info);
  if (RMW_RET_OK != rmw_ret) {
    return rmw_ret;
  }

  const char * node_name = node->name;
  const char * node_namespace = node->namespace_;
  const ConnextNodeInfo * connext_node_info = static_cast<ConnextNodeInfo *>(node->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(connext_node_info, RMW_RET_ERROR);
  DDS::DomainParticipant * participant = connext_node_info->participant;
  const std::vector<std::string> topic_fqdns = _get_topic_fqdns(topic_name, no_mangle);

  DDS::GUID_t participant_guid;
  DDS_InstanceHandle_to_GUID(&participant_guid, participant->get_instance_handle());

  DDS::InstanceHandleSeq handles;
  if (participant->get_discovered_participants(handles) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("unable to fetch discovered participants.");
    return RMW_RET_ERROR;
  }

  std::map<DDS::GUID_t, ParticipantNameInfo> participant_guid_to_name;
  participant_guid_to_name[participant_guid].name = node_name;
  participant_guid_to_name[participant_guid].namespace_ = node_namespace;
  for (int i = 0; i < handles.length(); ++i) {
    DDS::ParticipantBuiltinTopicData pbtd;
    DDS::ReturnCode_t dds_ret = participant->get_discovered_participant_data(pbtd, handles[i]);
    if (DDS::RETCODE_OK == dds_ret) {
      uint8_t * buf = pbtd.user_data.value.get_contiguous_buffer();
      if (nullptr != buf) {
        std::vector<uint8_t> kv(buf, buf + pbtd.user_data.value.length());
        auto map = rmw::impl::cpp::parse_key_value(kv);
        auto name_found = map.find("name");
        auto ns_found = map.find("namespace");
        if (name_found != map.end() && ns_found != map.end()) {
          DDS::GUID_t guid;
          DDS_BuiltinTopicKey_to_GUID(&guid, pbtd.key);
          participant_guid_to_name[guid].name = std::string(
            name_found->second.begin(),
            name_found->second.end());
          participant_guid_to_name[guid].namespace_ = std::string(
            ns_found->second.begin(),
            ns_found->second.end());
        }
      }
    } else {
      RMW_SET_ERROR_MSG("unable to fetch data on discovered participant.");
      return RMW_RET_ERROR;
    }
  }

  CustomDataReaderListener * slave_target = is_publisher ?
    static_cast<CustomDataReaderListener *>(connext_node_info->publisher_listener) :
    static_cast<CustomDataReaderListener *>(connext_node_info->subscriber_listener);

  std::vector<const DDSTopicEndpointInfo *> dds_topic_endpoint_infos;
  for (const auto & topic_fqdn : topic_fqdns) {
    slave_target->fill_topic_endpoint_infos(topic_fqdn, no_mangle, dds_topic_endpoint_infos);
  }

  // add all the elements from the vector to rmw_topic_endpoint_info_array_t
  size_t count = dds_topic_endpoint_infos.size();
  rmw_ret = rmw_topic_endpoint_info_array_init_with_size(participants_info, count, allocator);
  if (RMW_RET_OK != rmw_ret) {
    rmw_error_string_t error_message = rmw_get_error_string();
    rmw_reset_error();
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "rmw_topic_endpoint_info_array_init_with_size failed to allocate memory: %s",
      error_message.str);
    return rmw_ret;
  }
  for (size_t i = 0; i < count; ++i) {
    rmw_ret = _set_rmw_topic_endpoint_info(
      &participants_info->info_array[i],
      dds_topic_endpoint_infos[i],
      participant_guid_to_name,
      no_mangle,
      is_publisher,
      allocator);
    if (RMW_RET_OK != rmw_ret) {
      rmw_ret_t ret = rmw_topic_endpoint_info_array_fini(participants_info, allocator);
      if (RMW_RET_OK != ret) {
        RMW_SET_ERROR_MSG("unable to fini rmw_topic_endpoint_info_array_t \"participants_info\".");
      }
      return rmw_ret;
    }
  }

  return RMW_RET_OK;
}

rmw_ret_t
get_publishers_info_by_topic(
  const char * identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * publishers_info)
{
  return _get_info_by_topic(
    identifier,
    node,
    allocator,
    topic_name,
    no_mangle,
    true, /*is_publisher*/
    publishers_info);
}

rmw_ret_t
get_subscriptions_info_by_topic(
  const char * identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscriptions_info)
{
  return _get_info_by_topic(
    identifier,
    node,
    allocator,
    topic_name,
    no_mangle,
    false, /*is_publisher*/
    subscriptions_info);
}
