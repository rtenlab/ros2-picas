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

#include <string>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/sanity_checks.h"

#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw_connext_shared_cpp/node_names.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

rmw_ret_t
get_node_names_impl(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_names)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_namespaces)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  DDS::DomainParticipant * participant = static_cast<ConnextNodeInfo *>(node->data)->participant;
  DDS::InstanceHandleSeq handles;

  if (participant->get_discovered_participants(handles) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("unable to fetch discovered participants.");
    return RMW_RET_ERROR;
  }

  DDS::DomainParticipantQos participant_qos;
  DDS::ReturnCode_t status = participant->get_qos(participant_qos);

  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default participant qos");
    return RMW_RET_ERROR;
  }

  rmw_ret_t final_ret = RMW_RET_OK;

  auto length = handles.length() + 1;  // add yourself

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // Pre-allocate temporary buffer for all node names & namespaces
  // Nodes that are not created by ROS 2 could provide empty names
  // Such names should not be returned
  rcutils_string_array_t tmp_names_list = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t tmp_namespaces_list = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t tmp_enclaves_list = rcutils_get_zero_initialized_string_array();

  int named_nodes_num = 1;

  rcutils_ret_t rcutils_ret = rcutils_string_array_init(&tmp_names_list, length, &allocator);

  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }

  rcutils_ret = rcutils_string_array_init(&tmp_namespaces_list, length, &allocator);

  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }

  if (enclaves) {
    rcutils_ret = rcutils_string_array_init(&tmp_enclaves_list, length, &allocator);

    if (rcutils_ret != RCUTILS_RET_OK) {
      RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
      final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
      goto cleanup;
    }
  }

  // add yourself
  tmp_names_list.data[0] = rcutils_strdup(participant_qos.participant_name.name, allocator);
  if (!tmp_names_list.data[0]) {
    RMW_SET_ERROR_MSG("could not allocate memory for a node name");
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }
  tmp_namespaces_list.data[0] = rcutils_strdup(node->namespace_, allocator);
  if (!tmp_namespaces_list.data[0]) {
    RMW_SET_ERROR_MSG("could not allocate memory for a node namespace");
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }
  if (enclaves) {
    tmp_enclaves_list.data[0] = rcutils_strdup(
      node->context->options.enclave, allocator);
    if (!tmp_namespaces_list.data[0]) {
      RMW_SET_ERROR_MSG("could not allocate memory for a enclave name");
      final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
      goto cleanup;
    }
  }

  for (auto i = 1; i < length; ++i) {
    DDS::ParticipantBuiltinTopicData pbtd;
    auto dds_ret = participant->get_discovered_participant_data(pbtd, handles[i - 1]);
    std::string name;
    std::string namespace_;
    std::string enclave;
    if (DDS::RETCODE_OK == dds_ret) {
      auto data = static_cast<unsigned char *>(pbtd.user_data.value.get_contiguous_buffer());
      std::vector<uint8_t> kv(data, data + pbtd.user_data.value.length());

      auto map = rmw::impl::cpp::parse_key_value(kv);

      auto name_found = map.find("name");
      auto ns_found = map.find("namespace");
      auto enclave_found = map.find("enclave");

      if (name_found != map.end()) {
        name = std::string(name_found->second.begin(), name_found->second.end());
      }

      if (ns_found != map.end()) {
        namespace_ = std::string(ns_found->second.begin(), ns_found->second.end());
      }

      if (enclave_found != map.end()) {
        enclave = std::string(
          enclave_found->second.begin(), enclave_found->second.end());
      }
    }

    // ignore discovered participants without a name
    if (name.empty()) {
      tmp_names_list.data[i] = nullptr;
      tmp_namespaces_list.data[i] = nullptr;
      continue;
    }

    tmp_names_list.data[named_nodes_num] = rcutils_strdup(name.c_str(), allocator);
    if (!tmp_names_list.data[named_nodes_num]) {
      RMW_SET_ERROR_MSG("could not allocate memory for a node's name");
      final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
      goto cleanup;
    }

    tmp_namespaces_list.data[named_nodes_num] = rcutils_strdup(namespace_.c_str(), allocator);
    if (!tmp_namespaces_list.data[named_nodes_num]) {
      RMW_SET_ERROR_MSG("could not allocate memory for a node's namespace");
      final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
      goto cleanup;
    }

    if (enclaves) {
      tmp_enclaves_list.data[named_nodes_num] = rcutils_strdup(
        enclave.c_str(), allocator);
      if (!tmp_enclaves_list.data[named_nodes_num]) {
        RMW_SET_ERROR_MSG("could not allocate memory for a node's namespace");
        final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
        goto cleanup;
      }
    }

    ++named_nodes_num;
  }

  // prepare actual output without empty names
  rcutils_ret = rcutils_string_array_init(node_names, named_nodes_num, &allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("could not allocate memory for node_names output");
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }

  rcutils_ret = rcutils_string_array_init(node_namespaces, named_nodes_num, &allocator);

  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("could not allocate memory for node_namespaces output");
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }

  if (enclaves) {
    rcutils_ret = rcutils_string_array_init(enclaves, named_nodes_num, &allocator);

    if (rcutils_ret != RCUTILS_RET_OK) {
      RMW_SET_ERROR_MSG("could not allocate memory for node_namespaces output");
      final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
      goto cleanup;
    }
  }

  for (auto i = 0; i < named_nodes_num; ++i) {
    node_names->data[i] = rcutils_strdup(tmp_names_list.data[i], allocator);
    node_namespaces->data[i] = rcutils_strdup(tmp_namespaces_list.data[i], allocator);
    if (enclaves) {
      enclaves->data[i] = rcutils_strdup(tmp_enclaves_list.data[i], allocator);
    }
  }

  rcutils_ret = rcutils_string_array_fini(&tmp_names_list);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to free tmp_names_list");
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }

  rcutils_ret = rcutils_string_array_fini(&tmp_namespaces_list);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to free tmp_namespaces_list");
    final_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
    goto cleanup;
  }

  return RMW_RET_OK;

cleanup:
  if (node_names) {
    rcutils_ret = rcutils_string_array_fini(node_names);
    if (rcutils_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_connext_cpp",
        "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }
  }

  if (node_namespaces) {
    rcutils_ret = rcutils_string_array_fini(node_namespaces);
    if (rcutils_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_connext_cpp",
        "failed to cleanup during error handling: %s", rcutils_get_error_string().str
      );
      rcutils_reset_error();
    }
  }

  if (enclaves) {
    rcutils_ret = rcutils_string_array_fini(enclaves);
    if (rcutils_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_connext_cpp",
        "failed to cleanup during error handling: %s", rcutils_get_error_string().str
      );
      rcutils_reset_error();
    }
  }

  rcutils_ret = rcutils_string_array_fini(&tmp_names_list);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_connext_cpp",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str
    );
    rcutils_reset_error();
  }

  rcutils_ret = rcutils_string_array_fini(&tmp_namespaces_list);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_connext_cpp",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str
    );
    rcutils_reset_error();
  }

  rcutils_ret = rcutils_string_array_fini(&tmp_enclaves_list);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_connext_cpp",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str
    );
    rcutils_reset_error();
  }

  return final_ret;
}

rmw_ret_t
get_node_names(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  return get_node_names_impl(implementation_identifier, node, node_names, node_namespaces, nullptr);
}

rmw_ret_t
get_node_names_with_enclaves(
  const char * implementation_identifier,
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(enclaves)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  return get_node_names_impl(
    implementation_identifier, node, node_names, node_namespaces, enclaves);
}
