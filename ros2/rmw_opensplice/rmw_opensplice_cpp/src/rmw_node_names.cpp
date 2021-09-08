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

#include <vector>
#include <string>
#include <iostream>

#include "rcutils/types/string_array.h"
#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rmw/allocators.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/sanity_checks.h"
#include "types.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  DDS::DataReader_var loopup_datareader;
  DDS::ReturnCode_t status;
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return RMW_RET_ERROR;
  }
  if (rmw_check_zero_rmw_string_array(node_names) != RMW_RET_OK) {
    return RMW_RET_ERROR;
  }
  if (rmw_check_zero_rmw_string_array(node_namespaces) != RMW_RET_OK) {
    return RMW_RET_ERROR;
  }

  // Lookup the ROS Node names from the DDS Builtin Participant Topic userData
  //   For extensibility and backwards compatibility a format is applied that must be understood.
  //   The format specifies a list of name-value pairs,
  //   Format (byte ascii string) syntax : { <name> '=' <value> ';' }*
  //   A <name> is a string and a <value> is a sequence of octets.
  //   The '=' and ';' characters are reserved delimiters.
  //   For <name> only alphanumeric character are allowed.
  //   For <value> there is no limitation, ';' can be used as part of the <value> when escaped by
  //   a second ';'.
  //   Implemented policy value: "name=<node-name>;"
  //   userData not following this policy will be ignored completely.

  // Lookup all Builtin Participant Topic DataReader instance handles to access the userData.
  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return RMW_RET_ERROR;
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

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  int length = data.length();
  rcutils_string_array_t node_list = rcutils_get_zero_initialized_string_array();
  rcutils_ret_t rcutils_ret = rcutils_string_array_init(&node_list, length, &allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    rcutils_reset_error();
    return rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
  }

  rcutils_string_array_t ns_list = rcutils_get_zero_initialized_string_array();
  rcutils_ret = rcutils_string_array_init(&ns_list, length, &allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    rcutils_reset_error();
    return rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
  }

  int n = 0;
  for (auto i = 0; i < length; i++) {
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
        node_list.data[n] = rcutils_strndup(name.c_str(), name.size(), allocator);
        if (!node_list.data[n]) {
          RMW_SET_ERROR_MSG("could not allocate memory for node name");
          goto fail;
        }

        std::string ns(ns_found->second.begin(), ns_found->second.end());
        ns_list.data[n] = rcutils_strndup(ns.c_str(), ns.size(), allocator);
        if (!ns_list.data[n]) {
          RMW_SET_ERROR_MSG("could not allocate memory for node name");
          goto fail;
        }
        n++;
      }
    }
  }

  participantReader->return_loan(data, info);

  // Allocate the node_names out-buffer according to the number of Node names
  rcutils_ret = rcutils_string_array_init(node_names, n, &allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    rcutils_reset_error();
    goto fail;
  }

  rcutils_ret = rcutils_string_array_init(node_namespaces, n, &allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    rcutils_reset_error();
    goto fail;
  }

  // Move the content from temporary list to the out-buffer and release temporary list.
  for (auto i = 0; i < n; ++i) {
    node_names->data[i] = node_list.data[i];
    node_list.data[i] = NULL;
  }

  // Move the content from temporary list to the out-buffer and release temporary list.
  for (auto i = 0; i < n; ++i) {
    node_namespaces->data[i] = ns_list.data[i];
    ns_list.data[i] = NULL;
  }

  return RMW_RET_OK;
fail:
  rcutils_ret = rcutils_string_array_fini(&ns_list);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_opensplice_cpp",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  rcutils_ret = rcutils_string_array_fini(&node_list);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_opensplice_cpp",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  rcutils_ret = rcutils_string_array_fini(node_names);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_opensplice_cpp",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  rcutils_ret = rcutils_string_array_fini(node_namespaces);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_opensplice_cpp",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  return RMW_RET_BAD_ALLOC;
}
}  // extern "C"
