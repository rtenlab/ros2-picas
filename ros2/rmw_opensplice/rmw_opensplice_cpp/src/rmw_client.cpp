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
#include <dds_dcps.h>
#include <string>

#include "rosidl_typesupport_opensplice_c/identifier.h"
#include "rosidl_typesupport_opensplice_cpp/identifier.hpp"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"


#include "identifier.hpp"
#include "qos.hpp"
#include "types.hpp"
#include "typesupport_macros.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return nullptr;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, opensplice_cpp_identifier,
    return nullptr)

  RMW_OPENSPLICE_EXTRACT_SERVICE_TYPESUPPORT(
    type_supports, type_support)

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return NULL;
  }

  const service_type_support_callbacks_t * callbacks =
    static_cast<const service_type_support_callbacks_t *>(type_support->data);
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return NULL;
  }

  DDS::DataReaderQos datareader_qos;
  DDS::DataWriterQos datawriter_qos;

  // Past this point, a failure results in unrolling code in the goto fail block.
  rmw_client_t * client = nullptr;
  const char * error_string = nullptr;
  DDS::DataReader * response_datareader = nullptr;
  DDS::ReadCondition * read_condition = nullptr;
  DDS::DataWriter * request_datawriter = nullptr;
  OpenSpliceStaticClientInfo * client_info = nullptr;
  // Begin initializing elements.
  client = rmw_client_allocate();
  if (!client) {
    RMW_SET_ERROR_MSG("failed to allocate client");
    goto fail;
  }

  if (!get_datareader_qos(nullptr, *qos_profile, datareader_qos)) {
    goto fail;
  }

  if (!get_datawriter_qos(nullptr, *qos_profile, datawriter_qos)) {
    goto fail;
  }

  error_string = callbacks->create_requester(
    participant, service_name,
    reinterpret_cast<void **>(&request_datawriter),
    reinterpret_cast<void **>(&response_datareader),
    reinterpret_cast<const void *>(&datareader_qos),
    reinterpret_cast<const void *>(&datawriter_qos),
    qos_profile->avoid_ros_namespace_conventions,
    &rmw_allocate);
  // @todo: ross-desmond
  // Discovery for built in topics goes here
  // -- instance handles are a difference size than keys are...
  // They are most likely the middle 8 Bytes
  // defining system:local ids and leaving out domain since it may be assumed.
  // However this is not defined in reference manuals and will need to be checked.

  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to create request_datawriter: ") +
      error_string).c_str());
    goto fail;
  }
  if (!request_datawriter) {
    RMW_SET_ERROR_MSG("failed to create request_datawriter: request_datawriter is null");
    goto fail;
  }
  if (!response_datareader) {
    RMW_SET_ERROR_MSG("failed to create request_datawriter: response_datareader is null");
    goto fail;
  }

  read_condition = response_datareader->create_readcondition(
    DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  client_info = static_cast<OpenSpliceStaticClientInfo *>(
    rmw_allocate(sizeof(OpenSpliceStaticClientInfo)));
  if (!client_info) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  client_info->requester_ = request_datawriter;
  client_info->callbacks_ = callbacks;
  client_info->response_datareader_ = response_datareader;
  client_info->read_condition_ = read_condition;

  client->implementation_identifier = opensplice_cpp_identifier;
  client->data = client_info;
  client->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  if (!client->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(client->service_name), service_name, strlen(service_name) + 1);
  return client;
fail:
  if (response_datareader) {
    if (read_condition) {
      if (response_datareader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
        fprintf(stderr, "leaking readcondition while handling failure\n");
      }
    }
  }

  if (request_datawriter) {
    const char * error_string = callbacks->destroy_requester(request_datawriter, &rmw_free);
    if (error_string) {
      std::stringstream ss;
      ss << "failed to destroy request_datawriter: " << error_string << ", at: " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (client_info) {
    rmw_free(client_info);
  }
  if (client) {
    rmw_client_free(client);
  }
  return nullptr;
}

rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  (void)node;
  if (!client) {
    RMW_SET_ERROR_MSG("client handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client handle,
    client->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  OpenSpliceStaticClientInfo * client_info =
    static_cast<OpenSpliceStaticClientInfo *>(client->data);

  auto result = RMW_RET_OK;
  if (client_info) {
    auto response_datareader = client_info->response_datareader_;
    if (response_datareader) {
      auto read_condition = client_info->read_condition_;
      if (read_condition) {
        if (response_datareader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete readcondition");
          result = RMW_RET_ERROR;
        }
        client_info->read_condition_ = nullptr;
      }
    }
  } else {
    RMW_SET_ERROR_MSG("client_info handle is null");
    return RMW_RET_ERROR;
  }

  const service_type_support_callbacks_t * callbacks =
    static_cast<const service_type_support_callbacks_t *>(client_info->callbacks_);
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }

  const char * error_string = callbacks->destroy_requester(client_info->requester_, &rmw_free);
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to destroy request_datawriter: ") +
      error_string).c_str());
    return RMW_RET_ERROR;
  }
  if (client->service_name) {
    rmw_free(const_cast<char *>(client->service_name));
  }
  rmw_free(client_info);
  rmw_client_free(client);
  return result;
}
}  // extern "C"
