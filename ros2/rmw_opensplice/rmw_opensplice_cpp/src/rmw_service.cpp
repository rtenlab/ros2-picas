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
rmw_service_t *
rmw_create_service(
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
  rmw_service_t * service = nullptr;
  const char * error_string = nullptr;
  DDS::DataReader * request_datareader = nullptr;
  DDS::ReadCondition * read_condition = nullptr;
  void * responder = nullptr;
  OpenSpliceStaticServiceInfo * service_info = nullptr;
  // Begin initialization of elements.
  service = rmw_service_allocate();
  if (!service) {
    RMW_SET_ERROR_MSG("failed to allocate service");
    goto fail;
  }

  if (!get_datareader_qos(nullptr, *qos_profile, datareader_qos)) {
    goto fail;
  }

  if (!get_datawriter_qos(nullptr, *qos_profile, datawriter_qos)) {
    goto fail;
  }

  error_string = callbacks->create_responder(
    participant, service_name,
    reinterpret_cast<void **>(&responder),
    reinterpret_cast<void **>(&request_datareader),
    reinterpret_cast<const void *>(&datareader_qos),
    reinterpret_cast<const void *>(&datawriter_qos),
    qos_profile->avoid_ros_namespace_conventions,
    &rmw_allocate);
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to create responder: ") + error_string).c_str());
    goto fail;
  }

  read_condition = request_datareader->create_readcondition(
    DDS::ANY_SAMPLE_STATE, DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  service_info =
    static_cast<OpenSpliceStaticServiceInfo *>(rmw_allocate(sizeof(OpenSpliceStaticServiceInfo)));
  if (!service_info) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  service_info->responder_ = responder;
  service_info->callbacks_ = callbacks;
  service_info->request_datareader_ = request_datareader;
  service_info->read_condition_ = read_condition;

  service->implementation_identifier = opensplice_cpp_identifier;
  service->data = service_info;
  service->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  if (!service->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(service->service_name), service_name, strlen(service_name) + 1);
  return service;
fail:
  if (request_datareader) {
    if (read_condition) {
      if (request_datareader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
        fprintf(stderr, "leaking readcondition while handling failure\n");
      }
    }
  }

  if (responder) {
    const char * error_string = callbacks->destroy_responder(responder, &rmw_free);
    if (error_string) {
      std::stringstream ss;
      ss << "failed to destroy responder: " << error_string << ", at: " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (service_info) {
    rmw_free(service_info);
  }
  if (service) {
    rmw_service_free(service);
  }
  return nullptr;
}

rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  (void)node;
  if (!service) {
    RMW_SET_ERROR_MSG("service handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service handle,
    service->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  OpenSpliceStaticServiceInfo * service_info =
    static_cast<OpenSpliceStaticServiceInfo *>(service->data);

  auto result = RMW_RET_OK;
  if (service_info) {
    auto request_datareader = service_info->request_datareader_;
    if (request_datareader) {
      auto read_condition = service_info->read_condition_;
      if (read_condition) {
        if (request_datareader->delete_readcondition(read_condition) != DDS::RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete readcondition");
          result = RMW_RET_ERROR;
        }
        service_info->read_condition_ = nullptr;
      }
    }
  } else {
    RMW_SET_ERROR_MSG("service_info handle is null");
    return RMW_RET_ERROR;
  }

  const service_type_support_callbacks_t * callbacks =
    static_cast<const service_type_support_callbacks_t *>(service_info->callbacks_);
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }
  const char * error_string = callbacks->destroy_responder(service_info->responder_, &rmw_free);
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to destroy responder: ") + error_string).c_str());
  }

  if (service->service_name) {
    rmw_free(const_cast<char *>(service->service_name));
  }

  rmw_free(service_info);
  rmw_service_free(service);
  return result;
}
}  // extern "C"
