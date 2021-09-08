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

#include <string>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"

#include "identifier.hpp"
#include "types.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header, void * ros_response)
{
  if (!service) {
    RMW_SET_ERROR_MSG("service handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service handle,
    service->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  if (!request_header) {
    RMW_SET_ERROR_MSG("ros request header handle is null");
    return RMW_RET_ERROR;
  }
  if (!ros_response) {
    RMW_SET_ERROR_MSG("ros response handle is null");
    return RMW_RET_ERROR;
  }

  OpenSpliceStaticServiceInfo * service_info =
    static_cast<OpenSpliceStaticServiceInfo *>(service->data);
  if (!service_info) {
    RMW_SET_ERROR_MSG("service info handle is null");
    return RMW_RET_ERROR;
  }
  void * responder = service_info->responder_;
  if (!responder) {
    RMW_SET_ERROR_MSG("responder handle is null");
    return RMW_RET_ERROR;
  }
  const service_type_support_callbacks_t * callbacks = service_info->callbacks_;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }
  const char * error_string =
    callbacks->send_response(responder, request_header, ros_response);
  if (error_string) {
    RMW_SET_ERROR_MSG(error_string);
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client, rmw_request_id_t * request_header,
  void * ros_response, bool * taken)
{
  if (!client) {
    RMW_SET_ERROR_MSG("client handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client handle,
    client->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  if (!request_header) {
    RMW_SET_ERROR_MSG("ros request header handle is null");
    return RMW_RET_ERROR;
  }
  if (!ros_response) {
    RMW_SET_ERROR_MSG("ros response handle is null");
    return RMW_RET_ERROR;
  }
  if (taken == nullptr) {
    RMW_SET_ERROR_MSG("taken argument cannot be null");
    return RMW_RET_ERROR;
  }

  OpenSpliceStaticClientInfo * client_info =
    static_cast<OpenSpliceStaticClientInfo *>(client->data);
  if (!client_info) {
    RMW_SET_ERROR_MSG("client info handle is null");
    return RMW_RET_ERROR;
  }
  void * requester = client_info->requester_;
  if (!requester) {
    RMW_SET_ERROR_MSG("requester handle is null");
    return RMW_RET_ERROR;
  }
  const service_type_support_callbacks_t * callbacks = client_info->callbacks_;
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return RMW_RET_ERROR;
  }
  const char * error_string =
    callbacks->take_response(requester, request_header, ros_response, taken);
  if (error_string) {
    RMW_SET_ERROR_MSG((std::string("failed to take response: ") + error_string).c_str());
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}
}  // extern "C"
