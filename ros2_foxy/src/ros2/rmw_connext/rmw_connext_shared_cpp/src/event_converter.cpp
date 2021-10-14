// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <unordered_map>

#include "rmw_connext_shared_cpp/event_converter.hpp"

/// Mapping of RMW_EVENT to the corresponding DDS_StatusKind.
static const std::unordered_map<rmw_event_type_t, DDS::StatusKind> mask_map{
  {RMW_EVENT_LIVELINESS_CHANGED, DDS_LIVELINESS_CHANGED_STATUS},
  {RMW_EVENT_REQUESTED_DEADLINE_MISSED, DDS_REQUESTED_DEADLINE_MISSED_STATUS},
  {RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS},
  {RMW_EVENT_LIVELINESS_LOST, DDS_LIVELINESS_LOST_STATUS},
  {RMW_EVENT_OFFERED_DEADLINE_MISSED, DDS_OFFERED_DEADLINE_MISSED_STATUS},
  {RMW_EVENT_OFFERED_QOS_INCOMPATIBLE, DDS_OFFERED_INCOMPATIBLE_QOS_STATUS},
};

DDS::StatusKind get_status_kind_from_rmw(const rmw_event_type_t event_t)
{
  return mask_map.at(event_t);
}

bool is_event_supported(const rmw_event_type_t event_t)
{
  return mask_map.count(event_t) > 0;
}

rmw_ret_t check_dds_ret_code(const DDS::ReturnCode_t dds_return_code)
{
  switch (dds_return_code) {
    case DDS_RETCODE_OK:
      return RMW_RET_OK;
    case DDS_RETCODE_ERROR:
      return RMW_RET_ERROR;
    case DDS_RETCODE_TIMEOUT:
      return RMW_RET_TIMEOUT;
    default:
      return RMW_RET_ERROR;
  }
}
