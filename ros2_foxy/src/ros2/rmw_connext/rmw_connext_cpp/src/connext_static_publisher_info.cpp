// Copyright 2015-2019 Open Source Robotics Foundation, Inc.
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


#include "rmw_connext_cpp/connext_static_publisher_info.hpp"
#include "rmw_connext_shared_cpp/event_converter.hpp"
#include "rmw_connext_shared_cpp/qos.hpp"

rmw_ret_t ConnextStaticPublisherInfo::get_status(
  DDS::StatusMask mask,
  void * event)
{
  switch (mask) {
    case DDS::StatusKind::DDS_LIVELINESS_LOST_STATUS:
      {
        DDS::LivelinessLostStatus liveliness_lost;
        DDS::ReturnCode_t dds_return_code =
          topic_writer_->get_liveliness_lost_status(liveliness_lost);

        rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
        if (from_dds != RMW_RET_OK) {
          return from_dds;
        }

        rmw_liveliness_lost_status_t * rmw_liveliness_lost =
          static_cast<rmw_liveliness_lost_status_t *>(event);
        rmw_liveliness_lost->total_count = liveliness_lost.total_count;
        rmw_liveliness_lost->total_count_change = liveliness_lost.total_count_change;

        break;
      }
    case DDS::StatusKind::DDS_OFFERED_DEADLINE_MISSED_STATUS:
      {
        DDS::OfferedDeadlineMissedStatus offered_deadline_missed;
        DDS::ReturnCode_t dds_return_code =
          topic_writer_->get_offered_deadline_missed_status(offered_deadline_missed);

        rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
        if (from_dds != RMW_RET_OK) {
          return from_dds;
        }

        rmw_offered_deadline_missed_status_t * rmw_offered_deadline_missed =
          static_cast<rmw_offered_deadline_missed_status_t *>(event);
        rmw_offered_deadline_missed->total_count = offered_deadline_missed.total_count;
        rmw_offered_deadline_missed->total_count_change =
          offered_deadline_missed.total_count_change;

        break;
      }
    case DDS::StatusKind::DDS_OFFERED_INCOMPATIBLE_QOS_STATUS:
      {
        DDS::OfferedIncompatibleQosStatus offered_incompatible_qos;
        DDS::ReturnCode_t dds_return_code =
          topic_writer_->get_offered_incompatible_qos_status(offered_incompatible_qos);

        rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
        if (RMW_RET_OK != from_dds) {
          return from_dds;
        }

        rmw_offered_qos_incompatible_event_status_t * rmw_offered_qos_incompatible =
          static_cast<rmw_offered_qos_incompatible_event_status_t *>(event);
        rmw_offered_qos_incompatible->total_count = offered_incompatible_qos.total_count;
        rmw_offered_qos_incompatible->total_count_change =
          offered_incompatible_qos.total_count_change;
        rmw_offered_qos_incompatible->last_policy_kind = dds_qos_policy_to_rmw_qos_policy(
          offered_incompatible_qos.last_policy_id);

        break;
      }
    default:
      return RMW_RET_UNSUPPORTED;
  }
  return RMW_RET_OK;
}

DDS::Entity * ConnextStaticPublisherInfo::get_entity()
{
  return topic_writer_;
}
