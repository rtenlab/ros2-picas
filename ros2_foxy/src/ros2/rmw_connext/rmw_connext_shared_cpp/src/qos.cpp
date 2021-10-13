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

#include <limits>
#include "rmw_connext_shared_cpp/qos.hpp"

namespace
{

bool
is_time_default(const rmw_time_t & time)
{
  return time.sec == 0 && time.nsec == 0;
}

DDS_Duration_t
rmw_time_to_dds(const rmw_time_t & time)
{
  DDS_Duration_t duration;
  duration.sec = static_cast<DDS_Long>(time.sec);
  duration.nanosec = static_cast<DDS_UnsignedLong>(time.nsec);
  return duration;
}

template<typename DDSEntityQos>
bool
set_entity_qos_from_profile_generic(
  const rmw_qos_profile_t & qos_profile,
  DDSEntityQos & entity_qos)
{
  // Read properties from the rmw profile
  switch (qos_profile.history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      entity_qos.history.kind = DDS::KEEP_LAST_HISTORY_QOS;
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      entity_qos.history.kind = DDS::KEEP_ALL_HISTORY_QOS;
      break;
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
      break;
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
    default:
      RMW_SET_ERROR_MSG("Unknown QoS history policy");
      return false;
  }

  switch (qos_profile.reliability) {
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      entity_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
      break;
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      entity_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
      break;
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      break;
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
    default:
      RMW_SET_ERROR_MSG("Unknown QoS reliability policy");
      return false;
  }

  switch (qos_profile.durability) {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      entity_qos.durability.kind = DDS::TRANSIENT_LOCAL_DURABILITY_QOS;
      break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      entity_qos.durability.kind = DDS::VOLATILE_DURABILITY_QOS;
      break;
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      break;
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
    default:
      RMW_SET_ERROR_MSG("Unknown QoS durability policy");
      return false;
  }

  if (qos_profile.depth != RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT) {
    entity_qos.history.depth = static_cast<DDS::Long>(qos_profile.depth);
  }

  // DDS_DeadlineQosPolicy has default value of DDS_DURATION_INFINITE
  // don't overwrite if default passed
  if (!is_time_default(qos_profile.deadline)) {
    entity_qos.deadline.period = rmw_time_to_dds(qos_profile.deadline);
  }

  switch (qos_profile.liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      entity_qos.liveliness.kind = DDS::AUTOMATIC_LIVELINESS_QOS;
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      entity_qos.liveliness.kind = DDS::MANUAL_BY_TOPIC_LIVELINESS_QOS;
      break;
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
      break;
    case RMW_QOS_POLICY_LIVELINESS_UNKNOWN:
    default:
      RMW_SET_ERROR_MSG("Unknown QoS liveliness policy");
      return false;
  }
  if (!is_time_default(qos_profile.liveliness_lease_duration)) {
    entity_qos.liveliness.lease_duration = rmw_time_to_dds(qos_profile.liveliness_lease_duration);
  }

  // ensure the history depth is at least the requested queue size
  assert(entity_qos.history.depth >= 0);
  if (
    entity_qos.history.kind == DDS::KEEP_LAST_HISTORY_QOS &&
    static_cast<size_t>(entity_qos.history.depth) < qos_profile.depth)
  {
    if (qos_profile.depth > static_cast<size_t>((std::numeric_limits<DDS::Long>::max)())) {
      RMW_SET_ERROR_MSG(
        "failed to set history depth since the requested queue size exceeds the DDS type");
      return false;
    }
    entity_qos.history.depth = static_cast<DDS::Long>(qos_profile.depth);
  }
  return true;
}

bool
set_entity_qos_from_profile(
  const rmw_qos_profile_t & qos_profile,
  DDS::DataReaderQos & entity_qos)
{
  // Set any QoS settings that are specific to DataReader, then call the shared version
  return set_entity_qos_from_profile_generic(qos_profile, entity_qos);
}

bool
set_entity_qos_from_profile(
  const rmw_qos_profile_t & qos_profile,
  DDS::DataWriterQos & entity_qos)
{
  // Set any QoS settings that are specific to DataWriter, then call the shared version
  if (!is_time_default(qos_profile.lifespan)) {
    entity_qos.lifespan.duration = rmw_time_to_dds(qos_profile.lifespan);
  }
  return set_entity_qos_from_profile_generic(qos_profile, entity_qos);
}

}  // anonymous namespace

bool
get_datareader_qos(
  DDS::DomainParticipant * participant,
  const rmw_qos_profile_t & qos_profile,
  DDS::DataReaderQos & datareader_qos)
{
  DDS::ReturnCode_t status = participant->get_default_datareader_qos(datareader_qos);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default datareader qos");
    return false;
  }

  status = DDS::PropertyQosPolicyHelper::add_property(
    datareader_qos.property,
    "dds.data_reader.history.memory_manager.fast_pool.pool_buffer_max_size",
    "4096",
    DDS::BOOLEAN_FALSE);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to add qos property");
    return false;
  }

  status = DDS::PropertyQosPolicyHelper::add_property(
    datareader_qos.property,
    "reader_resource_limits.dynamically_allocate_fragmented_samples",
    "1",
    DDS::BOOLEAN_FALSE);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to add qos property");
    return false;
  }

  if (!set_entity_qos_from_profile(qos_profile, datareader_qos)) {
    return false;
  }

  return true;
}

bool
get_datawriter_qos(
  DDS::DomainParticipant * participant,
  const rmw_qos_profile_t & qos_profile,
  DDS::DataWriterQos & datawriter_qos)
{
  DDS::ReturnCode_t status = participant->get_default_datawriter_qos(datawriter_qos);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default datawriter qos");
    return false;
  }

  status = DDS::PropertyQosPolicyHelper::add_property(
    datawriter_qos.property,
    "dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size",
    "4096",
    DDS::BOOLEAN_FALSE);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to add qos property");
    return false;
  }

  if (!set_entity_qos_from_profile(qos_profile, datawriter_qos)) {
    return false;
  }

  // TODO(wjwwood): conditionally use the async publish mode using a heuristic:
  //  https://github.com/ros2/rmw_connext/issues/190
  datawriter_qos.publish_mode.kind = DDS::ASYNCHRONOUS_PUBLISH_MODE_QOS;

  return true;
}

rmw_qos_policy_kind_t
dds_qos_policy_to_rmw_qos_policy(DDS::QosPolicyId_t policy_id)
{
  switch (policy_id) {
    case DDS_DURABILITY_QOS_POLICY_ID:
      return RMW_QOS_POLICY_DURABILITY;
    case DDS_DEADLINE_QOS_POLICY_ID:
      return RMW_QOS_POLICY_DEADLINE;
    case DDS_LIVELINESS_QOS_POLICY_ID:
      return RMW_QOS_POLICY_LIVELINESS;
    case DDS_RELIABILITY_QOS_POLICY_ID:
      return RMW_QOS_POLICY_RELIABILITY;
    case DDS_HISTORY_QOS_POLICY_ID:
      return RMW_QOS_POLICY_HISTORY;
    case DDS_LIFESPAN_QOS_POLICY_ID:
      return RMW_QOS_POLICY_LIFESPAN;
    default:
      return RMW_QOS_POLICY_INVALID;
  }
}

template<typename AttributeT>
void
dds_qos_lifespan_to_rmw_qos_lifespan(
  const AttributeT & dds_qos,
  rmw_qos_profile_t * qos)
{
  qos->lifespan.sec = dds_qos.lifespan.duration.sec;
  qos->lifespan.nsec = dds_qos.lifespan.duration.nanosec;
}

template<>
void
dds_qos_lifespan_to_rmw_qos_lifespan<DDS::DataReaderQos>(
  const DDS::DataReaderQos & /*dds_qos*/,
  rmw_qos_profile_t * /*qos*/)
{
  // lifespan does does not exist in DataReader, so no-op here
}

template<>
void
dds_qos_lifespan_to_rmw_qos_lifespan<DDS::SubscriptionBuiltinTopicData>(
  const DDS::SubscriptionBuiltinTopicData & /*dds_qos*/,
  rmw_qos_profile_t * /*qos*/)
{
  // lifespan does does not exist in DataReader, so no-op here
}

template<typename AttributeT>
void
dds_qos_to_rmw_qos(
  const AttributeT & dds_qos,
  rmw_qos_profile_t * qos)
{
  dds_remote_qos_to_rmw_qos(dds_qos, qos);

  switch (dds_qos.history.kind) {
    case DDS_KEEP_LAST_HISTORY_QOS:
      qos->history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      break;
    case DDS_KEEP_ALL_HISTORY_QOS:
      qos->history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      break;
    default:
      qos->history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
      break;
  }
  qos->depth = static_cast<size_t>(dds_qos.history.depth);
}

template
void dds_qos_to_rmw_qos<DDS::DataWriterQos>(
  const DDS::DataWriterQos & dds_qos,
  rmw_qos_profile_t * qos);

template
void dds_qos_to_rmw_qos<DDS::DataReaderQos>(
  const DDS::DataReaderQos & dds_qos,
  rmw_qos_profile_t * qos);

template<typename AttributeT>
void
dds_remote_qos_to_rmw_qos(
  const AttributeT & dds_qos,
  rmw_qos_profile_t * qos)
{
  qos->history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
  qos->depth = RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT;

  switch (dds_qos.reliability.kind) {
    case DDS_BEST_EFFORT_RELIABILITY_QOS:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      break;
    case DDS_RELIABLE_RELIABILITY_QOS:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      break;
    default:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
      break;
  }

  switch (dds_qos.durability.kind) {
    case DDS_TRANSIENT_LOCAL_DURABILITY_QOS:
      qos->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      break;
    case DDS_VOLATILE_DURABILITY_QOS:
      qos->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
      break;
    default:
      qos->durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
      break;
  }

  qos->deadline.sec = dds_qos.deadline.period.sec;
  qos->deadline.nsec = dds_qos.deadline.period.nanosec;

  dds_qos_lifespan_to_rmw_qos_lifespan(dds_qos, qos);

  switch (dds_qos.liveliness.kind) {
    case DDS_AUTOMATIC_LIVELINESS_QOS:
      qos->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
      break;
    case DDS_MANUAL_BY_TOPIC_LIVELINESS_QOS:
      qos->liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
      break;
    default:
      qos->liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
      break;
  }
  qos->liveliness_lease_duration.sec = dds_qos.liveliness.lease_duration.sec;
  qos->liveliness_lease_duration.nsec = dds_qos.liveliness.lease_duration.nanosec;
}

template
void dds_remote_qos_to_rmw_qos<DDS::PublicationBuiltinTopicData>(
  const DDS::PublicationBuiltinTopicData & dds_qos,
  rmw_qos_profile_t * qos);

template
void dds_remote_qos_to_rmw_qos<DDS::SubscriptionBuiltinTopicData>(
  const DDS::SubscriptionBuiltinTopicData & dds_qos,
  rmw_qos_profile_t * qos);
