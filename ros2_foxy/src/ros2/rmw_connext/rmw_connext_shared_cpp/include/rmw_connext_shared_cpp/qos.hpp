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

#ifndef RMW_CONNEXT_SHARED_CPP__QOS_HPP_
#define RMW_CONNEXT_SHARED_CPP__QOS_HPP_

#include <cassert>
#include <limits>

#include "ndds_include.hpp"

#include "rmw/error_handling.h"
#include "rmw/types.h"
#include "rmw/incompatible_qos_events_statuses.h"

#include "rmw_connext_shared_cpp/visibility_control.h"

RMW_CONNEXT_SHARED_CPP_PUBLIC
bool
get_datareader_qos(
  DDS::DomainParticipant * participant,
  const rmw_qos_profile_t & qos_profile,
  DDS::DataReaderQos & datareader_qos);

RMW_CONNEXT_SHARED_CPP_PUBLIC
bool
get_datawriter_qos(
  DDS::DomainParticipant * participant,
  const rmw_qos_profile_t & qos_profile,
  DDS::DataWriterQos & datawriter_qos);

RMW_CONNEXT_SHARED_CPP_PUBLIC
rmw_qos_policy_kind_t
dds_qos_policy_to_rmw_qos_policy(DDS::QosPolicyId_t policy_id);

template<typename AttributeT>
void
dds_qos_to_rmw_qos(
  const AttributeT & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_CONNEXT_SHARED_CPP_PUBLIC
void
dds_qos_to_rmw_qos<DDS::DataWriterQos>(
  const DDS::DataWriterQos & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_CONNEXT_SHARED_CPP_PUBLIC
void
dds_qos_to_rmw_qos<DDS::DataReaderQos>(
  const DDS::DataReaderQos & dds_qos,
  rmw_qos_profile_t * qos);

template<typename AttributeT>
void
dds_remote_qos_to_rmw_qos(
  const AttributeT & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_CONNEXT_SHARED_CPP_PUBLIC
void
dds_remote_qos_to_rmw_qos<DDS::PublicationBuiltinTopicData>(
  const DDS::PublicationBuiltinTopicData & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_CONNEXT_SHARED_CPP_PUBLIC
void
dds_remote_qos_to_rmw_qos<DDS::SubscriptionBuiltinTopicData>(
  const DDS::SubscriptionBuiltinTopicData & dds_qos,
  rmw_qos_profile_t * qos);

#endif  // RMW_CONNEXT_SHARED_CPP__QOS_HPP_
