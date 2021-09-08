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

#ifndef QOS_HPP_
#define QOS_HPP_

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
#include <u_instanceHandle.h>

#include "rmw/rmw.h"

RMW_LOCAL
bool
get_datareader_qos(
  DDS::Subscriber * subscriber,
  const rmw_qos_profile_t & qos_profile,
  DDS::DataReaderQos & datareader_qos);

RMW_LOCAL
bool
get_datawriter_qos(
  DDS::Publisher * publisher,
  const rmw_qos_profile_t & qos_profile,
  DDS::DataWriterQos & datawriter_qos);

template<typename AttributeT>
void
dds_qos_to_rmw_qos(
  const AttributeT & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_LOCAL
void
dds_qos_to_rmw_qos<DDS::DataWriterQos>(
  const DDS::DataWriterQos & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_LOCAL
void
dds_qos_to_rmw_qos<DDS::DataReaderQos>(
  const DDS::DataReaderQos & dds_qos,
  rmw_qos_profile_t * qos);

#endif  // QOS_HPP_
