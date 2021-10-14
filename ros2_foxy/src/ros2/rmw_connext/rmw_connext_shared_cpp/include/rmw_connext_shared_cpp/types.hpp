// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_CONNEXT_SHARED_CPP__TYPES_HPP_
#define RMW_CONNEXT_SHARED_CPP__TYPES_HPP_

#include <cassert>
#include <exception>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rmw/rmw.h"
#include "topic_cache.hpp"
#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw_connext_shared_cpp/visibility_control.h"


enum EntityType {Publisher, Subscriber};

using DDSTopicEndpointInfo = TopicCache<DDS::GUID_t>::TopicInfo;

class CustomDataReaderListener
  : public DDS::DataReaderListener
{
public:
  explicit
  CustomDataReaderListener(
    const char * implementation_identifier, rmw_guard_condition_t * graph_guard_condition)
  : graph_guard_condition_(graph_guard_condition),
    implementation_identifier_(implementation_identifier)
  {}

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  bool add_information(
    const DDS::GUID_t & participant_guid,
    const DDS::GUID_t & guid,
    const std::string & topic_name,
    const std::string & type_name,
    const rmw_qos_profile_t & qos_profile,
    EntityType entity_type);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  bool remove_information(
    const DDS::GUID_t & guid,
    EntityType entity_type);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  bool add_information(
    const DDS::InstanceHandle_t & participant_instance_handle,
    const DDS::InstanceHandle_t & instance_handle,
    const std::string & topic_name,
    const std::string & type_name,
    const rmw_qos_profile_t & qos_profile,
    EntityType entity_type);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  bool remove_information(
    const DDS::InstanceHandle_t & instance_handle,
    EntityType entity_type);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  bool trigger_graph_guard_condition();

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  size_t count_topic(const std::string & topic_name);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  void fill_topic_endpoint_infos(
    const std::string & topic_name,
    bool no_mangle,
    std::vector<const DDSTopicEndpointInfo *> & topic_endpoint_infos);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  void fill_topic_names_and_types(
    bool no_demangle,
    std::map<std::string, std::set<std::string>> & topic_names_to_types);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  void fill_service_names_and_types(
    std::map<std::string, std::set<std::string>> & services);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  void fill_topic_names_and_types_by_guid(
    bool no_demangle,
    std::map<std::string, std::set<std::string>> & topic_names_to_types_by_guid,
    DDS_GUID_t & participant_guid);

  RMW_CONNEXT_SHARED_CPP_PUBLIC
  void fill_service_names_and_types_by_guid(
    std::map<std::string, std::set<std::string>> & services,
    DDS_GUID_t & participant_guid,
    const std::string & suffix);

protected:
  std::mutex mutex_;
  TopicCache<DDS::GUID_t> topic_cache;

private:
  rmw_guard_condition_t * graph_guard_condition_;
  const char * implementation_identifier_;
};

class CustomPublisherListener
  : public CustomDataReaderListener
{
public:
  CustomPublisherListener(
    const char * implementation_identifier, rmw_guard_condition_t * graph_guard_condition)
  : CustomDataReaderListener(implementation_identifier, graph_guard_condition)
  {}

  virtual void on_data_available(DDS::DataReader * reader);
};

class CustomSubscriberListener
  : public CustomDataReaderListener
{
public:
  CustomSubscriberListener(
    const char * implementation_identifier, rmw_guard_condition_t * graph_guard_condition)
  : CustomDataReaderListener(implementation_identifier, graph_guard_condition)
  {}

  virtual void on_data_available(DDS::DataReader * reader);
};

struct ConnextNodeInfo
{
  DDS::DomainParticipant * participant;
  CustomPublisherListener * publisher_listener;
  CustomSubscriberListener * subscriber_listener;
  rmw_guard_condition_t * graph_guard_condition;
};

struct ConnextPublisherGID
{
  DDS::InstanceHandle_t publication_handle;
};

struct ConnextWaitSetInfo
{
  DDS::WaitSet * wait_set;
  DDS::ConditionSeq * active_conditions;
  DDS::ConditionSeq * attached_conditions;
};

#endif  // RMW_CONNEXT_SHARED_CPP__TYPES_HPP_
