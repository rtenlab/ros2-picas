// Copyright 2014-2017 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_CONNEXT_CPP__CONNEXT_STATIC_SUBSCRIBER_INFO_HPP_
#define RMW_CONNEXT_CPP__CONNEXT_STATIC_SUBSCRIBER_INFO_HPP_

#include <atomic>

#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw_connext_shared_cpp/connext_static_event_info.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

#include "ndds/ndds_cpp.h"
#include "ndds/ndds_namespace_cpp.h"

#include "rosidl_typesupport_connext_cpp/message_type_support.h"
#include "rmw/types.h"
#include "rmw/ret_types.h"

class ConnextSubscriberListener;

struct ConnextStaticSubscriberInfo : ConnextCustomEventInfo
{
  DDS::Subscriber * dds_subscriber_;
  ConnextSubscriberListener * listener_;
  DDS::DataReader * topic_reader_;
  DDS::ReadCondition * read_condition_;
  const message_type_support_callbacks_t * callbacks_;
  /// Remap the specific RTI Connext DDS DataReader Status to a generic RMW status type.
  /**
   * \param mask input status mask
   * \param event
   */
  rmw_ret_t get_status(DDS::StatusMask mask, void * event) override;
  /// Return the topic reader entity for this subscriber.
  /**
   * \return the topic reader associated with this subscriber
   */
  DDS::Entity * get_entity() override;
};

class ConnextSubscriberListener : public DDS::SubscriberListener
{
public:
  virtual void on_subscription_matched(
    DDSDataReader *,
    const DDS_SubscriptionMatchedStatus & status)
  {
    current_count_ = status.current_count;
  }

  std::size_t current_count() const
  {
    return current_count_;
  }

private:
  std::atomic<std::size_t> current_count_;
};

#endif  // RMW_CONNEXT_CPP__CONNEXT_STATIC_SUBSCRIBER_INFO_HPP_
