// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rmw_connext_cpp/get_subscriber.hpp"

#include "rmw_connext_cpp/connext_static_subscriber_info.hpp"
#include "rmw_connext_cpp/identifier.hpp"

namespace rmw_connext_cpp
{

DDS::DataReader *
get_data_reader(rmw_subscription_t * subscription)
{
  if (!subscription) {
    return NULL;
  }
  if (subscription->implementation_identifier != rti_connext_identifier) {
    return NULL;
  }
  ConnextStaticSubscriberInfo * impl =
    static_cast<ConnextStaticSubscriberInfo *>(subscription->data);
  return impl->topic_reader_;
}

}  // namespace rmw_connext_cpp
