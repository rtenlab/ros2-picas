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

#include "rmw_connext_cpp/get_publisher.hpp"

#include "rmw_connext_cpp/connext_static_publisher_info.hpp"
#include "rmw_connext_cpp/identifier.hpp"

namespace rmw_connext_cpp
{

DDS::DataWriter *
get_data_writer(rmw_publisher_t * publisher)
{
  if (!publisher) {
    return NULL;
  }
  if (publisher->implementation_identifier != rti_connext_identifier) {
    return NULL;
  }
  ConnextStaticPublisherInfo * impl = static_cast<ConnextStaticPublisherInfo *>(publisher->data);
  return impl->topic_writer_;
}

}  // namespace rmw_connext_cpp
