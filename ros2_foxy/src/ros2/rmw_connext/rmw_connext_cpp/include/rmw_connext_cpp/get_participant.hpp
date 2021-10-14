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

#ifndef RMW_CONNEXT_CPP__GET_PARTICIPANT_HPP_
#define RMW_CONNEXT_CPP__GET_PARTICIPANT_HPP_

#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw/rmw.h"
#include "rmw_connext_cpp/visibility_control.h"

namespace rmw_connext_cpp
{

/// Return a native Connext participant handle.
/**
 * The function returns `NULL` when either the node handle is `NULL` or when the
 * node handle is from a different rmw implementation.
 *
 * \return native Connext participant handle if successful, otherwise `NULL`
 */
RMW_CONNEXT_CPP_PUBLIC
DDS::DomainParticipant *
get_participant(rmw_node_t * node);

}  // namespace rmw_connext_cpp

#endif  // RMW_CONNEXT_CPP__GET_PARTICIPANT_HPP_
