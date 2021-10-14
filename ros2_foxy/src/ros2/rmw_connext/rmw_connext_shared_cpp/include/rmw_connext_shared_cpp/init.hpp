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

#ifndef RMW_CONNEXT_SHARED_CPP__INIT_HPP_
#define RMW_CONNEXT_SHARED_CPP__INIT_HPP_

#include "rmw/types.h"

#include "rmw_connext_shared_cpp/visibility_control.h"

struct rmw_context_impl_t
{
  // Shutdown flag
  bool is_shutdown{false};
};

RMW_CONNEXT_SHARED_CPP_PUBLIC
rmw_ret_t init();

#endif  // RMW_CONNEXT_SHARED_CPP__INIT_HPP_
