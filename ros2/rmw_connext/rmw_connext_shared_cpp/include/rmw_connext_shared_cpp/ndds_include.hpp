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

#ifndef RMW_CONNEXT_SHARED_CPP__NDDS_INCLUDE_HPP_
#define RMW_CONNEXT_SHARED_CPP__NDDS_INCLUDE_HPP_

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wpedantic"
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#include <ndds/ndds_namespace_cpp.h>
#include <ndds/ndds_requestreply_cpp.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#endif  // RMW_CONNEXT_SHARED_CPP__NDDS_INCLUDE_HPP_
