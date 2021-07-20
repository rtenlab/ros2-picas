// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCL_LOGGING_LOG4CXX__LOGGING_INTERFACE_H_
#define RCL_LOGGING_LOG4CXX__LOGGING_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "rcl_logging_log4cxx/visibility_control.h"

typedef int rcl_logging_ret_t;

RCL_LOGGING_PUBLIC
rcl_logging_ret_t rcl_logging_external_initialize(
  const char * config_file,
  rcutils_allocator_t allocator);

RCL_LOGGING_PUBLIC
rcl_logging_ret_t rcl_logging_external_shutdown();

RCL_LOGGING_PUBLIC
void rcl_logging_external_log(int severity, const char * name, const char * msg);

RCL_LOGGING_PUBLIC
rcl_logging_ret_t rcl_logging_external_set_logger_level(const char * name, int level);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // RCL_LOGGING_LOG4CXX__LOGGING_INTERFACE_H_
