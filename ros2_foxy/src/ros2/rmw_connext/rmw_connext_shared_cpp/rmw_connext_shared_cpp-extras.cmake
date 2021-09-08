# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# copied from rmw_connext_shared_cpp/rmw_connext_shared_cpp-extras.cmake

include(
  "${rmw_connext_shared_cpp_DIR}/get_rmw_connext_output_filter.cmake")

find_package(connext_cmake_module QUIET)
find_package(Connext MODULE QUIET)

if(Connext_FOUND)
  list(APPEND rmw_connext_shared_cpp_DEFINITIONS ${Connext_DEFINITIONS})
  list(APPEND rmw_connext_shared_cpp_INCLUDE_DIRS ${Connext_INCLUDE_DIRS})
  list(APPEND rmw_connext_shared_cpp_LIBRARIES ${Connext_LIBRARIES})
endif()
