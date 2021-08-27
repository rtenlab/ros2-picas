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

cmake_minimum_required(VERSION 2.8.3)
project(connext_cmake_module_check_abi CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wl,--no-as-needed)
endif()

include_directories(@Connext_INCLUDE_DIRS@)
add_executable(exe
  "@connext_cmake_module_DIR@/check_abi.cpp")
target_compile_definitions(exe PRIVATE @Connext_DEFINITIONS@)
target_link_libraries(exe @Connext_LIBRARIES@)
set_target_properties(exe
  PROPERTIES CXX_STANDARD 14)
