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

# copied from rmw_opensplice_cpp/rmw_opensplice_cpp-extras.cmake

find_package(opensplice_cmake_module QUIET)
find_package(OpenSplice MODULE QUIET)

if(NOT OpenSplice_FOUND)
  message(STATUS
    "Could not find ADLINK OpenSplice - skipping rmw_opensplice_cpp")
  set(rmw_opensplice_cpp_FOUND FALSE)
else()
  list(APPEND rmw_opensplice_cpp_DEFINITIONS ${OpenSplice_DEFINITIONS})
  list(APPEND rmw_opensplice_cpp_INCLUDE_DIRS ${OpenSplice_INCLUDE_DIRS})
  foreach(_library ${OpenSplice_LIBRARIES})
    # ensure to add libraries with absolute paths
    if(NOT IS_ABSOLUTE "${_library}")
      set(_lib "NOTFOUND")
      find_library(
        _lib "${_library}"
        HINTS ${OpenSplice_LIBRARY_DIRS})
      if(NOT _lib)
        message(FATAL_ERROR "OpenSplice exports the library '${_library}' which couldn't be found")
      elseif(NOT IS_ABSOLUTE "${_lib}")
        message(FATAL_ERROR "The OpenSplice library '${_library}' was found at '${_lib}' which is not an absolute path")
      elseif(NOT EXISTS "${_lib}")
        message(FATAL_ERROR "The OpenSplice library '${_library}' was found at '${_lib}' which doesn't exist")
      endif()
      set(_library "${_lib}")
    endif()
    list(APPEND rmw_opensplice_cpp_LIBRARIES "${_library}")
  endforeach()
endif()
