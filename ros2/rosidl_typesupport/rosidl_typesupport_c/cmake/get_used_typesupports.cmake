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

#
# Get the concrete typesupport names to be used.
#
# :param var: the output variable name for the typesupport names
# :type var: list of strings
# :param typesupport_type: the type of the typesupport to look for
# :type typesupport_type: string
#
# @public
#
function(get_used_typesupports var typesupport_type)
  ament_index_get_resources(rmw_implementations "rmw_typesupport")
  if(rmw_implementations STREQUAL "")
    message(FATAL_ERROR "No RMW implementations found")
  endif()

  # all type supports available
  ament_index_get_resources(available_typesupports "${typesupport_type}")
  if(available_typesupports STREQUAL "")
    message(FATAL_ERROR "No '${typesupport_type}' found")
  endif()

  # supported type supports across all rmw implementations
  set(supported_typesupports "")
  foreach(rmw_implementation IN LISTS rmw_implementations)
    ament_index_get_resource(resource "rmw_typesupport" "${rmw_implementation}")
    foreach(ts IN LISTS resource)
      if(NOT ts IN_LIST supported_typesupports)
        list(APPEND supported_typesupports "${ts}")
      endif()
    endforeach()
  endforeach()

  # supported type supports across all rmw implementations which are available
  set(matching_typesupports "")
  foreach(ts IN LISTS supported_typesupports)
    if(ts IN_LIST available_typesupports)
      list(APPEND matching_typesupports "${ts}")
    endif()
  endforeach()
  list(REMOVE_ITEM matching_typesupports "${typesupport_type}")
  if(matching_typesupports STREQUAL "")
    message(FATAL_ERROR "No '${typesupport_type}' found for the following "
      "rmw implementations: ${rmw_implementations}")
  endif()

  # if across all available rmw implementation only one type support is supported
  # then return only that one which will ignore all other available
  # and enable to bypass the dynamic dispatch
  list(LENGTH matching_typesupports count)
  if(count EQUAL 1)
    message(STATUS "Using single matching ${typesupport_type}: ${matching_typesupports}")
    set(${var} "${matching_typesupports}" PARENT_SCOPE)
  else()
    message(STATUS "Using all available ${typesupport_type}: ${available_typesupports}")
    set(${var} "${available_typesupports}" PARENT_SCOPE)
  endif()
endfunction()
