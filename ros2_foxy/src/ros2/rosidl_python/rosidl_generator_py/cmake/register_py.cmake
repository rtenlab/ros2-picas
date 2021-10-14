# Copyright 2015 Open Source Robotics Foundation, Inc.
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

macro(rosidl_generator_py_extras BIN GENERATOR_FILES TEMPLATE_DIR)
  find_package(ament_cmake_core QUIET REQUIRED)
  # Make sure extension points are registered in order
  find_package(rosidl_generator_c QUIET REQUIRED)
  find_package(rosidl_typesupport_c QUIET REQUIRED)

  ament_register_extension(
    "rosidl_generate_idl_interfaces"
    "rosidl_generator_py"
    "rosidl_generator_py_generate_interfaces.cmake")

  normalize_path(BIN "${BIN}")
  set(rosidl_generator_py_BIN "${BIN}")

  set(rosidl_generator_py_GENERATOR_FILES "")
  foreach(_generator_file ${GENERATOR_FILES})
    normalize_path(_generator_file "${_generator_file}")
    list(APPEND rosidl_generator_py_GENERATOR_FILES "${_generator_file}")
  endforeach()

  normalize_path(TEMPLATE_DIR "${TEMPLATE_DIR}")
  set(rosidl_generator_py_TEMPLATE_DIR "${TEMPLATE_DIR}")
endmacro()
