# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

# Convert .idl files for compatibility with OpenSplice
rosidl_generate_dds_interfaces(
  ${rosidl_generate_interfaces_TARGET}__dds_opensplice_idl
  IDL_TUPLES ${rosidl_generate_interfaces_IDL_TUPLES}
  DEPENDENCY_PACKAGE_NAMES ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES}
  SERVICE_TEMPLATES
    "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/service_request_wrapper.idl.em"
    "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/service_response_wrapper.idl.em"
  OUTPUT_SUBFOLDERS "dds_opensplice"
  EXTENSION "rosidl_typesupport_opensplice_cpp.rosidl_generator_dds_idl_extension"
)

set(_target_suffix "__rosidl_typesupport_opensplice_cpp")

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_opensplice_cpp/${PROJECT_NAME}")
set(_dds_idl_base_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl")

set(_dds_idl_files "")
set(_generated_files "")
set(_generated_external_files "")
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _header_name)
  list(APPEND _dds_idl_files
    "${_dds_idl_base_path}/${PROJECT_NAME}/${_parent_folder}/dds_opensplice/${_idl_name}_.idl")
  list(APPEND _generated_files
    "${_output_path}/${_parent_folder}/dds_opensplice/${_header_name}__type_support.cpp"
    "${_output_path}/${_parent_folder}/${_header_name}__rosidl_typesupport_opensplice_cpp.hpp"
  )
  list(APPEND _generated_external_files
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_.h"
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_.cpp"
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_Dcps.h"
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_Dcps.cpp"
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_Dcps_impl.h"
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_Dcps_impl.cpp"
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_SplDcps.h"
    "${_output_path}/${_parent_folder}/dds_opensplice/${_idl_name}_SplDcps.cpp"
    "${_output_path}/${_parent_folder}/dds_opensplice/ccpp_${_idl_name}_.h")
endforeach()

# If not on Windows, disable some warnings with OpenSplice's generated code
if(NOT WIN32)
  set(_opensplice_compile_flags)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(_opensplice_compile_flags
      "-Wno-unused-but-set-variable"
    )
  endif()
  if(NOT _opensplice_compile_flags STREQUAL "")
    string(REPLACE ";" " " _opensplice_compile_flags "${_opensplice_compile_flags}")
    foreach(_gen_file ${_generated_external_files})
      set_source_files_properties("${_gen_file}"
        PROPERTIES COMPILE_FLAGS "${_opensplice_compile_flags}")
    endforeach()
  endif()
endif()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    # ${{_pkg_name}_DIR} is absolute path ending in 'share/<pkg_name>/cmake', so go back one
    # directory for IDL files
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_typesupport_opensplice_cpp_BIN}"
  ${rosidl_typesupport_opensplice_cpp_GENERATOR_FILES}
  "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/idl__rosidl_typesupport_opensplice_cpp.hpp.em"
  "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/idl__dds_opensplice__type_support.cpp.em"
  "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/msg__rosidl_typesupport_opensplice_cpp.hpp.em"
  "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/msg__type_support.cpp.em"
  "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/srv__rosidl_typesupport_opensplice_cpp.hpp.em"
  "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/srv__type_support.cpp.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    get_property(is_generated SOURCE "${dep}" PROPERTY GENERATED)
    if(NOT ${_is_generated})
      message(FATAL_ERROR "Target dependency '${dep}' does not exist")
    endif()
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_opensplice_cpp__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
  ADDITIONAL_FILES ${_dds_idl_files}
)

add_custom_command(
  OUTPUT
  ${_generated_files}
  ${_generated_external_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_typesupport_opensplice_cpp_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --dds-interface-base-path "${_dds_idl_base_path}"
  --idl-pp "${OpenSplice_IDLPP}"
  DEPENDS ${target_dependencies} ${_dds_idl_files}
  COMMENT "Generating C++ type support for PrismTech OpenSplice"
  VERBATIM
)

# generate header to switch between export and import for a specific package on Windows
set(_visibility_control_file
  "${_output_path}/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
configure_file(
  "${rosidl_typesupport_opensplice_cpp_TEMPLATE_DIR}/visibility_control.h.in"
  "${_visibility_control_file}"
  @ONLY
)
list(APPEND _generated_files "${_visibility_control_file}")

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(
    DIRECTORY "${_output_path}/"
    DESTINATION "include/${PROJECT_NAME}"
    PATTERN "*.cpp" EXCLUDE
  )
  if(
    NOT _generated_files STREQUAL "" OR
    NOT _generated_external_files STREQUAL ""
  )
    ament_export_include_directories(include)
  endif()
endif()

link_directories(${OpenSplice_LIBRARY_DIRS})
add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} SHARED
  ${_generated_files}
  ${_generated_external_files})
if(rosidl_generate_interfaces_LIBRARY_NAME)
  set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PROPERTIES OUTPUT_NAME "${rosidl_generate_interfaces_LIBRARY_NAME}${_target_suffix}")
endif()
set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PROPERTIES CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PROPERTIES COMPILE_FLAGS "-Wall -Wextra -Wpedantic")
endif()
if(WIN32)
  target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PRIVATE "ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_BUILDING_DLL_${PROJECT_NAME}")
endif()
target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_opensplice_cpp
)
ament_target_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  "OpenSplice"
  "rmw"
  "rosidl_typesupport_interface"
  "rosidl_typesupport_opensplice_cpp")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  set(_msg_include_dir "${${_pkg_name}_DIR}/../../../include/${_pkg_name}/msg/dds_opensplice")
  normalize_path(_msg_include_dir "${_msg_include_dir}")
  target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PUBLIC
    "${_msg_include_dir}"
  )
  ament_target_dependencies(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${_pkg_name})
  target_link_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${${_pkg_name}_LIBRARIES${_target_suffix}})
endforeach()

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__cpp
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__dds_opensplice_idl
)

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(
    TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  rosidl_export_typesupport_libraries(${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix})
endif()

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  if(NOT _generated_files STREQUAL "")
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(
      TESTNAME "cppcheck_rosidl_typesupport_opensplice_cpp"
      ${_generated_files})

    find_package(ament_cmake_cpplint REQUIRED)
    get_filename_component(_cpplint_root "${_output_path}" DIRECTORY)
    ament_cpplint(
      TESTNAME "cpplint_rosidl_typesupport_opensplice_cpp"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ROOT "${_cpplint_root}"
      ${_generated_files})

    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify(
      TESTNAME "uncrustify_rosidl_typesupport_opensplice_cpp"
      # the generated code might contain longer lines for templated types
      # set the value to zero to tell uncrustify to ignore line lengths
      MAX_LINE_LENGTH 0
      ${_generated_files})
  endif()
endif()
