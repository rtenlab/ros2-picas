cmake_minimum_required(VERSION 3.5)

project(rmw_connext_cpp)

if(DEFINED ENV{CONNEXT_STATIC_DISABLE})
  set(disable_static_connext $ENV{CONNEXT_STATIC_DISABLE})
else()
  set(disable_static_connext FALSE)
endif()
set(CONNEXT_STATIC_DISABLE ${disable_static_connext}
  CACHE BOOL "If Connext Static should be disabled.")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(connext_cmake_module REQUIRED)
find_package(Connext QUIET MODULE)
if(Connext_FOUND)
  find_package(Connext MODULE)
endif()
if(NOT Connext_FOUND OR CONNEXT_STATIC_DISABLE)
  ament_package()
  if(NOT Connext_FOUND)
    message(WARNING "Could not find RTI Connext - skipping '${PROJECT_NAME}'")
  else()
    message(STATUS "Connext static rmw implementation explicitly disabled - skipping '${PROJECT_NAME}'")
  endif()
  return()
endif()

find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)

find_package(rosidl_typesupport_connext_c)
if(NOT rosidl_typesupport_connext_c_FOUND)
  ament_package()
  message(WARNING "Could not find ROSIDL TypeSupport for Connext (rosidl_typesupport_connext_c) - skipping '${PROJECT_NAME}'")
  return()
endif()

find_package(rosidl_typesupport_connext_cpp)
if(NOT rosidl_typesupport_connext_cpp_FOUND)
  ament_package()
  message(WARNING "Could not find ROSIDL TypeSupport for Connext (rosidl_typesupport_connext_cpp) - skipping '${PROJECT_NAME}'")
  return()
endif()

find_package(rmw REQUIRED)
find_package(rmw_connext_shared_cpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

include_directories(include)

ament_export_include_directories(include)
ament_export_dependencies(
  rcpputils
  rcutils
  rmw
  rmw_connext_shared_cpp
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_connext_c
  rosidl_typesupport_connext_cpp)

register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_connext_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_connext_cpp"
)

# generate code for raw data
set(generated_directory "${CMAKE_CURRENT_BINARY_DIR}/resources/generated")
file(MAKE_DIRECTORY ${generated_directory})
set(generated_files
  ${generated_directory}/connext_static_serialized_data.cxx
  ${generated_directory}/connext_static_serialized_data.h
  ${generated_directory}/connext_static_serialized_dataPlugin.cxx
  ${generated_directory}/connext_static_serialized_dataPlugin.h
  ${generated_directory}/connext_static_serialized_dataSupport.cxx
  ${generated_directory}/connext_static_serialized_dataSupport.h
)
add_custom_command(
  OUTPUT ${generated_files}
  # We *explicitly* do not use Connext_DDSGEN_SERVER here.  The other time we invoke the
  # server is in rosidl_typesupport_connext_cpp, and that uses a different set of arguments.
  # We don't want to accidentally connect to that other version of the server that might
  # still be running.
  COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/bin/call-idl-pp.py" --idl-pp "${Connext_DDSGEN}" --idl-file "connext_static_serialized_data.idl" -d ${generated_directory}
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources"
  COMMENT "Generating serialized type support for RTI Connext (using '${Connext_DDSGEN}')"
  VERBATIM
)

# determine major version of Connext
find_file(connext_version_header "ndds_version.h"
  PATHS ${Connext_INCLUDE_DIRS}
  PATH_SUFFIXES "ndds"
  NO_DEFAULT_PATH)
if(NOT connext_version_header)
  message(FATAL_ERROR "Failed to find 'ndds/ndds_version.h' in '${Connext_INCLUDE_DIRS}'")
endif()
file(STRINGS "${connext_version_header}" connext_define_major_version LIMIT_COUNT 1 REGEX "#define RTI_DDS_VERSION_MAJOR  [0-9]+")
if("${connext_define_major_version}" STREQUAL "")
  message(FATAL_ERROR "Failed to find '#define RTI_DDS_VERSION_MAJOR' in '${connext_version_header}'")
endif()
string(REGEX REPLACE ".* ([0-9]+)" "\\1" connext_major_version "${connext_define_major_version}")

# patch the generate code for raw data
if("${connext_major_version}" LESS 6)
  set(patch_files_subdirectory "patch_files")
else()
  set(patch_files_subdirectory "patch_files_v6")
endif()
set(patch_files
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/${patch_files_subdirectory}/connext_static_serialized_data.cxx.patch
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/${patch_files_subdirectory}/connext_static_serialized_data.h.patch
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/${patch_files_subdirectory}/connext_static_serialized_dataPlugin.cxx.patch
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/${patch_files_subdirectory}/connext_static_serialized_dataPlugin.h.patch
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/${patch_files_subdirectory}/connext_static_serialized_dataSupport.cxx.patch
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/${patch_files_subdirectory}/connext_static_serialized_dataSupport.h.patch
)
set(patched_directory "${CMAKE_CURRENT_BINARY_DIR}/resources/patched")
file(MAKE_DIRECTORY ${patched_directory})
set(patched_files
  ${patched_directory}/connext_static_serialized_data.cxx
  ${patched_directory}/connext_static_serialized_data.h
  ${patched_directory}/connext_static_serialized_dataPlugin.cxx
  ${patched_directory}/connext_static_serialized_dataPlugin.h
  ${patched_directory}/connext_static_serialized_dataSupport.cxx
  ${patched_directory}/connext_static_serialized_dataSupport.h
)
add_custom_command(
  OUTPUT ${patched_files}
  COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/bin/apply-patch.py" --input ${generated_files} --patch ${patch_files} --out ${patched_files}
  DEPENDS ${generated_files}
  COMMENT "Patching serialized type support for RTI Connext"
  VERBATIM
)

add_library(
  rmw_connext_cpp
  SHARED
  ${patched_files}
  src/connext_static_publisher_info.cpp
  src/connext_static_subscriber_info.cpp
  src/get_client.cpp
  src/get_participant.cpp
  src/get_publisher.cpp
  src/get_service.cpp
  src/get_subscriber.cpp
  src/identifier.cpp
  src/process_topic_and_service_names.cpp
  src/rmw_client.cpp
  src/rmw_compare_gid_equals.cpp
  src/rmw_count.cpp
  src/rmw_event.cpp
  src/rmw_get_gid_for_publisher.cpp
  src/rmw_get_implementation_identifier.cpp
  src/rmw_get_serialization_format.cpp
  src/rmw_guard_condition.cpp
  src/rmw_init.cpp
  src/rmw_logging.cpp
  src/rmw_node.cpp
  src/rmw_node_info_and_types.cpp
  src/rmw_node_names.cpp
  src/rmw_publish.cpp
  src/rmw_publisher.cpp
  src/rmw_request.cpp
  src/rmw_response.cpp
  src/rmw_serialize.cpp
  src/rmw_service.cpp
  src/rmw_service_names_and_types.cpp
  src/rmw_service_server_is_available.cpp
  src/rmw_subscription.cpp
  src/rmw_take.cpp
  src/rmw_topic_names_and_types.cpp
  src/rmw_trigger_guard_condition.cpp
  src/rmw_wait.cpp
  src/rmw_wait_set.cpp
  src/serialization_format.cpp
  src/rmw_get_topic_endpoint_info.cpp)
ament_target_dependencies(rmw_connext_cpp
  "rcpputils"
  "rcutils"
  "rmw"
  "rmw_connext_shared_cpp"
  "rosidl_runtime_c"
  "rosidl_runtime_cpp"
  "rosidl_typesupport_connext_c"
  "rosidl_typesupport_connext_cpp"
  "Connext")
target_include_directories(rmw_connext_cpp PUBLIC ${patched_directory})
ament_export_libraries(rmw_connext_cpp)

# Causes the visibility macros to use dllexport rather than dllimport
# which is appropriate when building the library but not consuming it.
target_compile_definitions(rmw_connext_cpp
  PRIVATE "RMW_CONNEXT_CPP_BUILDING_DLL")

# On Windows this adds the RMW_BUILDING_DLL definition.
# On Unix (GCC or Clang) it hides the symbols with -fvisibility=hidden.
configure_rmw_library(rmw_connext_cpp)

if(WIN32)
  target_compile_definitions(rmw_connext_cpp
    PRIVATE "_CRT_NONSTDC_NO_DEPRECATE")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rmw_connext_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

get_rmw_connext_output_filter("rmw_connext_output_filter")
ament_index_register_resource("rmw_output_prefixes" CONTENT "${rmw_connext_output_filter}")
