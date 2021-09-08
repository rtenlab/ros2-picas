cmake_minimum_required(VERSION 3.5)

project(rmw_opensplice_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(opensplice_cmake_module REQUIRED)
find_package(OpenSplice MODULE)
if(NOT OpenSplice_FOUND)
  ament_package()
  message(WARNING "Could not find ADLINK OpenSplice - skipping '${PROJECT_NAME}'")
  return()
endif()

find_package(rosidl_typesupport_opensplice_c)
find_package(rosidl_typesupport_opensplice_cpp)
if(NOT rosidl_typesupport_opensplice_cpp_FOUND OR NOT rosidl_typesupport_opensplice_c_FOUND)
  ament_package()
  message(WARNING "Could not find ROSIDL TypeSupport for OpenSplice (rosidl_typesupport_opensplice_cpp or rosidl_typesupport_opensplice_c) - skipping '${PROJECT_NAME}'")
  return()
endif()

find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

ament_export_dependencies(
  rcutils
  rmw
  rosidl_generator_c
  rosidl_generator_cpp
  rosidl_typesupport_opensplice_c
  rosidl_typesupport_opensplice_cpp
)

link_directories(${OpenSplice_LIBRARY_DIRS})
add_library(rmw_opensplice_cpp SHARED
  src/demangle.cpp
  src/event_converter.cpp
  src/identifier.cpp
  src/namespace_prefix.cpp
  src/qos.cpp
  src/rmw_logging.cpp
  src/rmw_client.cpp
  src/rmw_compare_gids_equal.cpp
  src/rmw_count.cpp
  src/rmw_event.cpp
  src/rmw_get_gid_for_publisher.cpp
  src/rmw_get_implementation_identifier.cpp
  src/rmw_get_serialization_format.cpp
  src/rmw_guard_condition.cpp
  src/rmw_init.cpp
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
  src/types.cpp
)
ament_target_dependencies(rmw_opensplice_cpp
  "rcpputils"
  "rcutils"
  "rmw"
  "rosidl_generator_c"
  "rosidl_generator_cpp"
  "rosidl_typesupport_opensplice_c"
  "rosidl_typesupport_opensplice_cpp"
  "OpenSplice"
)
# On Windows this adds the RMW_BUILDING_DLL definition.
# On Unix (GCC or Clang) it hides the symbols with -fvisibility=hidden.
configure_rmw_library(rmw_opensplice_cpp)

if(WIN32)
  target_compile_definitions(rmw_opensplice_cpp
    PRIVATE "_CRT_NONSTDC_NO_DEPRECATE")
endif()

ament_export_libraries(rmw_opensplice_cpp)

register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_opensplice_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_opensplice_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${rmw_INCLUDE_DIRS})
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake")

install(
  TARGETS rmw_opensplice_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
