cmake_minimum_required(VERSION 3.5)

project(rmw_connext_shared_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(connext_cmake_module REQUIRED)
find_package(Connext QUIET MODULE)
if(Connext_FOUND)
  find_package(Connext MODULE)
endif()
if(NOT Connext_FOUND)
  ament_package()
  message(WARNING "Could not find RTI Connext - skipping '${PROJECT_NAME}'")
  return()
endif()
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

include_directories(include)

ament_export_include_directories(include)

ament_export_dependencies(rmw)

link_directories(${Connext_LIBRARY_DIRS})
add_library(
  rmw_connext_shared_cpp
  SHARED
  src/condition_error.cpp
  src/count.cpp
  src/demangle.cpp
  src/event.cpp
  src/event_converter.cpp
  src/guard_condition.cpp
  src/init.cpp
  src/namespace_prefix.cpp
  src/node.cpp
  src/node_names.cpp
  src/qos.cpp
  src/names_and_types_helpers.cpp
  src/node_info_and_types.cpp
  src/security_logging.cpp
  src/service_names_and_types.cpp
  src/topic_names_and_types.cpp
  src/trigger_guard_condition.cpp
  src/wait_set.cpp
  src/types/custom_data_reader_listener.cpp
  src/types/custom_publisher_listener.cpp
  src/types/custom_subscriber_listener.cpp
  src/topic_endpoint_info.cpp)
ament_target_dependencies(rmw_connext_shared_cpp
  "rcpputils"
  "rcutils"
  "rmw"
  "Connext")
ament_export_libraries(rmw_connext_shared_cpp)

if(WIN32)
  # Causes the visibility macros to use dllexport rather than dllimport
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(rmw_connext_shared_cpp
    PRIVATE "RMW_CONNEXT_SHARED_CPP_BUILDING_DLL"
            "_CRT_NONSTDC_NO_DEPRECATE")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${rmw_INCLUDE_DIRS})
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package(
  CONFIG_EXTRAS "rmw_connext_shared_cpp-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rmw_connext_shared_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
